#!/usr/bin/env python3
"""The camera's MQTT client must hold ONE connection open, not reconnect per publish.

Run from the repo root:  python3 tests/test_mqtt_persistent.py

Verified against a fake broker that speaks just enough MQTT 3.1.1 to count
connections, so the property under test — connection reuse — is measured
directly rather than inferred from the code.
"""
import os, socket, sys, tempfile, threading, time

TD = tempfile.mkdtemp(prefix='mqtttest-')
os.environ['NINTI_CONFIG'] = TD + '/cfg.json'
os.environ['NINTI_EVENT_DIR'] = TD + '/events'
os.environ['NINTI_CLIP_DIR'] = TD + '/clips'
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import sidecar as S

fails = []


class FakeBroker(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.srv = socket.socket()
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(('127.0.0.1', 0))
        self.srv.listen(8)
        self.port = self.srv.getsockname()[1]
        self.connects = 0
        self.publishes = []
        self.disconnects = 0
        self.pings = 0
        self.answer_pings = True
        self.will = None
        self.client_id = None
        self.keepalive = None
        self.live = None
        self.stop = False
        self.lock = threading.Lock()

    def run(self):
        while not self.stop:
            try:
                c, _ = self.srv.accept()
            except OSError:
                return
            threading.Thread(target=self._serve, args=(c,), daemon=True).start()

    def _rl(self, c):
        mult, val = 1, 0
        while True:
            b = c.recv(1)
            if not b:
                raise ConnectionError
            val += (b[0] & 127) * mult
            if not b[0] & 128:
                return val
            mult *= 128

    def _readn(self, c, n):
        buf = b''
        while len(buf) < n:
            part = c.recv(n - len(buf))
            if not part:
                raise ConnectionError
            buf += part
        return buf

    def _serve(self, c):
        try:
            while True:
                hdr = c.recv(1)
                if not hdr:
                    return
                n = self._rl(c)
                body = self._readn(c, n) if n else b''
                typ = hdr[0] >> 4
                if typ == 1:                                  # CONNECT
                    p = 10                                    # skip protocol header
                    flags = body[7]
                    self.keepalive = int.from_bytes(body[8:10], 'big')
                    def take(off):
                        ln = int.from_bytes(body[off:off + 2], 'big')
                        return body[off + 2:off + 2 + ln].decode(), off + 2 + ln
                    cid, p = take(p)
                    will = None
                    if flags & 0x04:
                        wt, p = take(p)
                        wm, p = take(p)
                        will = (wt, wm)
                    with self.lock:
                        self.connects += 1
                        self.client_id = cid
                        self.will = will
                        self.live = c
                    c.sendall(b'\x20\x02\x00\x00')            # CONNACK, accepted
                elif typ == 3:                                # PUBLISH
                    tl = int.from_bytes(body[0:2], 'big')
                    with self.lock:
                        self.publishes.append(body[2:2 + tl].decode())
                elif typ == 12:                               # PINGREQ
                    with self.lock:
                        self.pings += 1
                        answer = self.answer_pings
                    if answer:
                        c.sendall(b'\xd0\x00')                # PINGRESP
                elif typ == 14:                               # DISCONNECT
                    with self.lock:
                        self.disconnects += 1
                    return
        except Exception:
            return
        finally:
            try:
                c.close()
            except Exception:
                pass

    def drop_live(self):
        # shutdown() first: close() alone does not send a FIN while the serving
        # thread still holds a reference to the socket, so the client would see
        # nothing and the test would be measuring the test.
        with self.lock:
            c, self.live = self.live, None
        if c:
            try:
                c.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            try:
                c.close()
            except Exception:
                pass


b = FakeBroker()
b.start()
S.cfg.update({'mqtt_enabled': True, 'mqtt_host': '127.0.0.1', 'mqtt_port': b.port,
              'mqtt_username': '', 'mqtt_password': '', 'mqtt_base_topic': 'nintidetect/test'})

# 1. ten publishes must share one connection
for i in range(10):
    if not S._mqtt_publish_many([('nintidetect/test/status', {'n': i}, False)]):
        fails.append('publish %d failed' % i)
    time.sleep(0.05)
time.sleep(0.3)
print('after 10 publishes: connects=%d publishes=%d' % (b.connects, len(b.publishes)))
if b.connects != 1:
    fails.append('expected 1 connection for 10 publishes, got %d' % b.connects)
if len(b.publishes) != 10:
    fails.append('expected 10 publishes, broker saw %d' % len(b.publishes))

# 2. the CONNECT must carry a retained will, a sane keepalive and the device id
print('client_id=%s keepalive=%s will=%s' % (b.client_id, b.keepalive, b.will))
if not b.client_id or not b.client_id.startswith('nintidetect-'):
    fails.append('unexpected client id %r' % b.client_id)
if b.will != ('nintidetect/test/availability', 'offline'):
    fails.append('expected an offline Last Will, got %r' % (b.will,))
if b.keepalive != S.MQTT_KEEPALIVE:
    fails.append('keepalive %r != MQTT_KEEPALIVE' % b.keepalive)

# 3. a broker-side drop must be recovered on the next publish, not lost
b.drop_live()
time.sleep(0.6)
ok = S._mqtt_publish_many([('nintidetect/test/status', {'after': 'drop'}, False)])
time.sleep(0.3)
print('after broker drop: publish_ok=%s connects=%d' % (ok, b.connects))
if not ok:
    fails.append('publish after a broker-side drop should have reconnected and succeeded')
if b.connects != 2:
    fails.append('expected exactly 1 reconnect, got %d connections total' % b.connects)

# 4. REGRESSION: a busy link must still be pinged. The first version keyed the
#    ping off "time since last send", and the 5s status publishes meant it never
#    fired, so no PINGRESP ever arrived and the liveness check tore down a
#    healthy connection every 150s on the real device.
S.MQTT_PING_INTERVAL = 1.0
S.MQTT_PING_TIMEOUT = 4.0
S._mqtt_ping_next = 0
threading.Thread(target=S._mqtt_keepalive_loop, daemon=True).start()
before, pings_before = b.connects, b.pings
for _ in range(24):                       # keep publishing the whole time
    S._mqtt_publish_many([('nintidetect/test/status', {'busy': 1}, False)])
    time.sleep(0.3)
time.sleep(6)
print('busy link: pings=%d (was %d) connects=%d (was %d)'
      % (b.pings, pings_before, b.connects, before))
if b.pings - pings_before < 1:
    fails.append('no PINGREQ while publishing — publish traffic is suppressing the ping')
if b.connects != before:
    fails.append('keepalive opened %d extra connection(s) on a healthy link'
                 % (b.connects - before))

# 4b. a broker that stops answering pings must be detected, once
b.answer_pings = False
before = b.connects
time.sleep(S.MQTT_PING_TIMEOUT + 12)
b.answer_pings = True
time.sleep(3)
print('unanswered pings: connects went %d -> %d' % (before, b.connects))
if b.connects <= before:
    fails.append('a broker that stopped answering PINGREQ was never detected')
if b.connects - before > 3:
    fails.append('reconnect storm after a dead link: %d new connections'
                 % (b.connects - before))

# 5. shutdown must send DISCONNECT, so the broker logs a clean close
S.mqtt_shutdown()
time.sleep(0.4)
print('disconnects seen by broker: %d' % b.disconnects)
if b.disconnects < 1:
    fails.append('shutdown did not send a DISCONNECT packet')

# 6. a dead broker must back off, not hammer
b.stop = True
b.srv.close()
S.cfg['mqtt_port'] = 1           # nothing listening
S._mqtt_retry_at = 0
S._mqtt_retry_delay = S.MQTT_RETRY_MIN
t0 = time.time()
attempts = 0
orig = socket.create_connection
def counting(*a, **k):
    global attempts
    attempts += 1
    raise OSError('refused')
socket.create_connection = counting
for _ in range(40):
    S._mqtt_publish_many([('nintidetect/test/status', {}, False)])
    time.sleep(0.1)
socket.create_connection = orig
print('4s of publishes against a dead broker -> %d connection attempts' % attempts)
if attempts > 4:
    fails.append('no backoff: %d connect attempts in %.1fs' % (attempts, time.time() - t0))

import shutil
shutil.rmtree(TD, ignore_errors=True)
print()
if fails:
    print('FAIL:')
    for f in fails:
        print('  - ' + f)
    sys.exit(1)
print('ALL CHECKS PASSED')
