# Deploy Checklist — lichee-cam / NintiDetect

Device: `192.168.1.121` (LicheeRV Nano, cv181x)
Access: `ssh root@192.168.1.121`

---

## Sidecar-only change

**Requires reboot** — sidecar restart leaves stream_yolo stuck in `EnterVcodecLock` (VENC hardware state not reset). Use reboot for any sidecar change.

```bash
scp sidecar.py root@192.168.1.121:/root/sidecar.py
ssh root@192.168.1.121 'reboot'
```

---

## Binary + sidecar change

```bash
# Build
LICHEE=/home/thylation/Desktop/lichee-cam
RISCV=$LICHEE/build-env/host-tools/gcc/riscv64-linux-musl-x86_64
COMPILER=$RISCV/bin SDK_PATH=$LICHEE/build-env/sdk-link TPU_INC_SHIM=$LICHEE/build-env/tpu-inc-shim \
  cmake --build $LICHEE/build-yolo --target stream_yolo -j$(nproc)

# Deploy
scp build-yolo/bin/stream_yolo root@192.168.1.121:/root/stream_yolo.new
scp sidecar.py index.html hls.min.js jmuxer.min.js root@192.168.1.121:/root/
ssh root@192.168.1.121 'mv /root/stream_yolo.new /root/stream_yolo && reboot'
```

---

## Post-reboot checklist (~60s after reboot)

Run the one-shot verify first, then tick each item.

- [ ] Device responds: `ssh root@192.168.1.121 uptime`
- [ ] Exactly one `stream_yolo` process: `ps | grep stream_yolo`
- [ ] Exactly one `python3 sidecar.py` process: `ps | grep python3`
- [ ] No live-stream `ffmpeg` process required: `ps | grep ffmpeg` should normally be empty outside event remux.
- [ ] `stream_yolo` not stuck: `cat /proc/$(ps | awk '/stream_yolo/{print $1}' | head -1)/wchan` → should NOT be `EnterVcodecLock`
- [ ] Port 7778 listening: `ss -tlnp | grep 7778`
- [ ] **WebSocket live handshake** succeeds on `/ws/live`.
- [ ] **jmuxer.min.js fetchable**: `curl -s -o /dev/null -w "%{http_code}" http://localhost:7778/jmuxer.min.js` → `200`
- [ ] Web UI responds: `curl -s -o /dev/null -w "%{http_code}" http://localhost:7778/` → `200`
- [ ] No traceback in sidecar log: `cat /tmp/ninti_sidecar.log`
- [ ] Timing OK: `dmesg | grep timing | tail -5` → `motion=` should be <20ms
- [ ] Browser: MJPEG stream visible and moving
- [ ] Browser: detection zone overlay present
- [ ] Browser → Live tab: WS stream playing (not frozen); MJPEG fallback still works.

---

## One-shot verify command

```bash
ssh root@192.168.1.121 '
echo "=== processes ===";
ps | grep -E "stream_yolo|python3|ffmpeg" | grep -v grep;
echo "=== venc state ===";
cat /proc/$(ps | awk "/stream_yolo/{print \$1}" | head -1)/wchan 2>/dev/null; echo;
echo "=== port 7778 ===";
ss -tlnp | grep 7778;
echo "=== ws live handshake ===";
python3 - <<'"'"'PY'"'"'
import base64, hashlib, os, socket
key = base64.b64encode(os.urandom(16)).decode()
s = socket.create_connection(("127.0.0.1", 7778), 5)
s.sendall((f"GET /ws/live HTTP/1.1\r\nHost: localhost\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n").encode())
resp = s.recv(4096).decode("latin1", "ignore")
print(resp.split("\r\n")[0])
s.close()
PY
echo "=== jmuxer.min.js ===";
curl -s -o /dev/null -w "HTTP %{http_code}\n" http://localhost:7778/jmuxer.min.js;
echo "=== web ui ===";
curl -s -o /dev/null -w "HTTP %{http_code}\n" http://localhost:7778/;
echo "=== sidecar log ===";
cat /tmp/ninti_sidecar.log | tail -5;
'
```

Expected output:
```
=== processes ===
  NNN root     python3 /root/sidecar.py
  NNN root     /root/stream_yolo ...
=== venc state ===
(anything except EnterVcodecLock)
=== port 7778 ===
LISTEN 0  0  0.0.0.0:7778  ...
=== ws live handshake ===
HTTP/1.1 101 Switching Protocols
=== jmuxer.min.js ===
HTTP 200
=== web ui ===
HTTP 200
=== sidecar log ===
(no traceback)
```

---

## Red flags — reboot required

- More than one `stream_yolo` in `ps` → VPSS conflict
- `EnterVcodecLock` in wchan → VENC deadlock (sidecar was restarted without reboot)
- `CVI_VPSS_CreateGrp failed` in stream_yolo output → VPSS not released
- `/ws/live` does not return `101 Switching Protocols` → check `/tmp/ninti_sidecar.log` and whether port 7778 is serving the new sidecar.
- `Address in use` in sidecar log → `fuser -k 7778/tcp` then restart
