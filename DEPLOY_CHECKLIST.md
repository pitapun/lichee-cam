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
scp sidecar.py root@192.168.1.121:/root/sidecar.py
ssh root@192.168.1.121 'mv /root/stream_yolo.new /root/stream_yolo && reboot'
```

---

## Post-reboot checklist (~60s after reboot)

Run the one-shot verify first, then tick each item.

- [ ] Device responds: `ssh root@192.168.1.121 uptime`
- [ ] Exactly one `stream_yolo` process: `ps | grep stream_yolo`
- [ ] Exactly one `python3 sidecar.py` process: `ps | grep python3`
- [ ] Exactly one `ffmpeg` process: `ps | grep ffmpeg`
- [ ] `stream_yolo` not stuck: `cat /proc/$(ps | awk '/stream_yolo/{print $1}' | head -1)/wchan` → should NOT be `EnterVcodecLock`
- [ ] Port 7778 listening: `ss -tlnp | grep 7778`
- [ ] HLS segments exist: `ls /tmp/hls/*.ts | wc -l` → ≥ 3
- [ ] **live.m3u8 fetchable**: `curl -sv http://localhost:7778/hls/live.m3u8 2>&1 | grep -E "< HTTP|#EXTM3U"` → `HTTP/1.0 200` + `#EXTM3U`
- [ ] **hls.min.js fetchable**: `curl -s -o /dev/null -w "%{http_code}" http://localhost:7778/hls.min.js` → `200`
- [ ] Web UI responds: `curl -s -o /dev/null -w "%{http_code}" http://localhost:7778/` → `200`
- [ ] No traceback in sidecar log: `cat /tmp/ninti_sidecar.log`
- [ ] Timing OK: `dmesg | grep timing | tail -5` → `motion=` should be <20ms
- [ ] Browser: MJPEG stream visible and moving
- [ ] Browser: detection zone overlay present
- [ ] Browser → Live tab: HLS playing (not 404, not frozen)

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
echo "=== hls segments ===";
ls /tmp/hls/*.ts 2>/dev/null | wc -l;
echo "=== live.m3u8 ===";
curl -sv http://localhost:7778/hls/live.m3u8 2>&1 | grep -E "< HTTP|#EXTM3U|404";
echo "=== hls.min.js ===";
curl -s -o /dev/null -w "HTTP %{http_code}\n" http://localhost:7778/hls.min.js;
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
  NNN root     ffmpeg ...
=== venc state ===
(anything except EnterVcodecLock)
=== port 7778 ===
LISTEN 0  0  0.0.0.0:7778  ...
=== hls segments ===
5
=== live.m3u8 ===
< HTTP/1.0 200 OK
#EXTM3U
=== hls.min.js ===
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
- HLS segment count = 0 after 45s → check ffmpeg (`cat /proc/<ffmpeg_pid>/wchan` → `pipe_read` means waiting for data from stream_yolo)
- 404 on `/hls/live.m3u8` within first 20s after boot → normal, segments not yet generated; wait and retry
- `Address in use` in sidecar log → `fuser -k 7778/tcp` then restart
