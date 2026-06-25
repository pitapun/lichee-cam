# Operations

This document captures operational notes that are not part of core code design.

## Devices

Common targets:

- `192.168.1.121`: local LicheeRV Nano camera.
- `192.168.100.195`: remote LicheeRV Nano camera, reached via `pi5-4g`.
- `192.168.100.10`: Pi/Home Assistant/Frigate host, reached via `pi5-4g` as `pitapun`.

Examples:

```bash
ssh root@192.168.1.121
ssh -J pi5-4g root@192.168.100.195
ssh -J pi5-4g pitapun@192.168.100.10
```

## Deploy Web/UI/Sidecar Files

Use:

```bash
scripts/deploy-web.sh root@192.168.1.121
scripts/deploy-web.sh --jump pi5-4g root@192.168.100.195
```

The script deploys:

- `sidecar.py`
- `index.html`
- `ha-stream.html`
- `jmuxer.min.js`

It also migrates legacy `hls_*` config keys to `ws_*` keys and verifies:

- `/api/config`,
- `/jmuxer.min.js`,
- `/ws/live`.

Use `--reboot` when hardware state needs a full reset:

```bash
scripts/deploy-web.sh --jump pi5-4g --reboot root@192.168.100.195
```

## Sidecar Restart vs Reboot

Historically, sidecar restart could leave `stream_yolo` in a VENC deadlock.

For UI-only changes:

- copy `index.html`,
- no reboot required.

For sidecar-only logic changes:

- restart may be enough for simple tracking/UI server behavior,
- reboot is safer if VENC/VPSS ownership changes or if `stream_yolo` is stuck.

If unsure, reboot.

Red flags:

```bash
cat /proc/$(pidof stream_yolo)/wchan
```

If it shows `EnterVcodecLock`, reboot.

## Health Checks

On camera:

```bash
ps w | grep -E "sidecar|stream_yolo" | grep -v grep
curl -s -o /dev/null -w "%{http_code}\n" http://127.0.0.1:7778/
curl -s -o /dev/null -w "%{http_code}\n" http://127.0.0.1:7778/jmuxer.min.js
tail -80 /tmp/ninti_sidecar.log
tail -80 /tmp/stream_yolo.log
```

WS handshake:

```bash
python3 - <<'PY'
import base64, os, socket
key = base64.b64encode(os.urandom(16)).decode()
s = socket.create_connection(("127.0.0.1", 7778), timeout=5)
s.sendall((f"GET /ws/live HTTP/1.1\r\nHost: localhost\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n").encode())
print(s.recv(128).decode("latin1", "ignore").split("\r\n")[0])
s.close()
PY
```

Expected:

```text
HTTP/1.1 101 Switching Protocols
```

## Memory Checks

On Lichee camera:

```bash
free -m
grep -E "MemAvailable|Slab|SUnreclaim|Cached|Buffers|Swap" /proc/meminfo
for p in $(pidof python3 stream_yolo ffmpeg 2>/dev/null); do
  echo PID=$p CMD=$(tr "\0" " " </proc/$p/cmdline)
  grep -E "VmPeak|VmSize|VmRSS|VmData|Threads" /proc/$p/status
done
```

Known normal order of magnitude on `192.168.100.195`:

- `sidecar.py`: around 23-28MB RSS,
- `stream_yolo`: around 20-31MB RSS,
- some kernel slab/cache is normal.

## 192.168.100.10 Frigate Note

`192.168.100.10` is not the Lichee camera. It is the Pi/Home Assistant/Frigate
host.

Recent RAM finding:

- Frigate was the largest RAM user.
- `MaixCam` was configured as `2560x1440@5` detect/record.
- `MaixCam` RTSP `192.168.100.221:8554/live` was unreachable.
- Frigate watchdog kept restarting that ffmpeg path.
- Disabling `MaixCam` reduced memory pressure substantially.

Current mitigation:

```yaml
cameras:
  MaixCam:
    enabled: false
```

Backup created on that host:

```text
/home/pitapun/Frigate/config/config.yaml.bak-disable-maixcam-20260625205032
```

