# lichee-cam / NintiDetect

YOLOv8 object detector on LicheeRV Nano (cv181x SoC, GC4653 sensor).
Serves MJPEG stream, HLS live stream, and a web UI with event recording.

---

## Device file manifest

Everything needed on the device. Deploy all of these on a fresh install.

| Git path | Device path | Notes |
|----------|-------------|-------|
| `sidecar.py` | `/root/sidecar.py` | Main process: manages stream_yolo, tracking, web server |
| `index.html` | `/root/index.html` | Web UI HTML (read from disk — update without restart) |
| `hls.min.js` | `/root/hls.min.js` | HLS.js player for live stream tab |
| `S98ninti_sidecar` | `/etc/init.d/S98ninti_sidecar` | Init script (chmod +x after copy) |
| `bin/stream_yolo` | `/root/stream_yolo` | Compiled RISC-V binary (or build from `src/`) |
| `bin/mediamtx` | `/root/mediamtx` | RTSP relay (optional) |
| `bin/mediamtx.yml` | `/root/mediamtx.yml` | MediaMTX config (optional) |
| `models/yolov8n_coco80.cvimodel` | `/root/yolov8n_coco80.cvimodel` | Default detection model |
| `models/yolov5_cv181x.cvimodel` | `/root/yolov5_cv181x.cvimodel` | Alternative model |
| `models/yolov5su_new.cvimodel` | `/root/yolov5su_new.cvimodel` | Alternative model |
| `models/yolov8n_maixcam_640.cvimodel` | `/root/yolov8n_maixcam_640.cvimodel` | Alternative model |
| `libs/libcvi_tdl.so` | `/root/libs_patch/libcvi_tdl.so` | CVI TDL runtime |
| `libs/libcvi_tdl_app.so` | `/root/libs_patch/libcvi_tdl_app.so` | CVI TDL app runtime |
| `libs/libcvi_ive_tpu.so` | `/root/libs_patch/libcvi_ive_tpu.so` | CVI IVE TPU |
| `libs/libini.so` | `/root/libs_patch/libini.so` | INI parser |

`LD_LIBRARY_PATH` in the init script includes `/root/libs_patch` so these override system libs.

---

## Fresh install

```bash
IP=192.168.1.121   # or ProxyJump: ssh -o ProxyJump=pi5-4g root@192.168.100.195

# Core files
scp sidecar.py index.html hls.min.js jmuxer.min.js root@$IP:/root/

# Init script
scp S98ninti_sidecar root@$IP:/etc/init.d/S98ninti_sidecar
ssh root@$IP 'chmod +x /etc/init.d/S98ninti_sidecar'

# Binary
scp bin/stream_yolo root@$IP:/root/stream_yolo

# Models (large — skip if already present)
scp models/*.cvimodel root@$IP:/root/

# Shared libs
ssh root@$IP 'mkdir -p /root/libs_patch'
scp libs/*.so root@$IP:/root/libs_patch/

# Reboot to start cleanly
ssh root@$IP reboot
```

---

## Update recipes

### UI only — no restart needed

```bash
scp index.html root@$IP:/root/index.html
# Hard-refresh browser — live immediately
```

### Sidecar logic change (sidecar.py)

Requires reboot — sidecar restart without reboot leaves stream_yolo in VENC deadlock.

```bash
scp sidecar.py root@$IP:/root/sidecar.py
ssh root@$IP reboot
```

### Binary change (stream_yolo)

```bash
scp bin/stream_yolo root@$IP:/root/stream_yolo.new
ssh root@$IP 'mv /root/stream_yolo.new /root/stream_yolo && reboot'
```

### Binary + sidecar + UI

```bash
scp bin/stream_yolo root@$IP:/root/stream_yolo.new
scp sidecar.py index.html hls.min.js jmuxer.min.js root@$IP:/root/
ssh root@$IP 'mv /root/stream_yolo.new /root/stream_yolo && reboot'
```

Wait ~50s after reboot, then verify:

```bash
ssh root@$IP 'python3 - <<'"'"'PY'"'"'
import base64, os, socket
key = base64.b64encode(os.urandom(16)).decode()
s = socket.create_connection(("127.0.0.1", 7778), 5)
s.sendall((f"GET /ws/live HTTP/1.1\r\nHost: localhost\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n").encode())
print(s.recv(128).decode("latin1", "ignore").split("\r\n")[0])
s.close()
PY' # expect HTTP/1.1 101 Switching Protocols
curl -s -o /dev/null -w "%{http_code}" http://$IP:7778/ # expect 200
```

---

## Build

See `BUILD.md` for cross-compilation setup.

Quick rebuild after source change:

```bash
cd /home/thylation/Desktop/lichee-cam
cmake --build build-yolo --target stream_yolo -j$(nproc)
# Output: build-yolo/bin/stream_yolo
```

Copy the built binary to `bin/stream_yolo` before committing:

```bash
cp build-yolo/bin/stream_yolo bin/stream_yolo
git add bin/stream_yolo && git commit -m "..."
```

---

## Architecture

```
sidecar.py
  ├── launches stream_yolo as subprocess
  ├── reads MJPEG frames from :7777 → pre-buffer + event JPEG frames
  ├── receives detection UDP from stream_yolo (:5005)
  ├── tracks objects, manages event lifecycle
  ├── serves web UI on :7778 (index.html read from disk)
  └── serves HLS via ffmpeg → /tmp/hls/

stream_yolo (C++)
  ├── VPSS group 0: captures from GC4653 (1280x720 disp + 2560x1440 raw)
  ├── CHN0 VENC: HLS encoder (1280x720 H264 → /tmp/hls_feed.h264 FIFO)
  ├── CHN1 VENC: event recorder (2560x1440 H264, active during events only)
  ├── YOLO inference on cv181x NPU (640x640 zone crop)
  └── MJPEG output on :7777
```

Key constraints:
- CHN0 and CHN1 cannot run simultaneously — CHN0 paused during events, resumed after
- Single-core C906 RISC-V: `cv::setNumThreads(1)` prevents OpenMP overhead
- ION memory (VENC input): written via manual BGR→NV21 loop, not cvtColor

---

## Configuration

`/root/ninti_config.json` on device:

```json
{
  "threshold": 0.55,
  "zones": [{"x": 573, "y": 169, "size": 640}],
  "filter_classes": ["person", "cat", "dog"],
  "motion_enabled": true,
  "motion_sensitivity": 10,
  "fps_cap": 5
}
```

---

## Known issues

- Sidecar restart without reboot leaves stream_yolo in VENC `EnterVcodecLock` D-state. Always reboot after sidecar.py changes.
- `CVI_VPSS_CreateGrp failed`: VPSS not released after forced kill. Reboot to recover.
- WS live takes ~45s after reboot before first decodable H264 frames appear (camera init + VENC warm-up).
