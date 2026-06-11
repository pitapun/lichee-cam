# lichee-cam

NintiDetect on LicheeRV Nano (cv181x / GC4653 sensor).

## Overview

Runs a YOLOv8 object detector on the cv181x NPU, serving:

- MJPEG stream on port 7777
- Web UI + SSE detection events on port 7778
- RTSP stream (via MediaMTX) on port 8554, when in RTSP mode

## Files

| File | Purpose |
|------|---------|
| `sidecar.py` | Main process: manages stream_yolo, motion detection, tracking, web UI |
| `S98ninti_sidecar` | init.d service script (copy to `/etc/init.d/`) |
| `run_ninti_detect.sh` | Manual launcher for stream_yolo standalone |
| `start_gc4653_rtsp.sh` | RTSP mode launcher (MediaMTX + sample_venc) |

## Hardware

- Board: LicheeRV Nano (RISC-V, cv181x SoC)
- Sensor: GC4653 (MIPI CSI, 1440p 30fps)
- Model: `yolov5_cv181x.cvimodel` (YOLOv8, COCO 80 classes, compiled for cv181x NPU)

## Detection Pipeline

```
MOTION DETECT  (frame diff, independent of AI)
      |
AI DETECT      (YOLO inference on cv181x NPU, threshold configurable)
      |
AI TRACKING    (center-distance + IoU tracking, 5s still/lost timeout)
      |
   SAVE        (any tracked object with >= 1 hit is saved to events.jsonl)
```

Key behaviors:

- Motion detect fires on pixel-level frame difference on the full frame at 1/4 scale (no AI needed, fast)
- YOLO inference runs **only on the detection zone crop** (default 640x640), not the full frame — significantly reduces NPU workload and inference latency
- AI detect suppresses motion detect while active
- Tracking allows up to 1s detection gap before losing a track
- Object still for 5s or absent for 5s triggers save
- Any object detected at least once is saved (never discarded if tracked)

### Detection zone

Configured as `detection_zones` in `/root/ninti_config.json` (x, y, size in sensor pixels). The YOLO model receives only the cropped zone resized to `infer_size` (640). Coordinates of detections are scaled back to full-frame space before drawing and UDP broadcast.

HLS live stream is always encoded at 1280x720 regardless of sensor resolution. Sensor can be set higher (e.g. 1920x1080) for higher-quality event recordings while keeping live view at 720p.

## Web UI

Connect browser to `http://<board-ip>:7778`.

- Left sidebar: live pipeline state visualization
- Center: MJPEG stream with zone overlay
- Right sidebar: mode/threshold/zone/class config, detection log, event history

## Configuration

`/root/ninti_config.json` on device:

```json
{
  "mode": "ninti",
  "threshold": 0.60,
  "zones": [],
  "filter_classes": ["person", "vehicle", "animal"]
}
```

## Deployment

Initial install on device:

```sh
scp sidecar.py root@<ip>:/root/sidecar.py
scp S98ninti_sidecar root@<ip>:/etc/init.d/S98ninti_sidecar
ssh root@<ip> "chmod +x /etc/init.d/S98ninti_sidecar"
```

Update sidecar only (graceful, avoids VPSS conflict):

```sh
scp sidecar.py root@<ip>:/root/sidecar.py
ssh root@<ip> "/etc/init.d/S98ninti_sidecar restart"
# init script calls /api/yolo/stop first, then restarts — no manual curl needed
# DO NOT use killall stream_yolo — bypasses VPSS cleanup and corrupts hardware state
```

Update binary + sidecar (requires reboot for clean VPSS state):

```sh
scp build-riscv/bin/stream_yolo root@<ip>:/root/stream_yolo.new
scp sidecar.py root@<ip>:/root/sidecar.py
ssh root@<ip> "mv /root/stream_yolo.new /root/stream_yolo && reboot"
```

## Known Issues

- `CVI_VPSS_CreateGrp failed`: VPSS group 0 not released after forced kill of stream_yolo. Always stop via `/api/yolo/stop` before restarting. If already stuck, reboot device.
- D-state watchdog: sidecar monitors stream_yolo for kernel D-state (60s threshold) and triggers `reboot -f` automatically.
- GC4653 camera: 1440p 30fps, ISP tuning from lxowalle gc4653 30fps profile.
