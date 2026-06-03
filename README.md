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

- Motion detect fires on pixel-level frame difference (no AI needed)
- AI detect suppresses motion detect while active
- Tracking allows up to 1s detection gap before losing a track
- Object still for 5s or absent for 5s triggers save
- Any object detected at least once is saved (never discarded if tracked)

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

Update sidecar (graceful, avoids VPSS conflict):

```sh
# 1. Stop stream_yolo cleanly via API
curl -X POST http://<ip>:7778/api/yolo/stop

# 2. Copy new sidecar
scp sidecar.py root@<ip>:/root/sidecar.py

# 3. Restart service
ssh root@<ip> "/etc/init.d/S98ninti_sidecar restart"
```

## Known Issues

- `CVI_VPSS_CreateGrp failed`: VPSS group 0 not released after forced kill of stream_yolo. Always stop via `/api/yolo/stop` before restarting. If already stuck, reboot device.
- D-state watchdog: sidecar monitors stream_yolo for kernel D-state (60s threshold) and triggers `reboot -f` automatically.
- GC4653 camera: 1440p 30fps, ISP tuning from lxowalle gc4653 30fps profile.
