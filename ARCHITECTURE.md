# NintiCam Architecture

This document describes the current LicheeRV Nano camera stack.

## Processes

`sidecar.py` is the supervisor and web/API process.

It:

- launches and restarts `stream_yolo`,
- serves the web UI on port `7778`,
- serves `/ws/live` as the binary H264 live stream,
- receives object detections from `stream_yolo` over UDP port `5005`,
- applies class and filter-zone rules,
- tracks objects and manages event recording,
- serves event metadata and event video files,
- publishes optional MQTT/Home Assistant state.

`stream_yolo` is the RISC-V/CVI runtime process.

It:

- captures the GC4653 sensor through VI/VPSS,
- runs YOLO inference on cropped detection zones,
- sends detection JSON to sidecar UDP `5005`,
- encodes WS live H264,
- encodes HD event H264 while an event is active,
- optionally serves MJPEG fallback on port `7777`.

## Video Channels

The current design uses separate VPSS/VENC paths:

- CHN0 BGR: camera/capture source for YOLO and motion.
- CHN1 NV21: WS live encoder source.
- CHN2 NV21: HD event recorder source.

The sidecar config exposes these as:

- `sensor_width`, `sensor_height`: CHN0 BGR capture size.
- `target_fps`: sensor/capture rate cap. Per-channel fps is clamped to this.
- `ws_width`, `ws_height`, `ws_fps`: CHN1 live stream output.
- `record_width`, `record_height`, `record_fps`: CHN2 event recorder output.
- `rotation_180`: VPSS mirror+flip applied to all channels.

Internally, some C++ names still use `HLS` for historical reasons:

- `YOLO_HLS_W`
- `YOLO_HLS_H`
- `YOLO_HLS_FPS`

These now mean the WS live encoder channel.

## Detection Zones

`detection_zones` are square crops sent to YOLO. Each zone has:

- `x`
- `y`
- `size`

Rules:

- Maximum zones: 4.
- Minimum size: 640.
- A zone may extend beyond the visible frame.
- Empty config is auto-populated with a centered 640 zone.
- `active_detector=true` makes zone 0 follow the latest motion centroid.

## Filter Zones

`zones` are polygon filter zones applied in `sidecar.py` after detection.

Rules:

- If no enabled polygon exists, the whole image is accepted.
- If enabled polygons exist, detections outside all enabled polygons are ignored.
- These zones do not change where YOLO runs. They only filter accepted events.

## Event Lifecycle

Event recording is documented separately in `EVENT_RECORDING.md`.

Short version:

- recording starts after a track is confirmed,
- minimum recording duration is 3 seconds,
- recording stops 1 second after the tracked object disappears,
- recording does not stop just because the object is stationary,
- hard cap is 12 seconds.

## Memory Notes

On 128MB devices, the normal long-running memory users are:

- `sidecar.py`,
- `stream_yolo`,
- kernel slab/cache,
- H264 encoder buffers.

The old idle MJPEG prebuffer path caused steady memory pressure because sidecar
kept a permanent MJPEG connection open. Current behavior keeps idle MJPEG off.

