# Performance and Tuning

This device is a small cv181x system. Performance depends on how much work is
assigned to each VPSS/VENC channel and the NPU detector.

## Current Useful Defaults

Stable production-style settings on `192.168.100.195`:

```json
{
  "sensor_width": 1920,
  "sensor_height": 1080,
  "target_fps": 10,
  "ws_width": 854,
  "ws_height": 480,
  "ws_fps": 10,
  "record_width": 1920,
  "record_height": 1080,
  "record_fps": 10,
  "rotation_180": true,
  "yolo_motion_enabled": true,
  "yolo_motion_source": "bgr"
}
```

## Resolution Roles

Do not treat every resolution setting as the same thing.

`sensor_width` / `sensor_height`:

- CHN0 BGR capture size,
- YOLO crop source,
- motion source when `yolo_motion_source=bgr`.

`ws_width` / `ws_height`:

- CHN1 VPSS NV21 output,
- encoded for WS live,
- should be tuned for live smoothness.

`record_width` / `record_height`:

- CHN2 VPSS NV21 output,
- encoded only while event recording is active,
- should be tuned for saved clip quality.

## FPS Roles

`target_fps`:

- global capture/sensor cap,
- per-channel fps is clamped to it.

`ws_fps`:

- live WS H264 target,
- lower values save VENC/bandwidth.

`record_fps`:

- event recorder target fps,
- also used when remux fallback cannot infer raw H264 frame count quickly.

## Important Findings

Software resize was expensive on this build.

Old path:

- BGR resize for HLS/live used `cv::INTER_LINEAR`,
- `prepare_hls` could take around 700-800ms,
- loop fps could collapse to around 1fps.

Current path:

- VPSS provides the live NV21 channel directly,
- live no longer depends on software BGR resize,
- async detector avoids blocking the main capture loop on NPU inference.

## Async Detector

`detector.detect()` runs in a worker thread.

Benefits:

- main loop keeps feeding live/event encoders,
- event fps is not capped by one slow inference call,
- boxes may be 1-2 frames stale, which is acceptable for surveillance.

## Motion

Motion can run before YOLO to decide where the active detection zone should
follow.

Config:

```json
{
  "yolo_motion_enabled": true,
  "yolo_motion_source": "bgr"
}
```

Known sources:

- `bgr`: CHN0 BGR frame, highest fidelity but uses CPU memory bandwidth.
- `hls_y`: historical/experimental lower-cost path name; only use after testing.

## What Not To Do

Avoid continuous HD pre-roll encoding on this SoC.

Observed result:

- loop fps dropped from around 8fps to 3-5fps,
- memory rose into the 33-36MB range,
- device became unstable under short tests.

If pre-roll is required later, prefer:

- lower-resolution pre-roll,
- sidecar-side low-cost ring,
- or explicit SPS/PPS handling with limited encoder duty cycle.

## Practical Tuning Order

When live is too slow:

1. Lower `ws_width` / `ws_height`.
2. Lower `ws_fps`.
3. Keep `record_width` high if saved clips matter more than live preview.
4. Lower `target_fps` only when the whole capture pipeline is unstable.
5. Disable motion briefly to isolate motion cost.
6. Avoid raising `target_fps` above what the VI/VPSS/VENC stack can sustain.

