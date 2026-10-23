# Sensor Drivers

Reference source for the CMOS sensor drivers shipped in
`/mnt/system/usr/lib/libsns_*.so` on the LicheeRV Nano image.

Origin: <https://github.com/sophgo/SensorSupportList> (cloned 2026-06-09).

## Why this is here

`libsns_gc4653.so` etc. are precompiled binary blobs on the device.
The source in this directory is the authoritative reference for:

- Register addresses (e.g. GC4653 VTS = 0x0340 / 0x0341)
- The mode tables (e.g. `gc4653_cmos_param.h` — VtsDef=1500, MaxFps=30)
- The `pfn_cmos_fps_set` callback wired through `pstSnsObj->pfnExpAeCb`
- The init register sequence in `*_sensor_ctl.c`

## Layout

- `SensorSupportList/sensor/cv181x/` — drivers for the cv181x SoC family
  (LicheeRV Nano uses this; GC4653 sits under `gcore_gc4653/`).
- `SensorSupportList/sensor/cv182x/` — sibling family.

Each sensor directory contains:

- `<sensor>_cmos.c` — AE/exposure callbacks (`cmos_fps_set` lives here).
- `<sensor>_cmos_param.h` — mode tables (FPS range, VTS/HTS defaults).
- `<sensor>_sensor_ctl.c` — power-on register write sequence.
- `<sensor>_cmos_ex.h` — register addresses + mode enum.

## Runtime sensor fps override (used by YoloCamera)

The custom `opencv-mobile-4.10.0/modules/highgui/src/capture_cvi.cpp`
in this tree invokes `pstSnsObj->pfnWriteReg` directly on the VTS
registers from a background thread spawned in `start_streaming()`.
Triggered by `YOLO_CAP_FPS` env var (set by YoloCamera's `--fps N`).

Computation (GC4653 1440P mode):

```
vmax = 1500 * 30 / target_fps   # 1500 = mode VtsDef, 30 = mode MaxFps
VTS_H (0x0340) = (vmax >> 8) & 0xFF
VTS_L (0x0341) = vmax & 0xFF
```

Validated 2026-06-09 on .121:
target_fps=5 -> sensor truly runs 5fps, SoC temp 94-97°C -> 70.7°C.
