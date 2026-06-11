# Building stream_yolo

The binary `bin/stream_yolo` is built from `src/main.cpp` against a
patched `opencv-mobile-4.10.0`, cross-compiled for cv181x (riscv64
musl). The full build environment is NOT checked into this repo (~7 GB
of toolchain + opencv source).

## Inputs

- `src/main.cpp` — the YoloCamera entry-point. Owns CLI parsing
  (`--zones`, `--fps`), capture setup, motion gating, NPU inference,
  MJPEG/UDP output.
- `src/MJPEGWriter.{cpp,h}`, `src/yolo.{cpp,hpp}` — helpers (unchanged).
- `src/CMakeLists.txt.ref` — reference CMakeLists from the lichee build.
- `patches/capture_cvi.cpp.patch` — diff vs upstream opencv-mobile-4.10.0
  highgui. Contains the runtime sensor-fps override that lets us drop
  GC4653 from 30fps to e.g. 5fps (SoC cools from ~95°C to ~70°C).
- `patches/capture_cvi.cpp.orig` — captured upstream baseline.
- `patches/capture_cvi.cpp.patched` — full patched file ready to drop in.
- `sensor-drivers/` — reference source for the GC4653 driver shipped as
  precompiled `/mnt/system/usr/lib/libsns_gc4653.so` on device. Tells
  you which register addresses + math to use when extending.

## Recreating the build environment

Validated end-to-end 2026-06-09 — these exact commands rebuild the
patched libopencv_highgui.a containing the VTS override:

1. Get the sophgo riscv toolchain (~841 MB tarball, ~3.6 GB extracted):
   ```
   curl -LO https://sophon-file.sophon.cn/sophon-prod-s3/drive/23/03/07/16/host-tools.tar.gz
   tar -xzf host-tools.tar.gz
   export RISCV_ROOT_PATH=$(pwd)/host-tools/gcc/riscv64-linux-musl-x86_64
   ```
2. Get nihui's opencv-mobile build harness (~360 KB; it's a wrapper,
   not a full source tree):
   ```
   curl -L -o opencv-mobile.zip https://github.com/nihui/opencv-mobile/archive/refs/tags/v30.zip
   unzip -q opencv-mobile.zip   # produces opencv-mobile-30/
   ```
3. Get opencv-4.10.0 upstream source (~96 MB):
   ```
   curl -L -o opencv-4.10.0.zip https://github.com/opencv/opencv/archive/refs/tags/4.10.0.zip
   unzip -q opencv-4.10.0.zip
   ```
4. Apply nihui's strip + patches following the workflow steps from
   `opencv-mobile-30/.github/workflows/release.yml` (jobs `opencv4-source`
   and `licheerv-nano`). Key steps:
   - `truncate -s 0 cmake/OpenCVFindLibsGrfmt.cmake`
   - `rm -rf modules/gapi`
   - Bulk remove: `modules/core/src/{cuda_*,direct*,gl_*,intel_gpu_*,ocl*,opengl.cpp,ovx.cpp,umatrix.hpp,va_intel.cpp,va_wrapper.impl.hpp}`,
     headers `modules/core/include/opencv2/core/{cuda*.hpp,directx.hpp,ocl*.hpp,opengl.hpp,ovx.hpp,private.cuda.hpp,va_*.hpp}`,
     dirs `modules/core/include/opencv2/core/{cuda,opencl,openvx}`,
     `modules/photo/src/denoising.cuda.cpp`, `modules/photo/include/opencv2/photo/cuda.hpp`,
     all `*/src/{cuda,opencl}` and `*/perf/{cuda,opencl}` subdirs.
   - sed-strip include lines:
     `find modules -type f -exec sed -i -e '/opencl_kernels/d' -e '/cuda.hpp/d' -e '/opengl.hpp/d' -e '/ocl_defs.hpp/d' -e '/ocl.hpp/d' -e '/ovx_defs.hpp/d' -e '/ovx.hpp/d' -e '/va_intel.hpp/d' {} \;`
   - Apply patches in order: no-gpu, no-rtti, no-zlib, link-openmp,
     fix-windows-arm-arch, minimal-install (all `-p1`).
   - `cp opencv-mobile-30/patches/{draw_text.h,mono_font_data.h} modules/imgproc/src/`
   - `cp opencv-mobile-30/patches/fontface.html ./`
   - `patch -p1 -i opencv-mobile-30/patches/opencv-4.10.0-drawing-mono-font.patch`
   - `rm -rf modules/highgui && cp -r opencv-mobile-30/highgui modules/`
   - `rm -rf 3rdparty apps data doc samples platforms modules/{java,js,python,ts,dnn}`
   - `cp opencv-mobile-30/opencv4_cmake_options.txt ./options.txt`
5. Apply OUR cvi patch (the sensor fps override):
   ```
   patch -p0 modules/highgui/src/capture_cvi.cpp < /path/to/lichee-cam/patches/capture_cvi.cpp.patch
   ```
   AND extend `modules/highgui/src/capture_cvi.h` to declare the lichee
   extensions — add inside the `capture_cvi` class:
   ```
   int read_frame(unsigned char* bgrdata, bool retain_image_ptr = false);
   void* getImagePtr();
   void* getImagePtr2();
   void* getOriginalImagePtr();
   void releaseImagePtr();
   ```
   (The 1-arg `read_frame` in upstream becomes the new 2-arg signature.)
6. Configure + build opencv-mobile:
   ```
   cp opencv-mobile-30/toolchains/riscv64-unknown-linux-musl.toolchain.cmake .
   mkdir build && cd build
   cmake -DCMAKE_TOOLCHAIN_FILE=../riscv64-unknown-linux-musl.toolchain.cmake \
         -DCMAKE_INSTALL_PREFIX=install -DCMAKE_BUILD_TYPE=Release \
         -DWITH_CVI=ON $(cat ../options.txt) ..
   cmake --build . -j$(nproc)
   cmake --build . --target install
   ```
   Result: `build/install/lib/libopencv_highgui.a` etc. with
   our patch baked in (verify with `strings | grep YOLO_CAP_FPS`).
7. Build stream_yolo against the install dir. The lichee
   `src/CMakeLists.txt.ref` references the cvitek SDK V1 layout
   (`$SDK_PATH/cvitek_tdl_sdk/include`, `$SDK_PATH/sample/3rd/middleware/v2/include`
   etc.) and links a long list of cvitek shared libs (`libsns_full`,
   `libisp`, `libvpu`, `libcvi_bin_isp`, ...). These come from the
   Sipeed LicheeRV Nano BSP SDK (the legacy V1 / cv181x branch, not
   sophgo's newer top-of-tree tdl_sdk). Easiest sources:
   - <https://github.com/sipeed/LicheeRV-Nano-Build> (BSP + samples)
   - <https://github.com/sophgo/cvi_mpi> + <https://github.com/sophgo/middleware>
   With those checked out and `SDK_PATH` + `COMPILER` env set, build:
   ```
   export COMPILER=$RISCV_ROOT_PATH/bin
   export SDK_PATH=/path/to/sdk
   mkdir build-yolo && cd build-yolo
   cmake -DOpenCV_DIR=$(pwd)/../build-env/opencv-4.10.0/build/install/lib/cmake/opencv4 \
         /path/to/lichee-cam/src
   cmake --build . -j$(nproc)
   ```
8. `scp build-yolo/bin/stream_yolo root@<device>:/root/stream_yolo`.

## Runtime sensor fps override (in the patch)

The patch hooks `start_streaming()` and spawns a background thread that
3s after start writes the GC4653 VTS register (0x0340/0x0341) directly
via `pstSnsObj->pfnWriteReg`. Triggered by env var `YOLO_CAP_FPS` set
by `main.cpp` from the `--fps N` arg.

```
VMAX = 1500 * 30 / target_fps   # 1500 = mode VTS default, 30 = MaxFps
0x0340 = (VMAX >> 8) & 0xFF
0x0341 = VMAX & 0xFF
```

Validated 2026-06-09 on .121 with target_fps=5:
chip ID readback 0x4653, VTS register confirmed 0x2328, soc_temp
94-97°C → 70.7°C sustained.

Why direct register write works (and the SDK's
`pfn_cmos_fps_set` does not): the cv181x ISP commits sensor I2C only
on explicit register-update flags. `cmos_fps_set` only stages values
in an I2C cache that the AE thread never flushes for VTS. Direct
`pfnWriteReg` bypasses the cache and hits the sensor I2C bus
immediately. See `sensor-drivers/README.md` for the full call-chain
reasoning.
