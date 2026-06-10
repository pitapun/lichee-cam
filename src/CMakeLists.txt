cmake_minimum_required(VERSION 3.15)

project(stream_yolo)
set(CMAKE_CXX_STANDARD 11)

SET(CMAKE_C_COMPILER "$ENV{COMPILER}/riscv64-unknown-linux-musl-gcc")
SET(CMAKE_CXX_COMPILER "$ENV{COMPILER}/riscv64-unknown-linux-musl-g++")
SET(CMAKE_C_LINK_EXECUTABLE "$ENV{COMPILER}/riscv64-unknown-linux-musl-ld")
set(CMAKE_CXX_FLAGS "-march=rv64imafd -O3 -DSENSOR_GCORE_GC4653 -D_MIDDLEWARE_V2_ -DC906 -DUSE_TPU_IVE -fsigned-char -Wno-format-truncation -fdiagnostics-color=always -s")
#-DNDEBUG

include_directories(
    $ENV{SDK_PATH}/cvitek_tdl_sdk/include
    $ENV{SDK_PATH}/cvitek_tdl_sdk/include/cvi_tdl
    $ENV{SDK_PATH}/sample/3rd/middleware/v2/include
)
# tpu/include must come AFTER OpenCV (sdk ships its own opencv2/ headers in
# that dir which would otherwise shadow our patched build); we expose just
# the few tpu headers we actually need via the target line below.

if(NOT DEFINED OpenCV_DIR)
    set(OpenCV_DIR "$ENV{OPENCV_INSTALL}/lib/cmake/opencv4")
endif()

# Manually declare OpenMP::OpenMP_CXX because cmake's auto-detect doesn't
# probe the riscv cross-compiler properly; opencv-mobile was built with
# -fopenmp so downstream code needs to link with it too.
add_library(OpenMP::OpenMP_CXX INTERFACE IMPORTED)
set_property(TARGET OpenMP::OpenMP_CXX PROPERTY INTERFACE_COMPILE_OPTIONS "-fopenmp")
set_property(TARGET OpenMP::OpenMP_CXX PROPERTY INTERFACE_LINK_OPTIONS "-fopenmp")
add_library(OpenMP::OpenMP_C INTERFACE IMPORTED)
set_property(TARGET OpenMP::OpenMP_C PROPERTY INTERFACE_COMPILE_OPTIONS "-fopenmp")
set_property(TARGET OpenMP::OpenMP_C PROPERTY INTERFACE_LINK_OPTIONS "-fopenmp")

find_package(OpenCV REQUIRED)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
file(MAKE_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})

add_executable(stream_yolo main.cpp MJPEGWriter.cpp yolo.cpp)

target_include_directories(stream_yolo PRIVATE
    $ENV{TPU_INC_SHIM}
)

target_link_libraries(stream_yolo
    -mcpu=c906fdv
    -L$ENV{SDK_PATH}/sample/3rd/middleware/v2/lib
    -lsns_full -lisp -lvdec -lvenc -lawb -lae -laf -lcvi_bin -lcvi_bin_isp -lmisc -lisp_algo -lsys -lvpu
    -L$ENV{SDK_PATH}/sample/3rd/middleware/v2/lib/3rd
    -lini
    -L$ENV{SDK_PATH}/sample/3rd/tpu/lib
    -lcnpy -lcvikernel -lcvimath -lcviruntime -lz
    -L${CMAKE_CURRENT_SOURCE_DIR}/../libs
    -lcvi_ive_tpu
    -L$ENV{SDK_PATH}/cvitek_tdl_sdk/lib
    -lcvi_tdl
    -lpthread -latomic -lm
    ${OpenCV_LIBS}
)
