#!/bin/sh
# NintiDetect launcher — YOLO object detector with UDP JSON output + MJPEG preview.
# Requires RTSP to be stopped first (both share VPSS group 0).
# Usage: sh /root/run_ninti_detect.sh [threshold] [udp_port]
set -e
THRESH=${1:-0.50}
PORT=${2:-5005}
MODEL=/root/yolov5_cv181x.cvimodel

export LD_LIBRARY_PATH=/mnt/system/usr/lib:/usr/bin/lib:/root/libs_patch

exec /root/stream_yolo "$MODEL" 80 640 "$THRESH" "$PORT" 640 360
