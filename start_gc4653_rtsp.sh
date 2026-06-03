#!/bin/sh
set -eu
cd /root

# GC4653 -> two H264 FIFOs -> MediaMTX RTSP.
# Main URL: rtsp://<board-ip>:8554/live
# Sub URL:  rtsp://<board-ip>:8554/sub

MAIN_WIDTH=1920
MAIN_HEIGHT=1080
MAIN_QP=22
SUB_WIDTH=1280
SUB_HEIGHT=720
SUB_QP=26

if ! pidof mediamtx >/dev/null 2>&1; then
  /root/mediamtx /root/mediamtx.yml >/tmp/mediamtx.log 2>&1 &
  echo $! >/tmp/mediamtx.pid
  sleep 2
fi

if pidof sample_venc >/dev/null 2>&1; then
  ipaddr=$(ip -4 addr show wlan0 | awk '/inet /{print $2}' | cut -d/ -f1)
  echo "already running:"
  echo "  rtsp://${ipaddr}:8554/live"
  echo "  rtsp://${ipaddr}:8554/sub"
  exit 0
fi

killall ffmpeg tail 2>/dev/null || true
rm -f /root/test-0.h264 /root/test-1.h264 \
  /tmp/live_main_publish.log /tmp/live_sub_publish.log \
  /tmp/live_venc.log /tmp/live_venc.rc
mkfifo /root/test-0.h264 /root/test-1.h264

ffmpeg -hide_banner -loglevel info -fflags nobuffer -f h264 -i /root/test-0.h264 \
  -c:v copy -rtsp_transport tcp -f rtsp rtsp://127.0.0.1:8554/live \
  >/tmp/live_main_publish.log 2>&1 &
echo $! >/tmp/live_main_publish.pid

ffmpeg -hide_banner -loglevel info -fflags nobuffer -f h264 -i /root/test-1.h264 \
  -c:v copy -rtsp_transport tcp -f rtsp rtsp://127.0.0.1:8554/sub \
  >/tmp/live_sub_publish.log 2>&1 &
echo $! >/tmp/live_sub_publish.pid

/mnt/system/usr/bin/sample_venc \
  --numChn=2 \
  --frame_num=999999 \
  --testMode=2 --sensorEn=1 --bindmode=2 \
  --viWidth=2560 --viHeight=1440 \
  --chn=0 -c 264 -w "$MAIN_WIDTH" -h "$MAIN_HEIGHT" --gop=30 --frameQp="$MAIN_QP" \
  --chn=1 -c 264 -w "$SUB_WIDTH" -h "$SUB_HEIGHT" --gop=30 --frameQp="$SUB_QP" \
  >/tmp/live_venc.log 2>&1 &
echo $! >/tmp/live_venc.pid

ipaddr=$(ip -4 addr show wlan0 | awk '/inet /{print $2}' | cut -d/ -f1)
echo "rtsp://${ipaddr}:8554/live"
echo "rtsp://${ipaddr}:8554/sub"
