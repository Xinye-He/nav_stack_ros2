#!/usr/bin/env bash
set -euo pipefail
MONITOR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$MONITOR_DIR/config.sh"
command -v gst-launch-1.0 >/dev/null
available=0
for dev in "${CAM_DEVS[@]}"; do
  if [ -e "$dev" ]; then available=$((available + 1)); fi
done
if [ "$available" -eq 0 ]; then
  echo "没有可用相机，未启动推流" >&2
  exit 2
fi
LOG_DIR="$MONITOR_DIR/logs"
mkdir -p "$LOG_DIR"
# Clean up only this monitor's tracked pipelines before restarting.
bash "$MONITOR_DIR/stop_monitor.sh"
docker run -d --name mediamtx --network host \
  -v "$MONITOR_DIR/mediamtx.yml:/mediamtx.yml:ro" bluenviron/mediamtx:latest
sleep 2
for idx in "${!CAM_NAMES[@]}"; do
  name="${CAM_NAMES[$idx]}"
  dev="${CAM_DEVS[$idx]}"
  if [ ! -e "$dev" ]; then
    echo "跳过未识别相机：$name ($dev)"
    continue
  fi
  kind="${CAM_TYPES[$idx]:-usb}"
  case "$kind" in
    csi)
      source_args=(nvarguscamerasrc "sensor-id=$CSI_SENSOR_ID" !
        "video/x-raw(memory:NVMM),width=$CAM_WIDTH,height=$CAM_HEIGHT,format=NV12,framerate=$CAM_FPS/1" ! nvvidconv)
      ;;
    usb)
      source_args=(v4l2src "device=$dev" io-mode=2 do-timestamp=true !
        "image/jpeg,width=$CAM_WIDTH,height=$CAM_HEIGHT,framerate=$CAM_FPS/1" !
        queue max-size-buffers=2 leaky=downstream ! jpegparse ! nvv4l2decoder mjpeg=1 ! nvvidconv)
      ;;
    *) echo "不支持的相机类型：$kind" >&2; exit 2 ;;
  esac
  nohup gst-launch-1.0 -e "${source_args[@]}" ! \
    "video/x-raw(memory:NVMM),format=NV12,width=$CAM_WIDTH,height=$CAM_HEIGHT,framerate=$CAM_FPS/1" ! \
    nvv4l2h264enc bitrate=$((CAM_BITRATE_KBPS * 1000)) iframeinterval=15 insert-sps-pps=1 \
      maxperf-enable=1 preset-level=1 control-rate=1 ! \
    h264parse config-interval=1 disable-passthrough=true ! \
    rtspclientsink "location=rtsp://$RTSP_SERVER:$RTSP_PORT/$name" protocols=tcp sync=false async=false \
    > "$LOG_DIR/$name.log" 2>&1 < /dev/null &
  echo $! > "$MONITOR_DIR/$name.pid"
  echo "$name ($kind)：rtsp://$RTSP_SERVER:$RTSP_PORT/$name"
  sleep 1
done
