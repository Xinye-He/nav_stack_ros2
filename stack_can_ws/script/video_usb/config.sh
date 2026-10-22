#!/usr/bin/env bash
RTSP_SERVER="127.0.0.1"
RTSP_PORT=8554
# cam0 is the detected IMX219 CSI camera; cam1 is the USB/UVC camera.
CAM_NAMES=("cam0" "cam1")
CAM_TYPES=("csi" "usb")
CSI_SENSOR_ID=0
USB_CAMERA_DEVICE="${USB_CAMERA_DEVICE:-}"
if [ -z "$USB_CAMERA_DEVICE" ]; then
  for candidate in /dev/v4l/by-id/*-video-index0; do
    if [ -e "$candidate" ]; then USB_CAMERA_DEVICE="$candidate"; break; fi
  done
fi
if [ -z "$USB_CAMERA_DEVICE" ]; then
  for sysdev in /sys/class/video4linux/video*; do
    [ -e "$sysdev/device/driver" ] || continue
    if [ "$(basename "$(readlink -f "$sysdev/device/driver")")" = "uvcvideo" ]; then
      USB_CAMERA_DEVICE="/dev/$(basename "$sysdev")"
      break
    fi
  done
fi
CAM_DEVS=("/dev/video0" "${USB_CAMERA_DEVICE:-/dev/video-usb-not-connected}")
CAM_WIDTH=1920
CAM_HEIGHT=1080
CAM_FPS=30
CAM_BITRATE_KBPS=2000
