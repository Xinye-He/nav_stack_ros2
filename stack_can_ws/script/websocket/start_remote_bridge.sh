##!/bin/bash
#
#CONTAINER_NAME="remote_bridge"
#IMAGE="dustynv/ros:humble-desktop-l4t-r35.3.1"
#SCRIPT_PATH_CAN="/home/huang/GPS/websocket_can_bridge.py"
#SCRIPT_PATH_GPS="/home/huang/GPS/server_gps_ws.py"
#
#echo "Stopping old container (if any)..."
#docker stop "$CONTAINER_NAME" 2>/dev/null
#docker rm "$CONTAINER_NAME" 2>/dev/null
#
#echo "Starting container with auto-install and run..."
#
#docker run -itd \
#  --name "$CONTAINER_NAME" \
#  --net=host \
#  --privileged \
#  --env ROS_DOMAIN_ID=0 \
#  --env ROS_DISABLE_SHARED_MEMORY=1 \
#  --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
#  -v "$SCRIPT_PATH_CAN:/root/websocket_can_bridge.py" \
#  -v "$SCRIPT_PATH_GPS:/root/server_gps_ws.py" \
#  "$IMAGE" \
#  bash -c "
#    set -e
#    if ! command -v vim &> /dev/null; then
#      echo 'Installing vim and python3-pip...'
#      apt update && apt install -y vim python3-pip
#    fi
#    if ! python3 -c 'import websockets' 2>/dev/null; then
#      echo 'Installing websockets from Aliyun mirror...'
#      pip3 install websockets -i https://mirrors.aliyun.com/pypi/simple/
#    fi
#    source /opt/ros/humble/install/setup.bash
#    echo 'Starting WebSocket CAN Bridge...'
#    python3 /root/websocket_can_bridge.py &
#    echo 'Starting GPS WebSocket Server...'
#    exec python3 /root/server_gps_ws.py
#  "

#!/bin/bash

CONTAINER_NAME="remote_bridge"
IMAGE="dustynv/ros:humble-desktop-l4t-r35.3.1"
SCRIPT_PATH_CAN="/home/huang/GPS/websocket_can_bridge.py"
SCRIPT_PATH_GPS="/home/huang/GPS/server_gps_ws.py"
SCRIPT_PATH_HEADING="/home/huang/GPS/server_heading_ws.py"

echo "Stopping old container (if any)..."
docker stop "$CONTAINER_NAME" 2>/dev/null
docker rm "$CONTAINER_NAME" 2>/dev/null

echo "Starting container with auto-install and run..."

docker run -itd \
  --name "$CONTAINER_NAME" \
  --net=host \
  --privileged \
  --env ROS_DOMAIN_ID=0 \
  --env ROS_DISABLE_SHARED_MEMORY=1 \
  --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -v "$SCRIPT_PATH_CAN:/root/websocket_can_bridge.py" \
  -v "$SCRIPT_PATH_GPS:/root/server_gps_ws.py" \
  -v "$SCRIPT_PATH_HEADING:/root/server_heading_ws.py" \
  "$IMAGE" \
  bash -c "
    set -e
    if ! command -v vim &> /dev/null; then
      echo 'Installing vim and python3-pip...'
      apt update && apt install -y vim python3-pip
    fi
    if ! python3 -c 'import websockets' 2>/dev/null; then
      echo 'Installing websockets from Aliyun mirror...'
      pip3 install websockets -i https://mirrors.aliyun.com/pypi/simple/
    fi
    source /opt/ros/humble/install/setup.bash

    echo 'Starting WebSocket CAN Bridge...'
    python3 /root/websocket_can_bridge.py &

    echo 'Starting GPS WebSocket Server...'
    python3 /root/server_gps_ws.py &

    echo 'Starting Heading WebSocket Server...'
    python3 /root/server_heading_ws.py &

    wait
  "
