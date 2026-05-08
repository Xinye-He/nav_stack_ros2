#!/usr/bin/env bash

die() {
    printf '%s\n' "$1"
    exit 1
}

# paths to some project directories
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd -P)"
NETWORKS_DIR="data/networks"
DOCKER_ROOT="/root"     # where the project resides inside docker
STACK_CAN_WS_DIR="${STACK_CAN_WS_DIR:-$REPO_ROOT/stack_can_ws}"

# parse user arguments
USER_COMMAND="$*"
USER_VOLUME=""
DEV_VOLUME=""
ROS_DISTRO="${ROS_DISTRO:-}"
CONTAINER_IMAGE="xinye30/ucar-jetson:nav-stack-ros2"

# check for V4L2 devices
V4L2_DEVICES=""
for i in {0..9}; do
    if [ -a "/dev/video$i" ]; then
        V4L2_DEVICES="$V4L2_DEVICES --device /dev/video$i "
    fi
done

# check for display
DISPLAY_DEVICE=""
if [ -n "$DISPLAY" ]; then
    # 不要用 sudo，避免自启动卡密码
    xhost +si:localuser:root >/dev/null 2>&1 || true
    DISPLAY_DEVICE=" -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix "
fi

# print configuration helpers
print_var() {
    if [ -n "${!1}" ]; then
        local trimmed="$(echo -e "${!1}" | sed -e 's/^[[:space:]]*//')"
        printf '%-17s %s\n' "$1:" "$trimmed"
    fi
}

print_var "CONTAINER_IMAGE"
print_var "ROS_DISTRO"
print_var "DATA_VOLUME"
print_var "DEV_VOLUME"
print_var "USER_VOLUME"
print_var "USER_COMMAND"
print_var "STACK_CAN_WS_DIR"
print_var "V4L2_DEVICES"
print_var "DISPLAY_DEVICE"

cat /proc/device-tree/model > /tmp/nv_jetson_model

# 避免容器名冲突
docker rm -f nav_stack_ros2 >/dev/null 2>&1 || true

docker run --runtime nvidia -it --rm \
    --privileged \
    --name nav_stack_ros2 \
    --network host \
    -e ROS_DISTRO="${ROS_DISTRO}" \
    -e ROS_WS="${ROS_WS:-$DOCKER_ROOT/stack_can_ws}" \
    -e TGT_CMD="${USER_COMMAND:-}" \
    -e START_DOCKER_STARTUP \
    -e RMW_IMPLEMENTATION \
    -e ROS_DOMAIN_ID \
    -v /tmp/argus_socket:/tmp/argus_socket \
    -v /etc/enctune.conf:/etc/enctune.conf \
    -v /etc/nv_tegra_release:/etc/nv_tegra_release \
    -v /tmp/nv_jetson_model:/tmp/nv_jetson_model \
    -v /var/run/dbus:/var/run/dbus \
    -v /var/run/avahi-daemon/socket:/var/run/avahi-daemon/socket \
    -v "$STACK_CAN_WS_DIR:/root/stack_can_ws" \
    --volume /dev:/dev \
    --device /dev \
    --group-add dialout \
    -w "$DOCKER_ROOT" \
    $DISPLAY_DEVICE $V4L2_DEVICES \
    $DATA_VOLUME $USER_VOLUME $DEV_VOLUME \
    "$CONTAINER_IMAGE" \
    bash -lc '
      set +e

      # 自动探测 ROS_DISTRO（未传则从已安装版本中选择）
      if [ -z "$ROS_DISTRO" ]; then
        for d in humble foxy galactic; do
          if [ -f "/opt/ros/$d/setup.bash" ]; then ROS_DISTRO="$d"; break; fi
        done
      fi

      if [ -n "$ROS_DISTRO" ] && [ -f "/opt/ros/$ROS_DISTRO/install/setup.bash" ]; then
        echo "[container] source /opt/ros/$ROS_DISTRO/install/setup.bash"
        source "/opt/ros/$ROS_DISTRO/install/setup.bash"
      else
        echo "[container] WARN: 未找到可用的 ROS 环境（ROS_DISTRO=$ROS_DISTRO）"
      fi

      # 可选：source 工作空间（默认 /root/stack_can_ws，可用 ROS_WS 覆盖）
      if [ -n "$ROS_WS" ] && [ -f "$ROS_WS/install/setup.bash" ]; then
        echo "[container] source $ROS_WS/install/setup.bash"
        source "$ROS_WS/install/setup.bash"
	if [ -f "$ROS_WS/install/stack_msgs/share/stack_msgs/local_setup.bash" ]; then
          source "$ROS_WS/install/stack_msgs/share/stack_msgs/local_setup.bash"
        fi
      fi

      START_DOCKER_STARTUP="${START_DOCKER_STARTUP:-true}"
      if [ "$START_DOCKER_STARTUP" = "true" ]; then
        echo "[container] start docker_startup.launch.py in background ..."
        nohup ros2 launch main docker_startup.launch.py > /tmp/docker_startup.log 2>&1 &
        STARTUP_PID=$!
        echo "[container] docker_startup.launch.py 已启动 (PID: $STARTUP_PID)，日志: /tmp/docker_startup.log"
        sleep 1
      fi

      # 默认流程：健康检查通过后启动 demo.launch.py；失败则保留容器 shell 便于复位后重试。
      if [ -z "$TGT_CMD" ]; then
        TGT_CMD="$ROS_WS/script/start_demo_after_health_check.sh"
      fi

      echo "[container] exec: $TGT_CMD"
      eval "$TGT_CMD"
      CMD_STATUS=$?
      if [ "$CMD_STATUS" -ne 0 ]; then
        echo "[container] command exited with status $CMD_STATUS"
      fi
      # 命令结束后留在交互 shell，便于看日志/排障
      exec bash
    '
