#!/usr/bin/env bash

die() {
    printf '%s\n' "$1"
    exit 1
}

# paths to some project directories
NETWORKS_DIR="data/networks"
DOCKER_ROOT="/root"     # where the project resides inside docker

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
    -e RMW_IMPLEMENTATION \
    -e ROS_DOMAIN_ID \
    -v /tmp/argus_socket:/tmp/argus_socket \
    -v /etc/enctune.conf:/etc/enctune.conf \
    -v /etc/nv_tegra_release:/etc/nv_tegra_release \
    -v /tmp/nv_jetson_model:/tmp/nv_jetson_model \
    -v /var/run/dbus:/var/run/dbus \
    -v /var/run/avahi-daemon/socket:/var/run/avahi-daemon/socket \
    -v /home/xinye30/lidar_rosbag:/root/lidar_rosbag \
    -v /home/xinye30/nav_stack_ros2/stack_can_ws:/root/stack_can_ws \
    --volume /dev:/dev \
    --device /dev \
    --group-add dialout \
    -w "$DOCKER_ROOT" \
    $DISPLAY_DEVICE $V4L2_DEVICES \
    $DATA_VOLUME $USER_VOLUME $DEV_VOLUME \
    "$CONTAINER_IMAGE" \
    bash -lc '
      set -e

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

      SCRIPT_DIR="$ROS_WS/script"
      if [ -n "$ROS_WS" ] && [ -f "$SCRIPT_DIR/csv_receive_all.py" ]; then
        echo "[container] start csv_receive_all.py ..."
        cd "$SCRIPT_DIR"
        nohup python3 csv_receive_all.py > /tmp/csv_receive.log 2>&1 &
        CSV_PID=$!
        echo "[container] csv_receive_all.py 已启动 (PID: $CSV_PID)，日志: /tmp/csv_receive.log"
        cd "$ROS_WS"  # 回到工作空间根目录
      else
        echo "[container] WARN: $SCRIPT_DIR/csv_receive_all.py 不存在，跳过启动"
      fi

      # 默认命令：跟随模式的 launch（你也可以在调用脚本时覆盖）
      if [ -z "$TGT_CMD" ]; then
        TGT_CMD="ros2 launch main follow.launch.py"
      fi

      echo "[container] exec: $TGT_CMD"
      eval "$TGT_CMD" || true
      # 命令结束后留在交互 shell，便于看日志/排障
      exec bash
    '
