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
DOCKER_WORKDIR="${DOCKER_WORKDIR:-/root/stack_can_ws}"

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
print_var "DOCKER_WORKDIR"
print_var "V4L2_DEVICES"
print_var "DISPLAY_DEVICE"

cat /proc/device-tree/model > /tmp/nv_jetson_model

# Camera scripts must run on the host, not inside the navigation container.
if ! bash "$STACK_CAN_WS_DIR/script/video_usb/ensure_camera_service.sh"; then
    echo "[camera] 相机远控不可用，导航仍可启动；相机指令将返回具体错误。" >&2
fi

# systemd has no interactive terminal. Keep the usual manual launch unchanged.
DEMO_NONINTERACTIVE="${DEMO_NONINTERACTIVE:-0}"
DOCKER_TERMINAL_ARGS=(-it)
if [ "$DEMO_NONINTERACTIVE" = "1" ]; then
    DOCKER_TERMINAL_ARGS=()
fi

# Remote restart (and its installation) arms persistent safe startup.
NAV_START_PAUSED="${NAV_START_PAUSED:-0}"
if [ -f "$STACK_CAN_WS_DIR/script/device_control/.runtime/start_paused.json" ]; then
    NAV_START_PAUSED=1
fi

# 避免容器名冲突
docker rm -f nav_stack_ros2 >/dev/null 2>&1 || true

docker run --runtime nvidia "${DOCKER_TERMINAL_ARGS[@]}" --rm \
    --privileged \
    --name nav_stack_ros2 \
    --network host \
    -e ROS_DISTRO="${ROS_DISTRO}" \
    -e ROS_WS="${ROS_WS:-$DOCKER_ROOT/stack_can_ws}" \
    -e TGT_CMD="${USER_COMMAND:-}" \
    -e START_DOCKER_STARTUP \
    -e DEMO_NONINTERACTIVE="$DEMO_NONINTERACTIVE" \
    -e NAV_START_PAUSED="$NAV_START_PAUSED" \
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
    -w "$DOCKER_WORKDIR" \
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

      export NAV_STATUS_FILE="/tmp/nav_device_status.json"
      python3 "$ROS_WS/script/status_store.py" starting "正在启动外设" --reset
      rm -f "$NAV_STATUS_FILE.ready"
      nohup python3 "$ROS_WS/script/status_service.py" > /tmp/device_status.log 2>&1 &
      STATUS_PID=$!
      for attempt in {1..50}; do
        if [ -f "$NAV_STATUS_FILE.ready" ]; then break; fi
        if ! kill -0 "$STATUS_PID" 2>/dev/null; then break; fi
        sleep 0.1
      done
      if [ ! -f "$NAV_STATUS_FILE.ready" ] || ! kill -0 "$STATUS_PID" 2>/dev/null; then
        echo "[container] 状态服务启动失败，请查看 /tmp/device_status.log"
        cat /tmp/device_status.log
        if [ "${DEMO_NONINTERACTIVE:-0}" = "1" ]; then exit 1; fi
        exec bash
      fi

      START_DOCKER_STARTUP="${START_DOCKER_STARTUP:-true}"
      if [ "$START_DOCKER_STARTUP" = "true" ]; then
        echo "[container] start docker_startup.launch.py in background ..."
        nohup ros2 launch main docker_startup.launch.py > /tmp/docker_startup.log 2>&1 &
        STARTUP_PID=$!
        echo "[container] docker_startup.launch.py 已启动 (PID: $STARTUP_PID)，日志: /tmp/docker_startup.log"
        sleep 1
      fi

      # 进入 ROS 工作空间，确保后续命令和最终交互 shell 都停留在 /root/stack_can_ws。
      cd "$ROS_WS" 2>/dev/null || cd /root/stack_can_ws 2>/dev/null || true

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
      if [ "${DEMO_NONINTERACTIVE:-0}" = "1" ]; then
        # Retain telemetry after startup/runtime failure, without restarting motion.
        echo "[container] 后台模式：保留状态服务；修复故障后手动重新启动控制程序。"
        wait "$STATUS_PID"
        exit 1
      fi
      # 命令结束后留在交互 shell，便于看日志/排障
      exec bash
    '
