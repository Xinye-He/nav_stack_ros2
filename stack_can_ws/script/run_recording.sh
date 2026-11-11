#!/usr/bin/env bash
# Source inside the navigation container after the ROS environment is loaded.
nav_record_start() {
    local root="${ROS_WS}/data/run_logs" available
    mkdir -p "$root" || return 1
    available=$(df -Pk "$root" | awk 'NR==2 {print $4}')
    if [[ ! "$available" =~ ^[0-9]+$ ]] || (( available < 5242880 )); then
        echo '[recording] 磁盘不足5GiB，未启动持久化记录' >&2
        return 1
    fi
    export NAV_RUN_LOG_DIR
    NAV_RUN_LOG_DIR=$(mktemp -d "$root/$(date +%Y%m%d_%H%M%S)_XXXXXX") || return 1
    export ROS_LOG_DIR="$NAV_RUN_LOG_DIR/ros"
    mkdir -p "$ROS_LOG_DIR" || return 1
    export HEALTH_LOG="$NAV_RUN_LOG_DIR/health_monitor_startup.log"
    echo "[recording] 本次记录: $NAV_RUN_LOG_DIR"
    export NAV_STARTUP_LOG="$NAV_RUN_LOG_DIR/docker_startup.log"
    export NAV_DEVICE_LOG="$NAV_RUN_LOG_DIR/device_status.log"
    # The log supervisor reads stdout/stderr, rotates files and mirrors to console.
    exec > >(python3 "$ROS_WS/script/run_recording_log.py" "$NAV_RUN_LOG_DIR/navigation") 2>&1
    (
        ros2 bag record -o "$NAV_RUN_LOG_DIR/topics" -b 268435456 \
            /rosout /bale_target /at_task_waiting /bale_active /bale_controller/status \
            /stack_cmd/bale /stack_cmd/traj /stack_cmd/websocket /stack_can/status \
            /vehicle/execution_status /drive_cmd /abort /reset_estop /task_done \
            /fix /fix_center_raw /fix_center /heading_deg /vehicle_heading_deg \
            /safety/network_result /vehicle/reset_result > "$NAV_RUN_LOG_DIR/bag.log" 2>&1 &
        bag_pid=$!
        while kill -0 "$bag_pid" 2>/dev/null; do
            sleep 10
            available=$(df -Pk "$NAV_RUN_LOG_DIR" | awk 'NR==2 {print $4}')
            used=$(du -sk "$NAV_RUN_LOG_DIR/topics" 2>/dev/null | awk '{print $1}')
            if [[ ! "$available" =~ ^[0-9]+$ ]] || [[ ! "$used" =~ ^[0-9]+$ ]] || (( available < 5242880 || used >= 2097152 )); then
                echo '[recording] 录制达到2GiB或磁盘剩余不足5GiB，停止话题录制' >> "$NAV_RUN_LOG_DIR/bag.log"
                kill -INT "$bag_pid" 2>/dev/null || true
                break
            fi
        done
        wait "$bag_pid"
    ) &
    echo "[recording] 话题记录进程已启动"
}
