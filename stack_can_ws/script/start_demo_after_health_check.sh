#!/usr/bin/env bash
set -e
set -o pipefail

ROS_WS="${ROS_WS:-/root/stack_can_ws}"
HEALTH_LOG="${HEALTH_LOG:-/tmp/health_monitor_startup.log}"
DEMO_LAUNCH="${DEMO_LAUNCH:-ros2 launch main demo.launch.py}"

run_health_check() {
    python3 "$ROS_WS/script/health_monitor.py" \
        --once \
        --check-fix \
        --check-heading \
        --check-lidar \
        --check-ultrasonic \
        --check-can \
        --fix-topic /fix \
        --heading-topic /heading_deg \
        --lidar-topic /rslidar_points \
        --ultrasonic-topic /ultrasonic_distances \
        --can-feedback-topic /stack_can/feedback \
        --startup-timeout "${STARTUP_HEALTH_TIMEOUT:-5.0}" \
        --runtime-timeout "${RUNTIME_HEALTH_TIMEOUT:-2.0}"
}

python3 "$ROS_WS/script/status_store.py" starting "正在检查外设"
echo "[container] run startup health check ..."
if run_health_check 2>&1 | tee "$HEALTH_LOG"; then
    echo "[container] startup health check passed, launching demo.launch.py ..."
    python3 - "$ROS_WS/script" <<'STATUS'
import sys, time
sys.path.insert(0, sys.argv[1])
from status_store import update
update('starting', '外设检查通过，正在启动控制程序', phase='launching', launch_at=time.monotonic())
STATUS
    set +e
    bash -lc "$DEMO_LAUNCH"
    result=$?
    python3 - "$ROS_WS/script" "$result" <<'STATUS'
import sys
sys.path.insert(0, sys.argv[1])
from status_store import read, update
state = read()
update('runtime_fault' if state.get('ever_ready') else 'startup_failed',
       '控制程序已退出，退出码：' + sys.argv[2])
STATUS
    exit "$result"
fi

echo ""
echo "[container] DEMO 启动检查失败，demo.launch.py 未启动。"
echo "[container] 基础节点仍在后台运行；日志："
echo "  - $HEALTH_LOG"
echo "  - /tmp/docker_startup.log"
echo ""
echo "[container] 修复 RTK/航向/雷达/超声波/CAN 后，在当前容器里重新执行："
echo "  $ROS_WS/script/start_demo_after_health_check.sh"
python3 "$ROS_WS/script/status_store.py" startup_failed "外设启动检查失败，请查看各外设错误"
exit 2
