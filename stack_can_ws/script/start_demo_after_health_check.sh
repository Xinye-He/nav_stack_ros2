#!/usr/bin/env bash
set -e
set -o pipefail

ROS_WS="${ROS_WS:-/root/stack_can_ws}"
HEALTH_LOG="${HEALTH_LOG:-/tmp/startup_health_check.log}"
DEMO_LAUNCH="${DEMO_LAUNCH:-ros2 launch main demo.launch.py}"

run_health_check() {
    python3 "$ROS_WS/script/startup_health_check.py" \
        --once \
        --check-fix \
        --check-heading \
        --check-lidar \
        --fix-topic /fix \
        --heading-topic /heading_deg \
        --lidar-topic /rslidar_points \
        --startup-timeout "${STARTUP_HEALTH_TIMEOUT:-3.0}" \
        --runtime-timeout "${RUNTIME_HEALTH_TIMEOUT:-2.0}"
}

echo "[container] run startup health check ..."
if run_health_check 2>&1 | tee "$HEALTH_LOG"; then
    echo "[container] startup health check passed, launching demo.launch.py ..."
    exec bash -lc "$DEMO_LAUNCH"
fi

echo ""
echo "[container] DEMO 启动检查失败，demo.launch.py 未启动。"
echo "[container] 基础节点仍在后台运行；日志："
echo "  - $HEALTH_LOG"
echo "  - /tmp/docker_startup.log"
echo ""
echo "[container] 修复 RTK/雷达/网络后，在当前容器里重新执行："
echo "  $ROS_WS/script/start_demo_after_health_check.sh"
exit 2
