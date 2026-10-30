#!/usr/bin/env bash
# 在运行中的导航容器内打开上位机完整键盘控制客户端。
set -euo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-nav_stack_ros2}"
CLIENT_DIR="/root/stack_can_ws/script/websocket"

if ! command -v docker >/dev/null 2>&1; then
    echo "未找到 docker 命令。" >&2
    exit 1
fi
if [[ ! -t 0 || ! -t 1 ]]; then
    echo "请在交互终端中运行本脚本（SSH连接请使用 ssh -t）。" >&2
    exit 1
fi
if ! running="$(docker inspect --format '{{.State.Running}}' "$CONTAINER_NAME")"; then
    echo "无法访问容器 $CONTAINER_NAME，请检查容器是否存在及Docker权限。" >&2
    exit 1
fi
if [[ "$running" != true ]]; then
    echo "容器 $CONTAINER_NAME 未运行，请先启动导航服务。" >&2
    exit 1
fi

exec docker exec -it --workdir "$CLIENT_DIR" "$CONTAINER_NAME" \
    python3 hmi_keyboard_client.py --host 127.0.0.1 --full-control
