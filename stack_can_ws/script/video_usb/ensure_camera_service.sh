#!/usr/bin/env bash
set -euo pipefail
MONITOR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUNTIME_DIR="$MONITOR_DIR/.runtime"
mkdir -p "$RUNTIME_DIR"
exec 9>"$RUNTIME_DIR/launch.lock"
flock -x 9
probe() {
  python3 - "$RUNTIME_DIR/control.sock" <<'PY'
import socket, sys
s=socket.socket(socket.AF_UNIX)
s.settimeout(2)
try:
    s.connect(sys.argv[1])
    s.sendall(b'{"action":"status","request_id":"launcher-probe"}\n')
    if not s.recv(8192):
        raise OSError('no response')
except OSError:
    sys.exit(1)
finally:
    s.close()
PY
}
if probe 2>/dev/null; then exit 0; fi
nohup python3 "$MONITOR_DIR/camera_control_service.py" > "$RUNTIME_DIR/service.log" 2>&1 9>&- < /dev/null &
agent_pid=$!
for attempt in {1..50}; do
  if probe 2>/dev/null; then
    echo "[camera] 宿主机相机控制服务已就绪"
    exit 0
  fi
  if ! kill -0 "$agent_pid" 2>/dev/null; then break; fi
  sleep 0.1
done
echo "[camera] 相机控制服务启动失败，查看 $RUNTIME_DIR/service.log" >&2
exit 1
