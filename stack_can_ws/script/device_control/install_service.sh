#!/usr/bin/env bash
set -euo pipefail
if [ "$(id -u)" -ne 0 ]; then echo '请使用sudo安装宿主机设备管理服务' >&2; exit 1; fi
CONTROL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
install -d -m 0770 -o root -g docker "$CONTROL_DIR/.runtime"
# First installation also makes the bootstrap restart safely paused.
if [ ! -f "$CONTROL_DIR/.runtime/start_paused.json" ]; then
    printf '%s\n' '{"startup_mode":"paused","reason":"device_control_installation"}' > "$CONTROL_DIR/.runtime/start_paused.json"
fi
cat > /etc/systemd/system/nav-device-control.service <<EOF
[Unit]
Description=Guarded navigation restart coordinator
Before=nav-stack-ros2.service
[Service]
Type=simple
User=root
Group=docker
ExecStart=/usr/bin/python3 "$CONTROL_DIR/service.py"
Environment=PYTHONDONTWRITEBYTECODE=1
Restart=on-failure
RestartSec=2
NoNewPrivileges=true
CapabilityBoundingSet=
ProtectSystem=strict
ProtectHome=read-only
ReadWritePaths=$CONTROL_DIR/.runtime
PrivateTmp=true
ProtectKernelTunables=true
ProtectControlGroups=true
RestrictAddressFamilies=AF_UNIX
[Install]
WantedBy=multi-user.target
EOF
install -d /etc/systemd/system/nav-stack-ros2.service.d
cat > /etc/systemd/system/nav-stack-ros2.service.d/device-control.conf <<'EOF'
[Unit]
Wants=nav-device-control.service
After=nav-device-control.service
EOF
systemctl daemon-reload
systemctl enable --now nav-device-control.service
