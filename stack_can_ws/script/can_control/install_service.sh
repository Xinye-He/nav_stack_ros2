#!/usr/bin/env bash
set -euo pipefail
if [ "$(id -u)" -ne 0 ]; then echo '请使用 sudo 运行此安装脚本' >&2; exit 1; fi
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
install -d -m 0770 -o root -g docker "$SCRIPT_DIR/.runtime"
cat > /etc/systemd/system/nav-can-control.service <<EOF
[Unit]
Description=Navigation CAN read-only status and guarded BUS-OFF recovery
Before=nav-stack-ros2.service
[Service]
Type=simple
User=root
Group=docker
ExecStart=/usr/bin/python3 "$SCRIPT_DIR/service.py"
Environment=PYTHONDONTWRITEBYTECODE=1
Restart=on-failure
RestartSec=2
NoNewPrivileges=true
CapabilityBoundingSet=CAP_NET_ADMIN
ProtectSystem=strict
ProtectHome=read-only
ReadWritePaths=$SCRIPT_DIR/.runtime
PrivateTmp=true
ProtectKernelTunables=true
ProtectControlGroups=true
RestrictAddressFamilies=AF_UNIX AF_NETLINK
[Install]
WantedBy=multi-user.target
EOF
install -d /etc/systemd/system/nav-stack-ros2.service.d
cat > /etc/systemd/system/nav-stack-ros2.service.d/can-control.conf <<'EOF'
[Unit]
Wants=nav-can-control.service
After=nav-can-control.service
EOF
systemctl daemon-reload
systemctl enable --now nav-can-control.service
