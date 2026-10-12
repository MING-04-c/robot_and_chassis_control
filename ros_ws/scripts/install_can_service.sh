#!/usr/bin/env bash
# Install setup_can.sh as a systemd service so can0 is configured at boot.

set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
    echo "请使用 sudo 运行：sudo ./scripts/install_can_service.sh" >&2
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SERVICE_SOURCE="${WORKSPACE_DIR}/systemd/masc-chassis-can.service"
SERVICE_TARGET="/etc/systemd/system/masc-chassis-can.service"
SETUP_TARGET="/usr/local/lib/masc-chassis/setup_can.sh"

install -D -m 0755 "${SCRIPT_DIR}/setup_can.sh" "${SETUP_TARGET}"
install -D -m 0644 "${SERVICE_SOURCE}" "${SERVICE_TARGET}"

systemctl daemon-reload
systemctl enable masc-chassis-can.service
systemctl restart masc-chassis-can.service

echo
echo "CAN 开机服务已安装并启动。"
systemctl --no-pager --full status masc-chassis-can.service
