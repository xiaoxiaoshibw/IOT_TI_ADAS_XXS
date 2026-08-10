#!/usr/bin/env bash
# 安装 / 卸载 CARLA supervisor 与 port_guard systemd 单元
#
# 用法：
#   install_supervisor.sh install    # 安装并启用
#   install_supervisor.sh uninstall  # 停用并删除
#   install_supervisor.sh status     # 查看当前状态
#
# 默认装到 /etc/systemd/system/。如需用户级 systemd，运行：
#   systemctl --user link $HOME/.config/systemd/user/carla-supervisor.service

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SYSTEMD_DIR="${REPO_ROOT}/adas_bridge_pc/systemd"

usage() {
  cat <<EOF
用法: $0 <install|uninstall|status>
EOF
}

install_units() {
  echo "[install] 复制 systemd unit 到 /etc/systemd/system/"
  sudo install -m 0644 \
    "${SYSTEMD_DIR}/carla-supervisor.service" \
    "${SYSTEMD_DIR}/adas-port-guard.service" \
    /etc/systemd/system/
  sudo systemctl daemon-reload
  sudo systemctl enable --now carla-supervisor.service
  sudo systemctl enable --now adas-port-guard.service
  echo "[install] 完成"
  systemctl --no-pager status carla-supervisor.service adas-port-guard.service || true
}

uninstall_units() {
  echo "[uninstall] 停用并删除 systemd unit"
  sudo systemctl disable --now carla-supervisor.service || true
  sudo systemctl disable --now adas-port-guard.service || true
  sudo rm -f /etc/systemd/system/carla-supervisor.service \
            /etc/systemd/system/adas-port-guard.service
  sudo systemctl daemon-reload
  echo "[uninstall] 完成"
}

status_units() {
  echo "[status] carla-supervisor:"
  systemctl --no-pager status carla-supervisor.service || true
  echo "[status] adas-port-guard:"
  systemctl --no-pager status adas-port-guard.service || true
}

case "${1:-}" in
  install) install_units ;;
  uninstall) uninstall_units ;;
  status) status_units ;;
  *) usage; exit 1 ;;
esac
