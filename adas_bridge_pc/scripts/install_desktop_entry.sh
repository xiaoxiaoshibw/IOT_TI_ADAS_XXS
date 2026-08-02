#!/usr/bin/env bash
# 安装桌面入口：双击图标即启动 ADAS 安全仪表（GUI 内可一键拉起 CARLA + 桥）。
# 生成 ~/.local/share/applications/adas-dashboard.desktop，Exec 指向本仓库
# 的 start_gui.sh（内部会 source ROS 环境，QProcess 子进程继承该环境）。

set -euo pipefail
source "$(cd "$(dirname "$0")" && pwd)/common.sh"

entry_dir="${HOME}/.local/share/applications"
entry="${entry_dir}/adas-dashboard.desktop"
mkdir -p "${entry_dir}"

cat > "${entry}" <<EOF
[Desktop Entry]
Type=Application
Name=ADAS Safety Dashboard
Name[zh_CN]=ADAS 安全仪表
Comment=Qt6 safety dashboard with one-click CARLA + bridge launch
Comment[zh_CN]=Qt6 安全仪表，内置一键启动 CARLA 与桥接
Exec=${REPO_ROOT}/start_gui.sh
Path=${REPO_ROOT}
Terminal=false
Categories=Development;
StartupNotify=true
EOF

chmod +x "${entry}"
update-desktop-database "${entry_dir}" 2>/dev/null || true
printf '已安装桌面入口: %s\n' "${entry}"
