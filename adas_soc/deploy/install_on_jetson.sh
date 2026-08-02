#!/usr/bin/env bash
# 在 Jetson/Humble 上以 root 运行一次：把已构建的工作区装配为版本化
# v3 候选发布，安装 systemd 模板并保持 HIL 控制服务常驻。
# v3 重构(2026-07)后：启动服务并待 CAN 流新鲜稳定，会话自动走到 ACTIVE，
# 无需操作员显式 ARM/REARM 或物理按钮授权。
#
# 用法: sudo bash install_on_jetson.sh <version> [workspace]
#   version    发布版本号，如 0.0.17-v3-test
#   workspace  已构建的 colcon 工作区，默认 /home/jetson/adas/adas_soc_ws
set -euo pipefail

VERSION="${1:?usage: sudo bash install_on_jetson.sh <version> [workspace]}"
WORKSPACE="${2:-/home/jetson/adas/adas_soc_ws}"
DEPLOY_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEPLOY_TOOLS="${DEPLOY_DIR}/tools"
ROOT=/opt/adas
RELEASE="${ROOT}/releases/${VERSION}"
PROTOCOL_VERSION="${ADAS_PROTOCOL_VERSION:-3}"

[ "$(id -u)" -eq 0 ] || { echo "must run as root" >&2; exit 1; }
[ -r "${WORKSPACE}/install/setup.bash" ] || {
  echo "workspace not built: ${WORKSPACE}" >&2; exit 1; }
[ -r "${DEPLOY_TOOLS}/release_integrity.py" ] || {
  echo "missing ${DEPLOY_TOOLS}/release_integrity.py" >&2; exit 1; }
[ -r "${DEPLOY_TOOLS}/deploy_release.py" ] || {
  echo "missing ${DEPLOY_TOOLS}/deploy_release.py" >&2; exit 1; }

# A previously installed keeper must not race the intentional stop/symlink
# switch performed by deploy_release.py. It is re-enabled after activation.
systemctl disable --now adas-hil-keeper.timer >/dev/null 2>&1 || true

echo "== assemble release ${RELEASE}"
mkdir -p "${RELEASE}" "${ROOT}/releases" /etc/adas /var/log/adas
# 工作区开发期使用 --symlink-install；发布目录必须解引用这些链接，
# 否则完整性清单既无法覆盖源码外链接，也不能形成可独立回滚的发布物。
rsync -aL --delete "${WORKSPACE}/install/" "${RELEASE}/install/"
rsync -a --delete "${DEPLOY_TOOLS}/" "${RELEASE}/tools/"
chown -R jetson:jetson /var/log/adas

# A development workspace is commonly built with --symlink-install. For
# ament_python packages that leaves pythonpath_develop hooks pointing back to
# WORKSPACE/build; rsync -aL cannot make those textual hooks self-contained.
# Reinstall the two runtime Python packages directly into the release prefix.
PYTHON_BUILD_DIR="$(mktemp -d /var/tmp/adas-release-python.XXXXXX)"
trap 'rm -rf -- "${PYTHON_BUILD_DIR}"' EXIT
set +u
source "/opt/ros/${ADAS_ROS_DISTRO:-humble}/setup.bash"
set -u
colcon build \
  --base-paths \
    "${WORKSPACE}/src/system/adas_resource_monitor" \
    "${WORKSPACE}/src/system/adas_dtc_recorder" \
  --packages-select adas_resource_monitor adas_dtc_recorder \
  --build-base "${PYTHON_BUILD_DIR}/build" \
  --install-base "${RELEASE}/install"
rm -rf -- "${PYTHON_BUILD_DIR}"
trap - EXIT

echo "== create integrity manifest"
GIT_COMMIT="${ADAS_GIT_COMMIT:-unknown}"
python3 "${RELEASE}/tools/release_integrity.py" create \
  "${RELEASE}" "${RELEASE}/release_manifest.json" \
  --protocol-version "${PROTOCOL_VERSION}" --git-commit "${GIT_COMMIT}"
python3 "${RELEASE}/tools/release_integrity.py" verify \
  "${RELEASE}" "${RELEASE}/release_manifest.json" \
  --protocol-version "${PROTOCOL_VERSION}"
# adas-hil.service 以 jetson 用户运行，其 ExecStartPre 必须能读取清单与发布物。
# 所有权变更不修改清单覆盖的文件内容，仍由上一行校验保证完整性。
chown -R jetson:jetson "${RELEASE}"

echo "== install configuration and units"
if [ ! -f /etc/adas/adas-hil.env ]; then
  cp "${DEPLOY_DIR}/systemd/adas-hil.env.example" /etc/adas/adas-hil.env
  echo "   wrote /etc/adas/adas-hil.env (review before first start)"
fi
cp "${DEPLOY_DIR}/systemd/cyclonedds_orin.xml" /etc/adas/
cp "${DEPLOY_DIR}/systemd/adas-can.service" /etc/systemd/system/
cp "${DEPLOY_DIR}/systemd/adas-hil.service" /etc/systemd/system/
cp "${DEPLOY_DIR}/systemd/adas-hil-keeper.service" /etc/systemd/system/
cp "${DEPLOY_DIR}/systemd/adas-hil-keeper.timer" /etc/systemd/system/
install -d -m 0755 /etc/systemd/system/adas-hil.service.d
cp "${DEPLOY_DIR}/systemd/adas-hil.service.d/can-autostart.conf" \
  /etc/systemd/system/adas-hil.service.d/
cp "${DEPLOY_DIR}/logrotate/adas" /etc/logrotate.d/adas
systemctl daemon-reload

echo "== activate release (atomic symlink, then restart HIL onto this release)"
python3 "${RELEASE}/tools/deploy_release.py" --root "${ROOT}" activate \
  "${VERSION}" --protocol-version "${PROTOCOL_VERSION}"

systemctl enable adas-hil.service
systemctl enable --now adas-hil-keeper.timer
# start 对已运行服务是 no-op，会留下指向旧 release 的进程；restart 确保
# 激活完成后真实运行的进程与 /opt/adas/current 一致。keeper 此时已启用，
# 即使 restart 本身异常退出也会继续拉起。
systemctl restart adas-hil.service
echo "== done"
systemctl --no-pager status adas-hil.service | head -20 || true
echo
echo "常驻服务:     adas-hil.service + adas-hil-keeper.timer（已启用并运行）"
echo "授权控制:     v3 重构(2026-07)后无需任何授权——CAN 流新鲜稳定后 MCU 自动进入 ACTIVE"
echo "恢复通信:     自动——链路恢复后 MCU 自愈回 ACTIVE，无需 ARM/REARM/按钮"
