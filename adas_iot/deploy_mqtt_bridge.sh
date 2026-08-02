#!/usr/bin/env bash
# ADAS MQTT Bridge — Jetson Orin Nano 部署脚本
# =============================================
# 一键部署 MQTT IoT 桥接器到 Orin Nano
#
# 用法:
#   ./deploy_mqtt_bridge.sh                  # 部署到 ~/adas/
#   ./deploy_mqtt_bridge.sh --user jetson    # 指定用户名
#   ./deploy_mqtt_bridge.sh --ip 192.168.100.32

set -e

DIR="$(cd "$(dirname "$0")/.." && pwd)"
ORIN_USER="${ORIN_USER:-jetson}"
ORIN_IP="${ORIN_IP:-192.168.100.32}"
ORIN_DIR="~/adas/adas_soc_ws/src/system/adas_mqtt_bridge"

echo "=========================================="
echo " ADAS MQTT Bridge — Orin Nano 部署"
echo " 目标: ${ORIN_USER}@${ORIN_IP}"
echo "=========================================="

# 1. 构建传输包
BUILD_DIR=$(mktemp -d)
echo "创建传输包: ${BUILD_DIR}"

mkdir -p "${BUILD_DIR}/adas_mqtt_bridge"
cp -r "${DIR}/ADAS0.0.2/SoC/src/system/adas_mqtt_bridge/"* "${BUILD_DIR}/adas_mqtt_bridge/"

# 清理 pyc
find "${BUILD_DIR}" -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
find "${BUILD_DIR}" -name "*.pyc" -delete

# 2. 复制到 Orin
echo ""
echo "▶ 复制到 Orin ${ORIN_IP}..."
ssh "${ORIN_USER}@${ORIN_IP}" "mkdir -p ${ORIN_DIR}"
scp -r "${BUILD_DIR}/adas_mqtt_bridge/" "${ORIN_USER}@${ORIN_IP}:${ORIN_DIR}/"

# 3. 安装依赖并构建
echo ""
echo "▶ 在 Orin 上安装依赖..."
ssh "${ORIN_USER}@${ORIN_IP}" "
    # 安装 Python 依赖
    pip3 install paho-mqtt pyyaml --break-system-packages 2>/dev/null || true

    # 构建 ROS2 包
    cd ~/adas/adas_soc_ws
    source /opt/ros/humble/setup.bash
    colcon build --packages-select adas_mqtt_bridge --symlink-install 2>&1 | tail -5

    echo '✅ 构建完成'
"

# 4. 验证
echo ""
echo "▶ 验证部署..."
ssh "${ORIN_USER}@${ORIN_IP}" "
    source /opt/ros/humble/setup.bash
    source ~/adas/adas_soc_ws/install/setup.bash
    ros2 pkg list | grep mqtt_bridge
"

# 5. 一键启动（可选）
echo ""
echo "=========================================="
echo " 部署完成！"
echo "=========================================="
echo ""
echo "SSH 到 Orin 后运行:"
echo "  ssh ${ORIN_USER}@${ORIN_IP}"
echo "  source /opt/ros/humble/setup.bash"
echo "  source ~/adas/adas_soc_ws/install/setup.bash"
echo "  ros2 launch adas_mqtt_bridge mqtt_bridge.launch.py"
echo ""
echo "或使用 systemd 服务（推荐）:"
echo "  sudo tee /etc/systemd/system/adas-mqtt.service << 'EOF'"
echo "  [Unit]"
echo "  Description=ADAS MQTT IoT Bridge"
echo "  After=network.target"
echo "  [Service]"
echo "  Type=simple"
echo "  User=${ORIN_USER}"
echo "  ExecSource=/bin/bash -c 'source /opt/ros/humble/setup.bash && source ~/adas/adas_soc_ws/install/setup.bash && ros2 launch adas_mqtt_bridge mqtt_bridge.launch.py'"
echo "  Restart=on-failure"
echo "  [Install]"
echo "  WantedBy=multi-user.target"
echo "  EOF"
echo "  sudo systemctl enable adas-mqtt.service"
echo "  sudo systemctl start adas-mqtt.service"

# 清理
rm -rf "${BUILD_DIR}"
