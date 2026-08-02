#!/usr/bin/env bash
# 在 PC 上实时查看 Orin USB 摄像头画面（topic /adas/sensors/camera/image_raw）。
# 关键三件套：domain 43 + CycloneDDS（必须，否则默认 FastRTPS 收不到 Orin）+ 指定直连网卡。
set -e
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=43
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# PC 有多块网卡（wifi + 直连 Orin 的 USB 网卡），必须绑直连 Orin 那块，
# 否则 CycloneDDS 可能选到 wifi 导致收不到。换网卡/USB 口后用 `ip -brief addr` 重新确认网卡名。
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enx00e04c176a70"/></Interfaces></General></Domain></CycloneDDS>'
export RCUTILS_LOGGING_MIN_SEVERITY=ERROR   # 抑制 Humble<->Jazzy 跨版本 type-hash 警告

TOPIC="${1:-/adas/sensors/camera/image_raw}"
echo "查看 $TOPIC （Ctrl-C 退出）..."
exec ros2 run rqt_image_view rqt_image_view "$TOPIC"
