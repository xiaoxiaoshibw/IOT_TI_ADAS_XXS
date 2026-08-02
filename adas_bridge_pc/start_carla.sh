#!/usr/bin/env bash
# Start CARLA. Configure CARLA_ROOT; override CARLA_ARGS when needed.

set -euo pipefail
source "$(cd "$(dirname "$0")" && pwd)/scripts/common.sh"

CARLA_ARGS="${CARLA_ARGS:--quality-level=Epic}"
read -r -a carla_args <<< "${CARLA_ARGS}"
# 注意：本机打包版 CARLA 0.9.16(Shipping) 忽略命令行位置参数地图，始终按
# CarlaUE4/Config/DefaultEngine.ini 的 GameDefaultMap 开机（当前 Town10HD_Opt）。
# 项目默认地图不在此处固化，而是由桥用 client.load_world("${TOWN}") 权威加载
# （common.sh 的 TOWN 为单一事实源；carla_world.py 已带"已是目标图则跳过重载"守卫）。
# 这样既不改动这台共享 PC 的 CARLA 全局默认（会波及他人），又保证系统必落到 TOWN。
exec "$(carla_executable)" "${carla_args[@]}" "$@"
