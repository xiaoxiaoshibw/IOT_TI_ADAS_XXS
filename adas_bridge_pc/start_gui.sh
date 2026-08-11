#!/usr/bin/env bash
# Start the Qt6 ADAS safety dashboard.

set -euo pipefail

# 后端必须在加载 common.sh 前确定；common.sh 的默认值属于 HIL（domain 43）。
# 显式选择 MIL 时强制使用本机模拟硬件约定，避免继承终端里的 HIL DDS 环境。
requested_backend="${ADAS_GUI_BACKEND:-carla_local_soc}"
args=("$@")
for ((i = 0; i < ${#args[@]}; ++i)); do
  case "${args[i]}" in
    --backend)
      if ((i + 1 < ${#args[@]})); then requested_backend="${args[i + 1]}"; fi
      ;;
    --backend=*) requested_backend="${args[i]#--backend=}" ;;
  esac
done

case "${requested_backend}" in
  mil|carla_local_soc)
    export ADAS_GUI_BACKEND="${requested_backend}"
    export ROS_DOMAIN_ID=145
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    unset CYCLONEDDS_URI
    ;;
  hil)
    export ADAS_GUI_BACKEND=hil
    export ROS_DOMAIN_ID=43
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    ;;
  *)
    printf 'Error: unsupported backend %q; use mil, carla_local_soc or hil.\n' \
      "${requested_backend}" >&2
    exit 2
    ;;
esac

source "$(cd "$(dirname "$0")" && pwd)/scripts/common.sh"

source_workspace
# 从 Snap 版 VS Code 启动终端时会继承 core20 的 GTK/GIO 搜索路径，Qt 插件
# 随后加载旧 libpthread，报 __libc_pthread_init@GLIBC_PRIVATE。只清理 GUI
# 子进程的注入变量，不修改用户会话或 ROS/CARLA 环境。
unset SNAP SNAP_ARCH SNAP_COMMON SNAP_CONTEXT SNAP_COOKIE SNAP_DATA SNAP_EUID \
  SNAP_INSTANCE_NAME SNAP_LAUNCHER_ARCH_TRIPLET SNAP_LIBRARY_PATH SNAP_NAME \
  SNAP_REAL_HOME SNAP_REVISION SNAP_UID SNAP_USER_COMMON SNAP_USER_DATA SNAP_VERSION \
  GTK_EXE_PREFIX GTK_PATH GTK_MODULES GIO_MODULE_DIR
export QT_ACCESSIBILITY=0
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
exec ros2 run adas_gui adas_gui "$@"
