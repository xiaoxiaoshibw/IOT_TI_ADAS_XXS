#!/usr/bin/env bash
# adas_gui CLI helper：把 Orin ssh 密码写到 ~/.config/adas/adas_gui/secrets.ini，
# 跳过 GUI 首次启动的 QInputDialog。文件权限 0600，与 GUI 内部 SecureSettings
# 同一格式（QSettings IniFormat [orin] 节）。
#
# 用法：
#   ./scripts/setup_orin_creds.sh [host] [user]
#   ./scripts/setup_orin_creds.sh 192.168.100.32 jetson   # 交互式输入密码
#   printf 'yahboom\n' | ./scripts/setup_orin_creds.sh    # 走 stdin（CI 用）
#
# 不传 host/user 用默认（jetson@192.168.100.32）。

set -euo pipefail

HOST="${1:-192.168.100.32}"
USER="${2:-jetson}"
CONF_DIR="${XDG_CONFIG_HOME:-$HOME/.config}/adas/adas_gui"
CONF="$CONF_DIR/secrets.ini"

mkdir -p "$CONF_DIR"

if [ ! -t 0 ]; then
  # stdin 模式（CI / 脚本注入）
  PWD="$(cat)"
else
  # 交互式
  read -r -s -p "Orin ssh 密码（$USER@$HOST）：" PWD
  echo
fi

if [ -z "$PWD" ]; then
  echo "❌ 密码不能为空" >&2
  exit 1
fi

# 用 QSettings IniFormat：[orin] 节 + key = "orin/<host>/<user>"
# key 内部含 / 必须做 QSettings 转义？—— 见 secure_settings.cpp 实测：直接写
# "orin/host/user=value" 能被 QSettings 读回。复现这条事实，避免格式漂移。
KEY="orin/$HOST/$USER"
printf '[orin]\n%s=%s\n' "$KEY" "$PWD" > "$CONF"
chmod 600 "$CONF"

echo "✅ 已写入 $CONF（chmod 600）"
echo "   host=$HOST user=$USER key=$KEY"
echo "现在重启 adas_gui 后按「一键启动全流程」即可。"

# 校验可读
SEC_OUT="$(stat -c '%a' "$CONF")"
if [ "$SEC_OUT" != "600" ]; then
  echo "⚠️  权限不是 600（实际 $SEC_OUT），请检查 umask" >&2
fi