#!/usr/bin/env bash
# 下载 Font Awesome 6 Free Solid 字体，离线打包进 adas_gui。
# 资产来自 github.com/FortAwesome/Font-Awesome releases（OFL/CC BY 4.0 双许可）。
# 幂等：字体已存在则直接跳过。

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEST_DIR="$SCRIPT_DIR/../src/resources/assets/fonts"
DEST_OTF="$DEST_DIR/fa-solid-900.otf"
DEST_LIC="$DEST_DIR/LICENSE.txt"

if [[ -s "$DEST_OTF" && -s "$DEST_LIC" ]]; then
  echo "[fetch_fa] fa-solid-900.otf 已存在，跳过下载"
  exit 0
fi

mkdir -p "$DEST_DIR" /tmp/opencode
ZIP=/tmp/opencode/fontawesome-free-6.5.2-desktop.zip
URL="https://github.com/FortAwesome/Font-Awesome/releases/download/6.5.2/fontawesome-free-6.5.2-desktop.zip"
echo "[fetch_fa] 下载 $URL"
curl -fsSL -o "$ZIP" "$URL"
unzip -p "$ZIP" "fontawesome-free-6.5.2-desktop/otfs/Font Awesome 6 Free-Solid-900.otf" > "$DEST_OTF"
unzip -p "$ZIP" "fontawesome-free-6.5.2-desktop/LICENSE.txt" > "$DEST_LIC"
echo "[fetch_fa] 写入 $DEST_OTF"
ls -la "$DEST_DIR"