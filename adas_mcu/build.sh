#!/bin/bash
# build.sh — ADAS MCU 手动构建脚本 (Linux)
set -e

CGT=/home/xxs/程序/ti/ccs/tools/compiler/ti-cgt-c2000_25.11.1.LTS/ti-cgt-c2000_25.11.1.LTS
CW=/home/xxs/程序/ti/c2000/C2000Ware_26_01_00_00

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

CC=$CGT/bin/cl2000
LD=$CGT/bin/cl2000
HEX=$CGT/bin/hex2000

PROJ="$SCRIPT_DIR"
SRC=$PROJ/src
INC=$PROJ/include
BLD=$PROJ/build
OUT=$PROJ/adas_mcu_flash.out

CW_DEV=$CW/device_support/f28002x/common
CW_DL=$CW/driverlib/f28002x/driverlib

CFLAGS="
  --silicon_version=28
  --float_support=fpu32
  --tmu_support=tmu0
  --abi=eabi
  --idiv_support
  --large_memory_model
  --unified_memory
  --gen_func_subsections
  -O2
  -g
  --diag_warning=225
  --diag_wrap=off
  --display_error_number
  --include_path=$INC
  --include_path=$CW_DL
  --include_path=$CW_DEV/include
  --include_path=$CGT/include
  --define=_DEBUG
  --define=_FLASH
"

mkdir -p $BLD

OBJS=""
for src in $SRC/*.c; do
    bn=$(basename "$src" .c)
    obj=$BLD/${bn}.obj
    echo "  CC  $bn.c"
    $CC $CFLAGS --compile_only --obj_directory=$BLD "$src"
    OBJS="$OBJS $obj"
done

# F280025C Flash Boot ROM 固定跳转到 0x80000；该入口由纯汇编文件提供，
# 不在上面的 src/*.c 循环内。漏掉它会让 BEGIN/codestart 为空，复位后
# CPU 停在 Boot ROM，即使其它应用段已成功烧入也不会自主运行。
echo "  AS  f28002x_codestartbranch.asm"
$CC $CFLAGS --compile_only --obj_directory=$BLD \
  $CW_DEV/source/f28002x_codestartbranch.asm
OBJS="$OBJS $BLD/f28002x_codestartbranch.obj"

# Compile C2000Ware device.c (copied locally due to path encoding issues)
echo "  CC  device_c2000ware.c"
$CC $CFLAGS --compile_only --obj_directory=$BLD $SRC/device_c2000ware.c
OBJS="$OBJS $BLD/device_c2000ware.obj"

echo "  LD  → adas_mcu_flash.out"
$LD $CFLAGS \
  -z -m"${BLD}/adas_mcu_flash.map" \
  --stack_size=0x200 \
  --warn_sections \
  --reread_libs \
  --rom_model \
  -o "$OUT" \
  $OBJS \
  $CW_DEV/cmd/28002x_generic_flash_lnk.cmd \
  -l${CGT}/lib/rts2800_fpu32_eabi.lib \
  ${CW_DL}/ccs/Release/driverlib.lib

echo "  HEX → adas_mcu_flash.hex"
$HEX -i "$OUT" -o "${OUT%.out}.hex" -order LS -romwidth 16

echo "=== Build OK ==="
ls -lh "$OUT"
