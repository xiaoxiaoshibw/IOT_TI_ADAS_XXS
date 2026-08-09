#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
mcu="$root/adas_mcu"
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT
cc=(gcc -std=c99 -Wall -Wextra -Werror -I "$mcu/include")

"${cc[@]}" "$mcu/src/safety.c" "$mcu/tests/test_safety_host.c" -o "$tmp/safety"
"$tmp/safety"
"${cc[@]}" "$mcu/src/crc8.c" "$mcu/tests/test_crc_host.c" -o "$tmp/crc"
"$tmp/crc"
"${cc[@]}" "$mcu/src/control.c" "$mcu/tests/test_control_host.c" -o "$tmp/control"
"$tmp/control"
"${cc[@]}" "$mcu/src/crc8.c" "$mcu/src/self_test.c" \
  "$mcu/tests/test_self_test_host.c" -o "$tmp/self_test"
"$tmp/self_test"
"${cc[@]}" "$mcu/src/hil_session.c" "$mcu/tests/test_hil_session_host.c" \
  -o "$tmp/hil_session"
"$tmp/hil_session"

# ASR-PRO 帧解析器（停在 §2 修复的核心反例上：带 arg 后接 0x0C 0x0D 新帧不能丢）
cc_asr=(gcc -std=c99 -Wall -Wextra -Werror -I "$mcu/include" \
        -I "$mcu/tests/stubs")
"${cc_asr[@]}" "$mcu/src/asr_pro.c" "$mcu/tests/test_asr_parser_host.c" \
  -o "$tmp/asr_parser"
"$tmp/asr_parser"

gcc -std=c99 -Wall -Wextra -Werror -DADAS_HOST_TEST=1 \
  -I "$mcu/tests/stubs" -I "$mcu/include" -c "$mcu/src/can_comm.c" \
  -o "$tmp/can_comm.o"
echo "can communication host compile: PASS"

gcc -std=c99 -Wall -Wextra -Werror -DADAS_HOST_TEST=1 \
  -I "$mcu/tests/stubs" -I "$mcu/include" -c "$mcu/src/oled2.c" \
  -o "$tmp/oled2.o"
echo "OLED2 data display host compile: PASS"

gcc -std=c99 -Wall -Wextra -Werror -DADAS_HOST_TEST=1 \
  -I "$mcu/tests/stubs" -I "$mcu/include" -c "$mcu/src/dgus_screen.c" \
  -o "$tmp/dgus_screen.o"
echo "DGUS touch screen host compile: PASS"

if gcc -std=c99 -Wall -Wextra -Werror -DADAS_TEST_OMIT_BUS_OFF=1 \
  -I "$mcu/tests/stubs" -I "$mcu/include" -c "$mcu/src/can_comm.c" \
  -o "$tmp/guard.o" 2>"$tmp/guard.log"; then
  echo "bus-off guard negative test unexpectedly compiled" >&2
  exit 1
fi
grep -q "bus-off recovery cannot be disabled" "$tmp/guard.log"
echo "bus-off compile guard negative test: PASS"

python3 "$mcu/tests/test_protocol_reference.py"

# main.c 需要 driverlib/C28x 符号，host 上无法链接；但用 stubs 做 -fsyntax-only
# 可以提前捕获"删了声明/字段却留了引用"这一类只在 main.c 出现的回归。
gcc -std=c99 -Wall -Wextra -Werror -Wno-main -DADAS_HOST_TEST=1 \
  -I "$mcu/tests/stubs" -I "$mcu/include" -fsyntax-only "$mcu/src/main.c"
echo "main.c host syntax check: PASS"
