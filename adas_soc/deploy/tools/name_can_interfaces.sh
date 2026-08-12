#!/usr/bin/env bash
# Give the HIL PEAK adapter a stable name regardless of driver probe order.
set -euo pipefail

driver_of() {
  ethtool -i "$1" 2>/dev/null | awk '$1 == "driver:" { print $2 }'
}

peak_if=""
mttcan_if=""
for _ in $(seq 1 20); do
  peak_if=""
  mttcan_if=""
  for path in /sys/class/net/can*; do
    [ -e "$path" ] || continue
    iface="${path##*/}"
    case "$(driver_of "$iface")" in
      pcan) peak_if="$iface" ;;
      mttcan) mttcan_if="$iface" ;;
    esac
  done
  [ -n "$peak_if" ] && [ -n "$mttcan_if" ] && break
  sleep 0.25
done

[ -n "$peak_if" ] || { echo "PEAK pcan interface not found" >&2; exit 1; }
[ -n "$mttcan_if" ] || { echo "on-board mttcan interface not found" >&2; exit 1; }

if [ "$peak_if" = can1 ] && [ "$mttcan_if" = can0 ]; then
  exit 0
fi
[ "$peak_if" = can0 ] && [ "$mttcan_if" = can1 ] || {
  echo "unexpected CAN names: pcan=$peak_if mttcan=$mttcan_if" >&2
  exit 1
}

ip link set can0 down || true
ip link set can1 down || true
ip link set can1 name adas_mttcan_tmp
ip link set can0 name can1
ip link set adas_mttcan_tmp name can0

[ "$(driver_of can1)" = pcan ]
[ "$(driver_of can0)" = mttcan ]
