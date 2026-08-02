#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 <workspace-install-prefix>" >&2
  exit 2
fi

install_prefix="$(realpath "$1")"
mapfile -t pids < <(pgrep -f "${install_prefix}/.*/lib/.*/.*_node" || true)
cpu=0
rss=0
pss=0

for pid in "${pids[@]}"; do
  [[ -r "/proc/${pid}/smaps_rollup" ]] || continue
  process_cpu="$(ps -p "$pid" -o pcpu= | tr -d ' ')"
  process_rss="$(ps -p "$pid" -o rss= | tr -d ' ')"
  process_pss="$(awk '/^Pss:/{print $2}' "/proc/${pid}/smaps_rollup")"
  cpu="$(awk -v a="$cpu" -v b="${process_cpu:-0}" 'BEGIN{print a+b}')"
  rss=$((rss + ${process_rss:-0}))
  pss=$((pss + ${process_pss:-0}))
  printf 'pid=%s cpu=%s%% rss_kib=%s pss_kib=%s\n' \
    "$pid" "${process_cpu:-0}" "${process_rss:-0}" "${process_pss:-0}"
done

awk -v count="${#pids[@]}" -v cpu="$cpu" -v rss="$rss" -v pss="$pss" \
  'BEGIN {printf "TOTAL processes=%d cpu_one_core_percent=%.1f rss_mib=%.1f pss_mib=%.1f\n", count, cpu, rss/1024, pss/1024}'

