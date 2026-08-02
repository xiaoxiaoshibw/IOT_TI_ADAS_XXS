#!/usr/bin/env bash
# Collect CARLA crash evidence without restarting, killing, or changing the host.
set -Eeuo pipefail

output="${1:?usage: collect_carla_crash_artifacts.sh OUTPUT_DIR [SINCE]}"
since="${2:--5 min}"
mkdir -p "${output}"

date --iso-8601=ns >"${output}/collection_time.txt"
ps -eo pid,ppid,pgid,sid,stat,etime,lstart,cmd --forest >"${output}/process_tree.txt"
{
  printf 'ulimit_c='; ulimit -c
  printf 'core_pattern='; cat /proc/sys/kernel/core_pattern
  command -v coredumpctl || true
  coredumpctl list --no-pager 2>&1 || true
} >"${output}/core_status.txt"
{
  free -h
  df -h
  df -i
  df -h /dev/shm
  ulimit -a
  ipcs -m
  ps aux --sort=-%mem | head -30
  ss -lntup
  lsof -i :2000 2>&1 || true
} >"${output}/resources.txt"
{
  command -v nvidia-smi && nvidia-smi || true
  command -v rocm-smi && rocm-smi || true
  command -v glxinfo && glxinfo -B || true
  command -v vulkaninfo && timeout 20 vulkaninfo --summary || true
  lspci -nnk | grep -A4 -Ei 'vga|3d|display' || true
} >"${output}/gpu.txt" 2>&1
journalctl -k --since "${since}" --no-pager >"${output}/kernel.log" 2>&1 || true
dmesg -T 2>&1 | tail -300 >"${output}/dmesg_tail.log" || true
journalctl --since "${since}" --no-pager >"${output}/journal.log" 2>&1 || true
find /var/crash /var/lib/apport/coredump -maxdepth 2 -type f -printf '%T@ %s %p\n' \
  >"${output}/crash_inventory.txt" 2>/dev/null || true
