#!/usr/bin/env bash
set -Eeuo pipefail
HIL_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=adas_pc/tools/hil/hil_common.sh
source "${HIL_DIR}/hil_common.sh"
load_pc_ros
exec python3 "${ADAS_PC_DIR}/scripts/check_hil_ready.py" "$@"
