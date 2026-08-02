#!/usr/bin/env bash
set -Eeuo pipefail
HIL_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=adas_pc/tools/hil/hil_common.sh
source "${HIL_DIR}/hil_common.sh"

"${HIL_DIR}/check_hil_preflight.sh"
exec "${ADAS_PC_DIR}/start_pc_stack_clean.sh" "$@"
