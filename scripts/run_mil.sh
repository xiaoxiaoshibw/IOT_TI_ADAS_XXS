#!/usr/bin/env bash
# Canonical entrypoint for the local MIL simulated-hardware loop.
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$ROOT/scripts/run_local_three_machine_hil.sh" "$@"
