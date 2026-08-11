#!/usr/bin/env bash
# Shared CARLA installation resolution for PC launch and HIL preflight scripts.

resolve_carla_root() {
  if [[ -n "${CARLA_ROOT:-}" ]]; then
    printf '%s\n' "${CARLA_ROOT}"
    return 0
  fi

  local candidate
  for candidate in "${HOME}/程序/CARLA_0.9.16" "${HOME}/CARLA_0.9.16"; do
    if [[ -x "${candidate}/CarlaUE4.sh" ]]; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  done

  # Fail-closed callers still get the preferred default in their error output.
  printf '%s\n' "${HOME}/程序/CARLA_0.9.16"
}
