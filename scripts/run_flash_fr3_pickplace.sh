#!/usr/bin/env bash
# Compatibility name for the single formal continuous FLASH task entry.
set -euo pipefail
FLASH_TASK_MODE="${1:---check}"
if [[ $# -gt 0 ]]; then shift; fi
case "$FLASH_TASK_MODE" in --check) FLASH_TASK_MODE=--prepare;; --execute) ;; *) exit 2;; esac
FLASH_TASK_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec bash "$FLASH_TASK_ROOT/scripts/run_flash_fr3_raw_policy.sh" "$FLASH_TASK_MODE" "$@"
