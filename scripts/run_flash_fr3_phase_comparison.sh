#!/usr/bin/env bash
set -euo pipefail
FLASH_PHASE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$FLASH_PHASE_ROOT"
sha256sum --check SHA256SUMS
exec python -u scripts/run_flash_fr3_phase_comparison.py "$@"
