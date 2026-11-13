#!/usr/bin/env bash
# A separate port/GPU leaves the existing 10131 service and its tasks alone.
set -euo pipefail
FLASH_HISTORY_PACKAGE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
FLASH_HISTORY_REPRO="${FLASH_REPRODUCTION_ROOT:-$HOME/flash_fr3/flash_fr3_reproduction}"
FLASH_HISTORY_SOURCE="${FLASH_ROOT:-$HOME/flash_fr3/FLASH-Policy}"
test -f "$FLASH_HISTORY_REPRO/scripts/infer_flash_fr3_observation.py"
export PYTHONPATH="$FLASH_HISTORY_PACKAGE/scripts:$FLASH_HISTORY_REPRO/scripts${PYTHONPATH:+:$PYTHONPATH}"
CUDA_VISIBLE_DEVICES="${FLASH_GPU:-1}" python -u "$FLASH_HISTORY_PACKAGE/scripts/serve_flash_fr3_shadow.py" \
  --flash-root "$FLASH_HISTORY_SOURCE" \
  --dataset-cache "$FLASH_HISTORY_REPRO/data/pickplace_74eps_stride4" \
  --checkpoint "$FLASH_HISTORY_REPRO/runs/train_15k_seed42_20261022_160102/best.pt" \
  --warmup-observation "$FLASH_HISTORY_PACKAGE/observations/retimed_initial_observation.npz" \
  --bind 192.168.1.113 --port 10132 --device cuda:0 --allow-retimed-observations
