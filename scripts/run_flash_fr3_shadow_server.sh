#!/usr/bin/env bash
set -euo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
flash_root="${FLASH_ROOT:-$task_root/../FLASH-Policy}"
CUDA_VISIBLE_DEVICES="${FLASH_GPU:-1}" python -u "$task_root/scripts/serve_flash_fr3_shadow.py" \
  --flash-root "$flash_root" \
  --dataset-cache "$task_root/data/pickplace_74eps_stride4" \
  --checkpoint "$task_root/runs/train_15k_seed42_20261022_160102/best.pt" \
  --warmup-observation "$task_root/observations/flash_fr3_live_observation_20261022_01/observation.npz" \
  --bind 192.168.1.113 --port 10131 --device cuda:0
