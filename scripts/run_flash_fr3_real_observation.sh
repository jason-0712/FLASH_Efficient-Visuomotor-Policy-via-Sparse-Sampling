#!/usr/bin/env bash
set -euo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
flash_root="${FLASH_ROOT:-$task_root/../FLASH-Policy}"
checkpoint="$task_root/runs/train_15k_seed42_20261022_160102/best.pt"
observation="$task_root/observations/flash_fr3_live_observation_20261022_01/observation.npz"
CUDA_VISIBLE_DEVICES="${FLASH_GPU:-1}" python -u "$task_root/scripts/infer_flash_fr3_observation.py" \
  --flash-root "$flash_root" \
  --dataset-cache "$task_root/data/pickplace_74eps_stride4" \
  --checkpoint "$checkpoint" \
  --observation "$observation" \
  --output "$task_root/runs/real_observation_$(date +%Y%m%d_%H%M%S)_$$" \
  --device cuda:0
