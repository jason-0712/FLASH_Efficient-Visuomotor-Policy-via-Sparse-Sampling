#!/usr/bin/env bash
set -euo pipefail

task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
flash_source="${FLASH_ROOT:-$HOME/flash_fr3/FLASH-Policy}"
flash_cache="${FLASH_CACHE:-$task_root/data/pickplace_74eps_stride4}"
raw_dataset="$task_root/data/lerobot/quest3_franka_dualcam_pickplace_74eps"
if [[ ! -f "$flash_cache/manifest.json" && -z "${FLASH_CACHE:-}" && -f "$task_root/artifacts/flash_fr3/pickplace_74eps_stride4/manifest.json" ]]; then
  flash_cache="$task_root/artifacts/flash_fr3/pickplace_74eps_stride4"
fi
if [[ ! -f "$flash_source/roboverse_learn/il/policies/flash/flash_policy.py" ]]; then
  echo "找不到 FLASH 源码：$flash_source；请用 FLASH_ROOT 指定已有 FLASH-Policy 目录。" >&2
  exit 1
fi
if [[ ! -f "$flash_cache/manifest.json" && ! -f "$raw_dataset/meta/info.json" ]]; then
  echo "找不到缓存或包内原始示教；请完整解压训练包，或用 FLASH_CACHE 指定已准备的缓存目录。" >&2
  exit 1
fi

# Fail before changing packages if this is not the working CUDA training environment.
CUDA_VISIBLE_DEVICES="${FLASH_GPU:-1}" python - <<'PY'
import sys
import torch
assert torch.cuda.is_available(), "请先激活已有的 flash_fr3 CUDA 环境"
assert sys.version_info[:2] == (3, 11), "请使用已验证的 Python 3.11 环境"
assert torch.__version__.split('+')[0] == '2.7.0', "请使用已验证的 PyTorch 2.7.0 环境"
print("Python:", sys.version.split()[0], "PyTorch:", torch.__version__)
print("GPU:", torch.cuda.get_device_name(0))
PY

python -m pip install -r "$task_root/scripts/flash_fr3_requirements.txt"
python "$task_root/scripts/test_flash_fr3_adapter.py"
if [[ ! -f "$flash_cache/manifest.json" ]]; then
  python "$task_root/scripts/prepare_flash_fr3_data.py" \
    --dataset "$raw_dataset" --output "$flash_cache" --stride 4
fi
smoke_output="$task_root/runs/h100_smoke_$(date +%Y%m%d_%H%M%S_%N)"
CUDA_VISIBLE_DEVICES="${FLASH_GPU:-1}" python "$task_root/scripts/train_flash_fr3.py" \
  --flash-root "$flash_source" \
  --dataset-cache "$flash_cache" \
  --output "$smoke_output" \
  --device cuda:0 --batch-size 2 --smoke-test
echo "诊断结果：$smoke_output/smoke_result.json"
