#!/usr/bin/env bash
set -euxo pipefail
root=/home/hanyu/flash_fr3/flash_sim_reproduction_20261028
out="$root/comparison_20261028"
cd "$root/FLASH-Policy"
export PATH="$root/.venv/bin:$PATH" PYTHONPATH="$PWD" PYTHONUNBUFFERED=1
export CUDA_VISIBLE_DEVICES=GPU-ccf45489-f640-c333-5e47-fefd51eed0ec
export MUJOCO_GL=egl MUJOCO_EGL_DEVICE_ID=1 PYOPENGL_PLATFORM=egl
export OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8
finish() {
  rc=$?
  printf '{"ended_utc":"%s","exit_code":%s}\n' "$(date -u '+%Y-%m-%dT%H:%M:%SZ')" "$rc" > "$out/results/stability-flash-exit.json"
  exit "$rc"
}
trap finish EXIT
python -c 'import json,sys; assert json.load(open(sys.argv[1]))["prefix_exactly_equal"]' "$out/results/stability-smoke-validation.json"
python "$out/scripts/stable_eval.py" \
  --checkpoint "$root/FLASH-Policy/il_outputs/flash/libero_90.kitchen_scene1_put_the_black_bowl_on_the_plate/flash_bowl_s42_v1_20261028/checkpoints/10000.ckpt" \
  --output "$out/stability-flash-10000" --rollouts 50 \
  --protocol "$out/configs/stable-placement-protocol.json" \
  2>&1 | tee "$out/logs/stability-flash-10000.log"
