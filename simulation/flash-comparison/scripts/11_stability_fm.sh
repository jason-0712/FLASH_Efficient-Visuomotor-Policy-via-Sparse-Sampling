#!/usr/bin/env bash
set -euxo pipefail
root=/home/hanyu/flash_fr3/flash_sim_reproduction_20261028
out="$root/comparison_20261028"
while test ! -e "$out/results/stability-flash-exit.json"; do sleep 20; done
"$root/.venv/bin/python" -c 'import json,sys; assert json.load(open(sys.argv[1]))["exit_code"]==0' "$out/results/stability-flash-exit.json"
cd "$root/FLASH-Policy"
export PATH="$root/.venv/bin:$PATH" PYTHONPATH="$PWD" PYTHONUNBUFFERED=1
export CUDA_VISIBLE_DEVICES=GPU-ccf45489-f640-c333-5e47-fefd51eed0ec
export MUJOCO_GL=egl MUJOCO_EGL_DEVICE_ID=1 PYOPENGL_PLATFORM=egl
export OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8
finish() {
  rc=$?
  printf '{"ended_utc":"%s","exit_code":%s}\n' "$(date -u '+%Y-%m-%dT%H:%M:%SZ')" "$rc" > "$out/results/stability-fm-exit.json"
  exit "$rc"
}
trap finish EXIT
checkpoint="$root/FLASH-Policy/il_outputs/fm_dit/libero_90.kitchen_scene1_put_the_black_bowl_on_the_plate/fm_dit_bowl_s42_v1_20261028/checkpoints/10000.ckpt"
test -s "$checkpoint"
python "$out/scripts/stable_eval.py" --checkpoint "$checkpoint" --output "$out/stability-smoke-fm" --rollouts 1 \
  --protocol "$out/configs/stable-placement-protocol.json" 2>&1 | tee "$out/logs/stability-smoke-fm.log"
python "$out/scripts/stable_eval.py" --checkpoint "$checkpoint" --output "$out/stability-smoke-control-fm" --rollouts 1 --control-only \
  --protocol "$out/configs/stable-placement-protocol.json" 2>&1 | tee "$out/logs/stability-smoke-control-fm.log"
python "$out/scripts/validate_fm_stability_smoke.py" "$out" 2>&1 | tee "$out/logs/validate-fm-stability-smoke.log"
python "$out/scripts/stable_eval.py" --checkpoint "$checkpoint" --output "$out/stability-fm_dit-10000" --rollouts 50 \
  --protocol "$out/configs/stable-placement-protocol.json" 2>&1 | tee "$out/logs/stability-fm_dit-10000.log"
