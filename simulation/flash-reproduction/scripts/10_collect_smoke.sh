#!/usr/bin/env bash
set -euxo pipefail
root=/home/hanyu/flash_fr3/flash_sim_reproduction_20261028
cd "$root/FLASH-Policy"
export PATH="$root/.venv/bin:$PATH" PYTHONPATH="$PWD" PYTHONUNBUFFERED=1
export CUDA_VISIBLE_DEVICES=GPU-f16a68c8-8e24-9ac3-0713-b9bdbc3af6f6
export MUJOCO_GL=egl MUJOCO_EGL_DEVICE_ID=3 PYOPENGL_PLATFORM=egl
export OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8
test ! -e "$root/data-smoke"
timeout --signal=TERM --kill-after=30s 15m python -u scripts/advanced/collect_demo.py \
  --task libero_90.kitchen_scene1_put_the_black_bowl_on_the_plate \
  --sim mujoco --robot franka --num_envs 1 --demo_start_idx 0 --num_demo_success 2 \
  --cust_name smoke_20261028 --custom_save_dir "$root/data-smoke" \
  --level 0 --no-run-all --run_unfinished --headless \
  2>&1 | tee "$root/logs/collect-smoke-01.log"
