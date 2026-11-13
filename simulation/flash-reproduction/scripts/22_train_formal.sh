#!/usr/bin/env bash
set -euxo pipefail
root=/home/hanyu/flash_fr3/flash_sim_reproduction_20261028
cd "$root/FLASH-Policy"
export PATH="$root/.venv/bin:$PATH" PYTHONPATH="$PWD" PYTHONUNBUFFERED=1 HYDRA_FULL_ERROR=1
export CUDA_VISIBLE_DEVICES=GPU-f16a68c8-8e24-9ac3-0713-b9bdbc3af6f6
export MUJOCO_GL=egl MUJOCO_EGL_DEVICE_ID=3 PYOPENGL_PLATFORM=egl
export OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8 WANDB_MODE=disabled
task=libero_90.kitchen_scene1_put_the_black_bowl_on_the_plate
run=flash_bowl_s42_v1_20261028
test -s "$root/manifests/formal-zarr.json"
test ! -e "il_outputs/flash/$task/$run"
nvidia-smi --query-gpu=index,uuid,memory.free,utilization.gpu --format=csv > "$root/environment/gpu-before-training.csv"
printf '{"started_utc":"%s","requested_steps":15000,"seed":42,"batch_size":32,"run_id":"%s"}\n' "$(date -u '+%Y-%m-%dT%H:%M:%SZ')" "$run" > "$root/results/training-start.json"
nvidia-smi --id="$CUDA_VISIBLE_DEVICES" --query-gpu=timestamp,index,uuid,memory.used,utilization.gpu,power.draw --format=csv -l 5 > "$root/logs/gpu-training.csv" &
monitor_pid=$!
finish() {
  rc=$?
  kill "$monitor_pid" 2>/dev/null || true
  wait "$monitor_pid" 2>/dev/null || true
  printf '{"ended_utc":"%s","exit_code":%s}\n' "$(date -u '+%Y-%m-%dT%H:%M:%SZ')" "$rc" > "$root/results/training-process-exit.json"
  exit "$rc"
}
trap finish EXIT
bash -e roboverse_learn/il/il_run.sh --task_name_set "$task" --sim_set mujoco \
  --demo_num 40 --policy_name flash --downsample_ratio 4 --exp_name "$run" \
  --gpu 0 --num_steps 15000 --checkpoint_every_n_steps 1250 \
  --override train_config.training_params.seed=42 --override logging.mode=disabled \
  --train_enable True --eval_enable False 2>&1 | tee "$root/logs/train-formal.log"
test -s "il_outputs/flash/$task/$run/checkpoints/10000.ckpt"
