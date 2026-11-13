#!/usr/bin/env bash
set -euxo pipefail
root=/home/hanyu/flash_fr3/flash_sim_reproduction_20261028
cd "$root/FLASH-Policy"
export PATH="$root/.venv/bin:$PATH" PYTHONPATH="$PWD" PYTHONUNBUFFERED=1 HYDRA_FULL_ERROR=1
export CUDA_VISIBLE_DEVICES=GPU-f16a68c8-8e24-9ac3-0713-b9bdbc3af6f6
export MUJOCO_GL=egl MUJOCO_EGL_DEVICE_ID=3 PYOPENGL_PLATFORM=egl
export OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8 WANDB_MODE=disabled
python - "$root/results/training-process-exit.json" <<'PY'
import json,sys
assert json.load(open(sys.argv[1]))['exit_code']==0
PY
python "$root/scripts/verify_training.py" --root "$root" 2>&1 | tee "$root/logs/verify-training.log"
task=libero_90.kitchen_scene1_put_the_black_bowl_on_the_plate
run=flash_bowl_s42_v1_20261028
test -s "il_outputs/flash/$task/$run/checkpoints/10000.ckpt"
cp -a "il_outputs/flash/$task/$run/.hydra" "$root/configs/train-hydra"
test ! -e "$root/logs/eval-formal-10000.log"
nvidia-smi --id="$CUDA_VISIBLE_DEVICES" --query-gpu=timestamp,index,uuid,memory.used,utilization.gpu,power.draw --format=csv -l 5 > "$root/logs/gpu-evaluation.csv" &
monitor_pid=$!
finish() {
  rc=$?
  kill "$monitor_pid" 2>/dev/null || true
  wait "$monitor_pid" 2>/dev/null || true
  printf '{"ended_utc":"%s","exit_code":%s}\n' "$(date -u '+%Y-%m-%dT%H:%M:%SZ')" "$rc" > "$root/results/evaluation-process-exit.json"
  exit "$rc"
}
trap finish EXIT
bash -e roboverse_learn/il/il_run.sh --task_name_set "$task" --sim_set mujoco \
  --demo_num 40 --policy_name flash --downsample_ratio 4 --exp_name "$run" \
  --gpu 0 --num_steps 15000 --checkpoint_every_n_steps 1250 --eval_ckpt_name 10000 --dr_level_eval 0 \
  --override train_config.training_params.seed=42 --override logging.mode=disabled \
  --train_enable False --eval_enable True 2>&1 | tee "$root/logs/eval-formal-10000.log"
cp -a "il_outputs/flash/$task/$run/.hydra" "$root/configs/eval-hydra"
python "$root/scripts/summarize_eval.py" --root "$root" 2>&1 | tee "$root/logs/summarize-evaluation.log"
