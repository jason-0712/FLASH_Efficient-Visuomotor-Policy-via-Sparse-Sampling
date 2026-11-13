#!/usr/bin/env bash
set -euxo pipefail
root=/home/hanyu/flash_fr3/flash_sim_reproduction_20261028
out="$root/comparison_20261028"
cd "$root/FLASH-Policy"
export PATH="$root/.venv/bin:$PATH" PYTHONPATH="$PWD" PYTHONUNBUFFERED=1 HYDRA_FULL_ERROR=1
export CUDA_VISIBLE_DEVICES=GPU-eb1c016b-9505-66b6-0362-57dea67de4eb
export MUJOCO_GL=egl MUJOCO_EGL_DEVICE_ID=2 PYOPENGL_PLATFORM=egl
export OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8 WANDB_MODE=disabled
task=libero_90.kitchen_scene1_put_the_black_bowl_on_the_plate
run=flash_bowl_s42_v1_20261028
finish() {
  rc=$?
  printf '{"ended_utc":"%s","exit_code":%s}\n' "$(date -u '+%Y-%m-%dT%H:%M:%SZ')" "$rc" > "$out/results/flash-early-evals-exit.json"
  exit "$rc"
}
trap finish EXIT
for step in 2500 5000 7500; do
  test -s "il_outputs/flash/$task/$run/checkpoints/$step.ckpt"
  test ! -e "$out/logs/eval-flash-$step.log"
  bash -e roboverse_learn/il/il_run.sh --task_name_set "$task" --sim_set mujoco \
    --demo_num 40 --policy_name flash --downsample_ratio 4 --exp_name "$run" \
    --gpu 0 --num_steps 15000 --checkpoint_every_n_steps 1250 --eval_ckpt_name "$step" --dr_level_eval 0 \
    --override train_config.training_params.seed=42 --override logging.mode=disabled \
    --train_enable False --eval_enable True 2>&1 | tee "$out/logs/eval-flash-$step.log"
  cp -a "il_outputs/flash/$task/$run/.hydra" "$out/configs/flash-eval-$step-hydra"
done
