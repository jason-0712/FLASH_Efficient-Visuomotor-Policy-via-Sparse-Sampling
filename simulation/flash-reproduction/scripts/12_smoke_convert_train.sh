#!/usr/bin/env bash
set -euxo pipefail
root=/home/hanyu/flash_fr3/flash_sim_reproduction_20261028
cd "$root/FLASH-Policy"
export PATH="$root/.venv/bin:$PATH" PYTHONPATH="$PWD" PYTHONUNBUFFERED=1 HYDRA_FULL_ERROR=1
export CUDA_VISIBLE_DEVICES=GPU-f16a68c8-8e24-9ac3-0713-b9bdbc3af6f6
export MUJOCO_GL=egl MUJOCO_EGL_DEVICE_ID=3 PYOPENGL_PLATFORM=egl
export OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8 WANDB_MODE=disabled
task=libero_90.kitchen_scene1_put_the_black_bowl_on_the_plate
dataset="data_policy/${task}FrankaL0_obs:joint_pos_act:joint_pos_ds4_2.zarr"
python "$root/scripts/validate_demos.py" --root "$root/data-smoke" --expected 2 --output "$root/manifests/smoke-raw.json"
test ! -e "$dataset"
python -u roboverse_learn/il/data2zarr_dp.py --task_name "${task}FrankaL0_obs:joint_pos_act:joint_pos" \
  --expert_data_num 2 --metadata_dir "$root/data-smoke/success" \
  --observation_space joint_pos --action_space joint_pos --downsample_ratio 4 \
  2>&1 | tee "$root/logs/convert-smoke.log"
python "$root/scripts/validate_demos.py" --root "$root/data-smoke" --expected 2 --zarr "$dataset" --output "$root/manifests/smoke-zarr.json"
test ! -e "il_outputs/flash/$task/flash_bowl_smoke_s42_20261028"
bash -e roboverse_learn/il/il_run.sh --task_name_set "$task" --sim_set mujoco \
  --demo_num 2 --policy_name flash --downsample_ratio 4 --exp_name flash_bowl_smoke_s42_20261028 \
  --gpu 0 --num_steps 20 --checkpoint_every_n_steps 10 \
  --override train_config.training_params.seed=42 --override logging.mode=disabled \
  --train_enable True --eval_enable False 2>&1 | tee "$root/logs/train-smoke.log"
