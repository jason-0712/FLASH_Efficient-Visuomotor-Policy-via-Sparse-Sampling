#!/usr/bin/env bash
set -euxo pipefail
root=/home/hanyu/flash_fr3/flash_sim_reproduction_20261028
cd "$root/FLASH-Policy"
export PATH="$root/.venv/bin:$PATH" PYTHONPATH="$PWD" PYTHONUNBUFFERED=1
export CUDA_VISIBLE_DEVICES=GPU-f16a68c8-8e24-9ac3-0713-b9bdbc3af6f6
export OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8
task=libero_90.kitchen_scene1_put_the_black_bowl_on_the_plate
dataset="data_policy/${task}FrankaL0_obs:joint_pos_act:joint_pos_ds4_40.zarr"
test ! -e "$dataset"
python -u roboverse_learn/il/data2zarr_dp.py --task_name "${task}FrankaL0_obs:joint_pos_act:joint_pos" \
  --expert_data_num 40 --metadata_dir "$root/data-formal/success" \
  --observation_space joint_pos --action_space joint_pos --downsample_ratio 4 \
  2>&1 | tee "$root/logs/convert-formal.log"
python "$root/scripts/validate_demos.py" --root "$root/data-formal" --expected 40 --zarr "$dataset" --output "$root/manifests/formal-zarr.json"
python "$root/scripts/build_provenance.py" --root "$root"
diag="il_outputs/flash/$task/poly_diag_ylim.json"
if [ -f "$diag" ]; then mv "$diag" "$root/configs/smoke-poly-diag-ylim.json"; fi
/home/hanyu/miniconda3/envs/flash_fr3/bin/python -m pip freeze > "$root/environment/base-flash_fr3-after.txt"
cmp "$root/environment/base-flash_fr3-before.txt" "$root/environment/base-flash_fr3-after.txt"
git rev-parse HEAD > "$root/environment/repository-commit.txt"
git status --short > "$root/environment/repository-status.txt"
git diff > "$root/environment/source-changes.patch"
