#!/usr/bin/env bash
set -euxo pipefail
root=/home/hanyu/flash_fr3/flash_sim_reproduction_20261028
out="$root/comparison_20261028"
mkdir -p "$out"/{scripts,logs,configs,manifests,results,videos}
cd "$root/FLASH-Policy"
export PATH="$root/.venv/bin:$PATH" PYTHONPATH="$PWD" PYTHONUNBUFFERED=1 HYDRA_FULL_ERROR=1
export CUDA_VISIBLE_DEVICES=GPU-f16a68c8-8e24-9ac3-0713-b9bdbc3af6f6
export MUJOCO_GL=egl MUJOCO_EGL_DEVICE_ID=3 PYOPENGL_PLATFORM=egl
export OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8 WANDB_MODE=disabled
task=libero_90.kitchen_scene1_put_the_black_bowl_on_the_plate
run=fm_dit_bowl_s42_v1_20261028
data="data_policy/${task}FrankaL0_obs:joint_pos_act:joint_pos_ds1_40.zarr"
test ! -e "$data"
test ! -e "il_outputs/fm_dit/$task/$run"
python roboverse_learn/il/data2zarr_dp.py --task_name "${task}FrankaL0_obs:joint_pos_act:joint_pos" \
  --expert_data_num 40 --metadata_dir "$root/data-formal/success" \
  --observation_space joint_pos --action_space joint_pos --downsample_ratio 1 \
  2>&1 | tee "$out/logs/convert-ds1.log"
python "$out/scripts/validate_demos.py" --root "$root/data-formal" --expected 40 --stride 1 \
  --zarr "$data" --output "$out/manifests/ds1-validation.json"
python - "$root" "$out" <<'PY'
import json,sys,numpy as np
from pathlib import Path
from roboverse_learn.il.utils.sampler import get_val_mask
root,out=map(Path,sys.argv[1:])
old=json.loads((root/'manifests/formal-zarr-image-audit.json').read_text())
new=json.loads((out/'manifests/ds1-validation.json').read_text())
for a,b in zip(old['demonstrations'],new['demonstrations'],strict=True):
    for k in ['demo_id','source_reference_index','frames','metadata_sha256','rgb_sha256']: assert a[k]==b[k],(k,a,b)
ids=[r['source_reference_index'] for r in new['demonstrations']]
mask=get_val_mask(40,.02,42)
assert [ids[i] for i in np.flatnonzero(mask)]==[49]
(out/'manifests/aligned-dataset-split.json').write_text(json.dumps({'source_ids':ids,'optimization_ids':[ids[i] for i in np.flatnonzero(~mask)],'validation_ids':[49],'seed':42,'raw_source_hashes_equal':True},indent=2))
PY
printf '{"started_utc":"%s","requested_steps":15000,"seed":42,"batch_size":32}\n' "$(date -u '+%Y-%m-%dT%H:%M:%SZ')" > "$out/results/fm-training-start.json"
nvidia-smi --id="$CUDA_VISIBLE_DEVICES" --query-gpu=timestamp,index,uuid,memory.used,utilization.gpu --format=csv -l 5 > "$out/logs/gpu-fm-training.csv" &
monitor_pid=$!
finish() {
  rc=$?
  kill "$monitor_pid" 2>/dev/null || true
  wait "$monitor_pid" 2>/dev/null || true
  printf '{"ended_utc":"%s","exit_code":%s}\n' "$(date -u '+%Y-%m-%dT%H:%M:%SZ')" "$rc" > "$out/results/fm-training-exit.json"
  exit "$rc"
}
trap finish EXIT
bash -e roboverse_learn/il/il_run.sh --task_name_set "$task" --sim_set mujoco \
  --demo_num 40 --policy_name fm_dit --downsample_ratio 1 --exp_name "$run" \
  --gpu 0 --num_steps 15000 --checkpoint_every_n_steps 1250 \
  --override train_config.training_params.seed=42 --override logging.mode=disabled \
  --train_enable True --eval_enable False 2>&1 | tee "$out/logs/train-fm.log"
cp -a "il_outputs/fm_dit/$task/$run/.hydra" "$out/configs/fm-train-hydra"
test -s "il_outputs/fm_dit/$task/$run/checkpoints/10000.ckpt"
