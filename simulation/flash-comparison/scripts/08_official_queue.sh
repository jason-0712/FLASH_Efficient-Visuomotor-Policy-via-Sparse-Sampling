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
finish() {
  rc=$?
  printf '{"ended_utc":"%s","exit_code":%s}\n' "$(date -u '+%Y-%m-%dT%H:%M:%SZ')" "$rc" > "$out/results/official-queue-exit.json"
  exit "$rc"
}
trap finish EXIT
while test ! -e "$out/results/flash-early-evals-exit.json"; do sleep 20; done
python -c 'import json,sys; assert json.load(open(sys.argv[1]))["exit_code"]==0' "$out/results/flash-early-evals-exit.json"
for step in 2500 5000 7500; do
  python "$out/scripts/summarize_official.py" --root "$root" --policy flash --step "$step" --log "$out/logs/eval-flash-$step.log"
done
for entry in flash:10000 fm_dit:2500 fm_dit:5000 fm_dit:7500 fm_dit:10000; do
  policy=${entry%:*}
  step=${entry#*:}
  if test "$policy" = flash; then run=flash_bowl_s42_v1_20261028; ds=4; else run=fm_dit_bowl_s42_v1_20261028; ds=1; fi
  if test "$policy" = fm_dit; then
    python - "$root" "$step" "$task" "$run" <<'PY'
import json,sys,time
from pathlib import Path
root,step,task,run=sys.argv[1:];step=int(step)
path=Path(root)/f'FLASH-Policy/il_outputs/fm_dit/{task}/{run}'
while True:
    rows=(path/'logs.json.txt').read_text().splitlines()
    try: ready=json.loads(rows[-1])['global_step']>step+100
    except (KeyError,ValueError): ready=len(rows)>step+100
    if ready and (path/f'checkpoints/{step}.ckpt').is_file(): break
    receipt=Path(root)/'comparison_20261028/results/fm-training-exit.json'
    if receipt.exists():
        assert json.loads(receipt.read_text())['exit_code']==0
        assert (path/f'checkpoints/{step}.ckpt').is_file()
        break
    time.sleep(20)
PY
  fi
  destination="$out/official-runs/$policy-$step"
  test ! -e "$destination"
  test ! -e "$out/logs/eval-standard-$policy-$step.log"
  bash -e roboverse_learn/il/il_run.sh --task_name_set "$task" --sim_set mujoco \
    --demo_num 40 --policy_name "$policy" --downsample_ratio "$ds" --exp_name "$run" \
    --gpu 0 --num_steps 15000 --checkpoint_every_n_steps 1250 --eval_ckpt_name "$step" --dr_level_eval 0 \
    --override train_config.training_params.seed=42 --override logging.mode=disabled \
    --override "hydra.run.dir=$destination" --train_enable False --eval_enable True \
    2>&1 | tee "$out/logs/eval-standard-$policy-$step.log"
  python "$out/scripts/summarize_official.py" --root "$root" --policy "$policy" --step "$step" \
    --eval-root "$destination" --log "$out/logs/eval-standard-$policy-$step.log"
done
