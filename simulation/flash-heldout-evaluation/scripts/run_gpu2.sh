#!/usr/bin/env bash
set -euo pipefail
root=/home/hanyu/flash_fr3/flash_sim_reproduction_20261028
out="$root/heldout_eval_20261029"
frozen="$root/heldout_initials_20261029"
cd "$root/FLASH-Policy"
export PATH="$root/.venv/bin:$PATH" PYTHONPATH="$PWD" PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1
export CUDA_VISIBLE_DEVICES=GPU-eb1c016b-9505-66b6-0362-57dea67de4eb
export MUJOCO_GL=egl MUJOCO_EGL_DEVICE_ID=2 PYOPENGL_PLATFORM=egl
export OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8 WANDB_MODE=disabled MPLBACKEND=Agg
exec 9>"$out/job.lock"
flock -n 9
start_epoch=$(date +%s)
monitor_pid=""
finish() {
  rc=$?
  if test -n "$monitor_pid"; then kill "$monitor_pid" 2>/dev/null || true; wait "$monitor_pid" 2>/dev/null || true; fi
  end_epoch=$(date +%s)
  printf '{"ended_utc":"%s","exit_code":%s,"job_wall_seconds":%s}\n' "$(date -u '+%Y-%m-%dT%H:%M:%SZ')" "$rc" "$((end_epoch-start_epoch))" > "$out/results/job-exit.json"
  exit "$rc"
}
trap finish EXIT
printf '{"started_utc":"%s","physical_gpu_index":2,"uuid":"%s"}\n' "$(date -u '+%Y-%m-%dT%H:%M:%SZ')" "$CUDA_VISIBLE_DEVICES" > "$out/results/job-start.json"
nvidia-smi --query-gpu=index,uuid,name,memory.total,memory.used,utilization.gpu --format=csv > "$out/environment/gpu-before.csv"
nvidia-smi --query-compute-apps=gpu_uuid,pid,process_name,used_gpu_memory --format=csv > "$out/environment/processes-before.csv"
nvidia-smi --id=2 --query-gpu=timestamp,index,uuid,memory.used,utilization.gpu --format=csv -l 5 > "$out/logs/gpu2-telemetry.csv" &
monitor_pid=$!
check_gpu_free() {
  python - <<'PY'
import os,subprocess
rows=subprocess.check_output(['nvidia-smi','--query-compute-apps=gpu_uuid,pid','--format=csv,noheader'],text=True).splitlines()
busy=[row for row in rows if row.split(',')[0].strip()==os.environ['CUDA_VISIBLE_DEVICES']]
assert not busy, ('GPU 2 has another process; stopping without modifying it',busy)
PY
}
for policy in flash fm_dit; do
  check_gpu_free
  python "$out/scripts/evaluate_heldout.py" --policy "$policy" --rollouts 3 --output "$out/smoke/$policy" \
    --frozen-root "$frozen" --plan "$out/configs/frozen-evaluation-plan.json" > "$out/logs/smoke-$policy.log" 2>&1
done
python "$out/scripts/validate_smoke.py" --root "$out" > "$out/logs/smoke-validation.log" 2>&1
for policy in flash fm_dit; do
  check_gpu_free
  python "$out/scripts/evaluate_heldout.py" --policy "$policy" --rollouts 50 --output "$out/runs/$policy" \
    --frozen-root "$frozen" --plan "$out/configs/frozen-evaluation-plan.json" > "$out/logs/eval-$policy.log" 2>&1
done
nvidia-smi --query-gpu=index,uuid,name,memory.total,memory.used,utilization.gpu --format=csv > "$out/environment/gpu-after.csv"
nvidia-smi --query-compute-apps=gpu_uuid,pid,process_name,used_gpu_memory --format=csv > "$out/environment/processes-after.csv"
printf 'HELDOUT_GPU2_JOB_COMPLETE\n'
