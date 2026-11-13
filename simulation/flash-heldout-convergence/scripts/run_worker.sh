#!/usr/bin/env bash
set -euo pipefail
root=/home/hanyu/flash_fr3/flash_sim_reproduction_20261028
out="$root/heldout_curve_20261029"
frozen="$root/heldout_initials_20261029"
gpu="$1"
case "$gpu" in
  0) gpu_uuid=GPU-7425c8dc-71ce-0ff9-760d-a5bcb025a8ab; steps=(2500);;
  2) gpu_uuid=GPU-eb1c016b-9505-66b6-0362-57dea67de4eb; steps=(5000 7500);;
  *) exit 2;;
esac
cd "$root/FLASH-Policy"
export PATH="$root/.venv/bin:$PATH" PYTHONPATH="$PWD" PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1
export CUDA_VISIBLE_DEVICES="$gpu_uuid" MUJOCO_GL=egl MUJOCO_EGL_DEVICE_ID="$gpu" PYOPENGL_PLATFORM=egl
export OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8 WANDB_MODE=disabled MPLBACKEND=Agg
exec 9>"$out/gpu${gpu}.lock"
flock -n 9
start_epoch=$(date +%s)
monitor_pid=""
finish() {
  rc=$?
  if test -n "$monitor_pid"; then kill "$monitor_pid" 2>/dev/null || true; wait "$monitor_pid" 2>/dev/null || true; fi
  end_epoch=$(date +%s)
  printf '{"ended_utc":"%s","exit_code":%s,"job_wall_seconds":%s,"physical_gpu_index":%s}\n' "$(date -u '+%Y-%m-%dT%H:%M:%SZ')" "$rc" "$((end_epoch-start_epoch))" "$gpu" > "$out/results/job-gpu${gpu}-exit.json"
  nvidia-smi --query-compute-apps=gpu_uuid,pid,process_name,used_gpu_memory --format=csv > "$out/environment/processes-gpu${gpu}-after.csv" || true
  exit "$rc"
}
trap finish EXIT
printf '{"started_utc":"%s","physical_gpu_index":%s,"uuid":"%s"}\n' "$(date -u '+%Y-%m-%dT%H:%M:%SZ')" "$gpu" "$gpu_uuid" > "$out/results/job-gpu${gpu}-start.json"
nvidia-smi --query-gpu=index,uuid,name,memory.total,memory.used,utilization.gpu --format=csv > "$out/environment/gpu${gpu}-before.csv"
nvidia-smi --query-compute-apps=gpu_uuid,pid,process_name,used_gpu_memory --format=csv > "$out/environment/processes-gpu${gpu}-before.csv"
check_gpu_free() {
  python - <<'PY'
import os,subprocess
rows=subprocess.check_output(['nvidia-smi','--query-compute-apps=gpu_uuid,pid','--format=csv,noheader'],text=True).splitlines()
busy=[row for row in rows if row.split(',')[0].strip()==os.environ['CUDA_VISIBLE_DEVICES']]
assert not busy, ('Selected GPU has another process; stopping without modifying it',busy)
PY
}
check_gpu_free
nvidia-smi --id="$gpu" --query-gpu=timestamp,index,uuid,memory.used,utilization.gpu --format=csv -l 5 > "$out/logs/gpu${gpu}-telemetry.csv" &
monitor_pid=$!
if test "$gpu" = 0; then
  for policy in flash fm_dit; do
    check_gpu_free
    python "$out/scripts/evaluate_heldout.py" --policy "$policy" --step 10000 --physical-gpu "$gpu" --rollouts 3 --output "$out/bridge/10000/$policy" --frozen-root "$frozen" --plan "$out/configs/frozen-evaluation-plan.json" > "$out/logs/bridge10000-$policy-gpu0.log" 2>&1
  done
  python "$out/scripts/validate_bridge.py" --root "$out" --reference-root "$root/heldout_eval_20261029" > "$out/logs/bridge-validation.log" 2>&1
fi
for step in "${steps[@]}"; do
  for policy in flash fm_dit; do
    check_gpu_free
    python "$out/scripts/evaluate_heldout.py" --policy "$policy" --step "$step" --physical-gpu "$gpu" --rollouts 3 --output "$out/smoke/$step/$policy" --frozen-root "$frozen" --plan "$out/configs/frozen-evaluation-plan.json" > "$out/logs/smoke-$step-$policy-gpu${gpu}.log" 2>&1
  done
  python "$out/scripts/validate_smoke.py" --root "$out" --step "$step" > "$out/logs/smoke-${step}-validation.log" 2>&1
  for policy in flash fm_dit; do
    check_gpu_free
    python "$out/scripts/evaluate_heldout.py" --policy "$policy" --step "$step" --physical-gpu "$gpu" --rollouts 50 --output "$out/runs/$step/$policy" --frozen-root "$frozen" --plan "$out/configs/frozen-evaluation-plan.json" > "$out/logs/eval-$step-$policy-gpu${gpu}.log" 2>&1
  done
done
printf 'HELDOUT_CURVE_GPU%s_COMPLETE\n' "$gpu"
