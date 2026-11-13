#!/usr/bin/env bash
# Run on cps-server. Output directories must not already exist.
set -euo pipefail
root=/home/hanyu/flash_fr3/flash_sim_reproduction_20261028
stage="$root/placement_audit_20261028"
mode=${1:-formal}
case "$mode" in
  formal) plan="$stage/configs/audit-plan.json"; output="$stage/formal" ;;
  smoke-fixed) plan="$stage/configs/smoke-plan.json"; output="$stage/smoke-fixed" ;;
  *) exit 2 ;;
esac
cd "$root/FLASH-Policy"
export CUDA_VISIBLE_DEVICES="" MUJOCO_GL=disable PYTHONPATH="$PWD" PYTHONUNBUFFERED=1
export OMP_NUM_THREADS=2 MKL_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2
timeout --signal=TERM --kill-after=10s 15m "$root/.venv/bin/python" -u \
  "$stage/scripts/cpu_reference_audit.py" --root "$root" --output "$output" --plan "$plan" \
  2>&1 | tee "$stage/logs/$mode.log"
