#!/usr/bin/env bash
set -euo pipefail
root=/home/hanyu/flash_fr3/flash_sim_reproduction_20261028
out="$root/heldout_curve_20261029"
cd "$out"
export PATH="$root/.venv/bin:$PATH" PYTHONDONTWRITEBYTECODE=1 MPLBACKEND=Agg
export OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4
# CPU-only: never initialize policy inference or CUDA.
while ! test -f results/job-gpu2-2500-exit.json; do sleep 10; done
python - <<'PY'
import json
assert json.load(open('results/job-gpu2-2500-exit.json'))['exit_code']==0
PY
python scripts/analyze_checkpoint.py --root "$out" --frozen-root "$root/heldout_initials_20261029" --step 2500 > logs/analyze-2500.log 2>&1
python scripts/aggregate_curve.py --root "$out" --reference-root "$root/heldout_eval_20261029" > logs/aggregate.log 2>&1
python scripts/audit_supplemental.py --root "$out" > logs/audit-supplemental.log 2>&1
python scripts/verify_videos.py --root "$out" > logs/video-validation.log 2>&1
python scripts/plot_curve.py --root "$out" > logs/plot-curve.log 2>&1
python scripts/build_report.py --root "$out" > logs/build-report.log 2>&1
python - <<'PY'
import datetime,json
from pathlib import Path
Path('results/cpu-analysis-complete.json').write_text(json.dumps({'status':'PASS','finished_utc':datetime.datetime.now(datetime.timezone.utc).isoformat()},indent=2)+'\n')
PY
printf 'CPU_CURVE_ANALYSIS_COMPLETE\n'
