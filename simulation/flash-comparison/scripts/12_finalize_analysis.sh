#!/usr/bin/env bash
set -euxo pipefail
root=/home/hanyu/flash_fr3/flash_sim_reproduction_20261028
out="$root/comparison_20261028"
cd "$root/FLASH-Policy"
export PATH="$root/.venv/bin:$PATH" PYTHONPATH="$PWD" PYTHONUNBUFFERED=1
export OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8
for job in fm-training official-queue stability-flash stability-fm latency; do
  while test ! -e "$out/results/$job-exit.json"; do sleep 20; done
  python -c 'import json,sys; assert json.load(open(sys.argv[1]))["exit_code"]==0' "$out/results/$job-exit.json"
done
finish() {
  rc=$?
  printf '{"ended_utc":"%s","exit_code":%s}\n' "$(date -u '+%Y-%m-%dT%H:%M:%SZ')" "$rc" > "$out/results/final-analysis-exit.json"
  exit "$rc"
}
trap finish EXIT
python "$out/scripts/audit_fm_training.py" "$root" 2>&1 | tee "$out/logs/audit-fm-training.log"
python "$out/scripts/analyze_comparison.py" --root "$root" 2>&1 | tee "$out/logs/analyze-comparison.log"
git rev-parse HEAD > "$out/configs/source-commit.txt"
git status --short > "$out/configs/source-status.txt"
git diff --binary > "$out/configs/current-source.patch"
nvidia-smi --query-gpu=timestamp,index,uuid,memory.total,memory.used,utilization.gpu --format=csv > "$out/results/final-gpu-snapshot.csv"
