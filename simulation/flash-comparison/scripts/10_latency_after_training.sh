#!/usr/bin/env bash
set -euxo pipefail
root=/home/hanyu/flash_fr3/flash_sim_reproduction_20261028
out="$root/comparison_20261028"
while test ! -e "$out/results/fm-training-exit.json"; do sleep 20; done
"$root/.venv/bin/python" -c 'import json,sys; assert json.load(open(sys.argv[1]))["exit_code"]==0' "$out/results/fm-training-exit.json"
cd "$root/FLASH-Policy"
export PATH="$root/.venv/bin:$PATH" PYTHONPATH="$PWD" PYTHONUNBUFFERED=1
export CUDA_VISIBLE_DEVICES=GPU-f16a68c8-8e24-9ac3-0713-b9bdbc3af6f6
export OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8
finish() {
  rc=$?
  printf '{"ended_utc":"%s","exit_code":%s}\n' "$(date -u '+%Y-%m-%dT%H:%M:%SZ')" "$rc" > "$out/results/latency-exit.json"
  exit "$rc"
}
trap finish EXIT
nvidia-smi --id="$CUDA_VISIBLE_DEVICES" --query-gpu=timestamp,index,uuid,memory.used,utilization.gpu --format=csv > "$out/results/latency-gpu-before.csv"
python "$out/scripts/latency_benchmark.py" --root "$root" 2>&1 | tee "$out/logs/latency.log"
