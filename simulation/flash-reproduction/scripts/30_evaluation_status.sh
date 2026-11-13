#!/usr/bin/env bash
set -uo pipefail
root=/home/hanyu/flash_fr3/flash_sim_reproduction_20261028
date -Is
tmux list-sessions 2>/dev/null || true
"$root/.venv/bin/python" - "$root" <<'PY'
import json,sys
from pathlib import Path
root=Path(sys.argv[1])
for rel in ['results/training-process-exit.json','results/training-summary.json','results/evaluation-process-exit.json','results/evaluation-summary.json']:
    p=root/rel
    if p.exists(): print(rel,p.read_text())
run=root/'FLASH-Policy/il_outputs/flash/libero_90.kitchen_scene1_put_the_black_bowl_on_the_plate/flash_bowl_s42_v1_20261028'
files=list((run/'eval').rglob('[0-9][0-9][0-9][0-9].txt'))
k=sum('True' in p.read_text().split('SuccessOnce: ',1)[1].splitlines()[0] for p in files)
print('EVALUATION_RECORDS_COMPLETED',len(files),'SUCCESSES_SO_FAR',k)
PY
if [[ -f "$root/logs/eval-formal-10000.log" ]]; then
  tail -c 2200 "$root/logs/eval-formal-10000.log"
else
  tail -c 1600 "$root/logs/eval-formal-job.log"
fi
