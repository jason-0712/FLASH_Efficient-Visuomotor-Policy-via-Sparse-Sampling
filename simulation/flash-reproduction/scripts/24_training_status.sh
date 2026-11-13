#!/usr/bin/env bash
set -u
root=/home/hanyu/flash_fr3/flash_sim_reproduction_20261028
date -Is
tmux list-sessions 2>/dev/null || true
nvidia-smi --id=GPU-f16a68c8-8e24-9ac3-0713-b9bdbc3af6f6 --query-gpu=index,uuid,memory.used,utilization.gpu --format=csv
"$root/.venv/bin/python" - "$root" <<'PY'
import json,sys,math
from pathlib import Path
root=Path(sys.argv[1])
run=root/'FLASH-Policy/il_outputs/flash/libero_90.kitchen_scene1_put_the_black_bowl_on_the_plate/flash_bowl_s42_v1_20261028'
f=run/'logs.json.txt'
if f.exists():
    rows=[]
    for l in f.read_text().splitlines():
        try: rows.append(json.loads(l))
        except json.JSONDecodeError: pass
    if rows:
        last=rows[-1]
        report={'logged_updates':len(rows),'last_global_step':last['global_step'],'last_loss':last['train_loss'],
                'last_lr':last['lr'],'elapsed_seconds':last.get('elapsed_seconds'),
                'all_losses_finite':all(math.isfinite(r['train_loss']) for r in rows)}
        if len(rows)>=1001:
            start=rows[-501]
            duration=last['elapsed_seconds']-start['elapsed_seconds']
            report.update({'latest_500_steps_seconds':duration,'estimated_remaining_seconds':max(0,15000-last['global_step']-1)*duration/500})
        print(json.dumps(report,indent=2))
        (root/'results/current-training-progress.json').write_text(json.dumps(report,indent=2))
    else: print('NO_TRAINING_UPDATES_YET')
else: print('NO_TRAINING_JSON_LOG_YET')
print('CHECKPOINTS',[(p.name,p.stat().st_size) for p in sorted((run/'checkpoints').glob('*.ckpt'))])
receipt=root/'results/training-process-exit.json'
if receipt.exists(): print('PROCESS_EXIT',receipt.read_text())
PY
tail -c 2000 "$root/logs/train-formal.log"
