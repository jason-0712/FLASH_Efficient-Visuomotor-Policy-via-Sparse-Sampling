#!/usr/bin/env bash
set -uo pipefail
root=/home/hanyu/flash_fr3/flash_sim_reproduction_20261028
out="$root/comparison_20261028"
date -Is
tmux list-sessions 2>/dev/null || true
"$root/.venv/bin/python" - "$root" "$out" <<'PY'
import json,math,sys
from pathlib import Path
root,out=map(Path,sys.argv[1:])
task='libero_90.kitchen_scene1_put_the_black_bowl_on_the_plate'
run=root/f'FLASH-Policy/il_outputs/fm_dit/{task}/fm_dit_bowl_s42_v1_20261028'
f=run/'logs.json.txt'
if f.exists():
    rows=[]
    for s in f.read_text().splitlines():
        try: rows.append(json.loads(s))
        except json.JSONDecodeError: pass
    if rows:
        x={'updates':len(rows),'last_loss':rows[-1]['train_loss'],'all_finite':all(math.isfinite(r['train_loss']) for r in rows),'elapsed_seconds':rows[-1]['elapsed_seconds']}
        if len(rows)>501:
            span=rows[-1]['elapsed_seconds']-rows[-501]['elapsed_seconds']
            x.update(last_500_seconds=span,estimated_remaining_seconds=max(0,15000-len(rows))*span/500)
        print('FM_TRAINING',json.dumps(x))
        (out/'results/current-fm-training-progress.json').write_text(json.dumps(x,indent=2))
for step in [2500,5000,7500]:
    ds=list((root/f'FLASH-Policy/il_outputs/flash/{task}/flash_bowl_s42_v1_20261028/eval').rglob(f'{step}_*.ckpt_*'))
    for d in ds:
        files=list(d.glob('[0-9][0-9][0-9][0-9].txt'))
        k=sum('True' in f.read_text().split('SuccessOnce: ',1)[1].splitlines()[0] for f in files)
        print('FLASH_EVAL',step,'completed',len(files),'successes',k)
for f in sorted((out/'results').glob('*exit.json')): print(f.name,f.read_text())
for d in sorted((out/'official-runs').glob('*')):
    for e in (d/'eval').rglob('*.ckpt_*'):
        records=list(e.glob('[0-9][0-9][0-9][0-9].txt'))
        k=sum('True' in f.read_text().split('SuccessOnce: ',1)[1].splitlines()[0] for f in records)
        print('STANDARD_EVAL',d.name,'completed',len(records),'successes',k)
for name in ['stability-flash-10000','stability-fm_dit-10000']:
    f=out/name/'rollouts.jsonl'
    if f.exists():
        rows=[json.loads(s) for s in f.read_text().splitlines()]
        print('STABILITY',name,'completed',len(rows),'geometric',sum(r['geometric_success_once'] for r in rows),'stable',sum(r['stable_placement_success'] for r in rows))
PY
nvidia-smi --id=2,3 --query-gpu=index,uuid,memory.used,utilization.gpu --format=csv
