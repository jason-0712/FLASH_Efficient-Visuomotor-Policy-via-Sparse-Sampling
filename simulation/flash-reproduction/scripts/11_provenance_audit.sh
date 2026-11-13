#!/usr/bin/env bash
set -euxo pipefail
root=/home/hanyu/flash_fr3/flash_sim_reproduction_20261028
export FLASH_REPRO_ROOT="$root"
cd "$root/FLASH-Policy"
"$root/.venv/bin/python" -u - <<'PY' 2>&1 | tee "$root/logs/provenance-audit.log"
import json, hashlib, pickle
from pathlib import Path
from huggingface_hub import HfApi
import os
root=Path(os.environ['FLASH_REPRO_ROOT'])
repo=root/'FLASH-Policy'
old=Path('/home/hanyu/flash_fr3/flash_fr3_reproduction')
old_record={}
for rel in ['runs/train_15k_seed42_20261022_160102/config.json','data/pickplace_74eps_stride4/manifest.json','runs/train_15k_seed42_20261022_160102/offline_eval_20261022_175327/report.json']:
    p=old/rel
    if p.is_file(): old_record[rel]=json.loads(p.read_text())
(root/'environment/existing-fr3-evidence.json').write_text(json.dumps(old_record,indent=2,ensure_ascii=False))
print('EXISTING_FR3_CONFIG', old_record.get('runs/train_15k_seed42_20261022_160102/config.json'))
traj=repo/'roboverse_data/trajs/libero90/libero_90_kitchen_scene1_put_the_black_bowl_on_the_plate_traj_v2.pkl'
if traj.exists():
    x=pickle.loads(traj.read_bytes())
    print('REFERENCE_TOP_TYPE',type(x).__name__)
    if isinstance(x,dict):
        print('REFERENCE_KEYS',list(x))
        for k,v in x.items():
            print('REFERENCE_VALUE',k,type(v).__name__, len(v) if hasattr(v,'__len__') else None)
            if isinstance(v,list) and v: print('FIRST_ITEM_KEYS',list(v[0]) if isinstance(v[0],dict) else type(v[0]).__name__)
    print('REFERENCE_SHA256',hashlib.sha256(traj.read_bytes()).hexdigest(),traj.stat().st_size)
info=HfApi().dataset_info('RoboVerseOrg/roboverse_data')
print('HF_DATASET_HEAD',info.sha)
(root/'manifests/hf-dataset-head.json').write_text(json.dumps({'repo':'RoboVerseOrg/roboverse_data','observed_head':info.sha},indent=2))
PY
