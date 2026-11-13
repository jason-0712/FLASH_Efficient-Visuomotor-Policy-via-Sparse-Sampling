import argparse
import hashlib
import json
import pickle
from pathlib import Path

import numpy as np
from roboverse_learn.il.utils.sampler import get_val_mask


def sha256(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):
            h.update(b)
    return h.hexdigest()


def plain(x):
    if isinstance(x,dict):
        return {str(k):plain(v) for k,v in x.items()}
    if isinstance(x,(list,tuple)):
        return [plain(v) for v in x]
    if hasattr(x,'tolist'):
        return x.tolist()
    return x


p=argparse.ArgumentParser()
p.add_argument('--root',required=True)
a=p.parse_args()
root=Path(a.root)
data=root/'FLASH-Policy/roboverse_data'
traj=data/'trajs/libero90/libero_90_kitchen_scene1_put_the_black_bowl_on_the_plate_traj_v2.pkl'
refs=pickle.loads(traj.read_bytes())['franka']
raw=json.loads((root/'manifests/formal-raw.json').read_text())
ids=[v['source_reference_index'] for v in raw['demonstrations']]
assert len(ids)==40 and len(set(ids))==40
val_mask=get_val_mask(len(ids),0.02,42)
train=[v for i,v in enumerate(ids) if not val_mask[i]]
val=[v for i,v in enumerate(ids) if val_mask[i]]
initial=[]
for i,r in enumerate(refs):
    init=plain(r['init_state'])
    h=hashlib.sha256(json.dumps(init,sort_keys=True).encode()).hexdigest()
    initial.append({'reference_index':i,'initial_state':init,'initial_state_sha256':h,
                    'used_for_optimization':i in train,'used_for_validation':i in val,
                    'collected_for_dataset':i in ids,'reference_action_steps':len(r['actions'])})
split={'reference_file':str(traj),'reference_sha256':sha256(traj),'reference_count':len(refs),
       'collected_dataset_reference_ids':ids,'optimization_reference_ids':train,'validation_reference_ids':val,
       'dataset_split_seed':42,'dataset_val_ratio':0.02,'evaluation_reference_ids':list(range(len(refs))),
       'evaluation_ids_not_in_collected_dataset':[i for i in range(len(refs)) if i not in ids],
       'exact_unique_initial_state_hashes':len(set(v['initial_state_sha256'] for v in initial)),
       'protocol_note':'Official 50-rollout pool overlaps the collected demonstration initial states. This is not a 50-scene held-out test. Report the uncollected subset separately.',
       'initial_states':initial}
(root/'manifests/reference-split.json').write_text(json.dumps(split,indent=2,ensure_ascii=False))
assets=[]
for f in sorted(data.rglob('*')):
    if not f.is_file() or '.cache' in f.parts or f.name=='download.lock':
        continue
    meta=data/'.cache/huggingface/download'/f.relative_to(data).parent/(f.name+'.metadata')
    item={'path':str(f.relative_to(data)),'bytes':f.stat().st_size,'sha256':sha256(f)}
    if meta.is_file():
        item['hf_local_download_metadata']=meta.read_text()
    assets.append(item)
(root/'manifests/asset-files.json').write_text(json.dumps(assets,indent=2))
print(json.dumps({'dataset_ids':ids,'optimization_count':len(train),'validation_ids':val,
                  'evaluation_count':len(refs),'uncollected_evaluation_ids':split['evaluation_ids_not_in_collected_dataset'],
                  'exact_unique_initial_states':split['exact_unique_initial_state_hashes'],
                  'asset_file_count':len(assets)},indent=2))
