#!/usr/bin/env bash
set -euxo pipefail
root=/home/hanyu/flash_fr3/flash_sim_reproduction_20261028
cd "$root/FLASH-Policy"
export PYTHONPATH="$PWD" OMP_NUM_THREADS=2 MKL_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2
"$root/.venv/bin/python" - "$root" <<'PY'
import hashlib, json, sys
from pathlib import Path
import numpy as np
import metasim, roboverse_learn
root=Path(sys.argv[1])
dataset=root/'FLASH-Policy/data_policy/libero_90.kitchen_scene1_put_the_black_bowl_on_the_plateFrankaL0_obs:joint_pos_act:joint_pos_ds4_40.zarr'
records=[]
for p in sorted(dataset.rglob('*')):
    if not p.is_file(): continue
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(4*1024*1024),b''): h.update(b)
    records.append({'path':str(p.relative_to(dataset)),'bytes':p.stat().st_size,'sha256':h.hexdigest()})
out={'dataset_path':str(dataset),'file_count':len(records),'total_bytes':sum(r['bytes'] for r in records),'files':records}
(root/'manifests/dataset-files.json').write_text(json.dumps(out,indent=2))
smoke=root/'FLASH-Policy/il_outputs/flash/libero_90.kitchen_scene1_put_the_black_bowl_on_the_plate/flash_bowl_smoke_b4_s42_20261028/eval'
diagnostics=list(smoke.rglob('diag_data_demo0.npz'))
assert len(diagnostics)==1
with np.load(diagnostics[0]) as d:
    schema={k:{'shape':list(d[k].shape),'dtype':str(d[k].dtype)} for k in d.files}
    assert all(d[k].shape==(300,9) for k in ['pos_desired','pos_actual','vel_desired','vel_actual'])
    assert np.isclose(float(d['control_dt']),.015)
runtime={'executable':sys.executable,'prefix':sys.prefix,'base_prefix':sys.base_prefix,
         'metasim_file':metasim.__file__,'roboverse_learn_file':roboverse_learn.__file__,
         'sys_path':sys.path,'pyvenv_cfg':(root/'.venv/pyvenv.cfg').read_text(),
         'smoke_diagnostic_schema':schema}
(root/'environment/effective-import-paths.json').write_text(json.dumps(runtime,indent=2))
print(json.dumps({'dataset_files':len(records),'dataset_bytes':out['total_bytes'],'runtime':runtime},indent=2))
PY
git diff --check
git diff > "$root/environment/source-changes.patch"
git status --porcelain=v1 > "$root/environment/source-status.txt"
git rev-parse HEAD > "$root/environment/checkout-commit.txt"
df -h "$root" > "$root/environment/disk-after-data.txt"
