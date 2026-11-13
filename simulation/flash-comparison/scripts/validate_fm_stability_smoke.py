import json,sys
from pathlib import Path
import numpy as np
out=Path(sys.argv[1])
control=next((out/'stability-smoke-control-fm/official-style/eval').rglob('10000_*.ckpt_*'))
extended=next((out/'stability-smoke-fm/official-style/eval').rglob('10000_*.ckpt_*'))
row=json.loads((out/'stability-smoke-fm/rollouts.jsonl').read_text())
errors={}
with np.load(control/'vel_diag/npz_data/diag_data_demo0.npz') as a,np.load(extended/'vel_diag/npz_data/diag_data_demo0.npz') as b:
    n=len(a['pos_desired'])
    assert n==(row['first_geometric_success_step'] or 300)
    for k in ['pos_desired','pos_actual']:
        errors[k]=float(np.max(np.abs(a[k]-b[k][:n])))
        np.testing.assert_array_equal(a[k],b[k][:n])
with np.load(out/'stability-smoke-fm/episodes/0000.npz') as d:
    xy=np.linalg.norm(d['bowl_pose'][:,:2]-d['plate_pose'][:,:2],axis=1)
    height=d['bowl_pose'][:,2]-d['plate_pose'][:,2]
    np.testing.assert_array_equal(d['geometric_success'],(xy<.06)&(height>0)&(height<.03))
result={'prefix_exactly_equal':True,'control_steps':n,'prefix_max_errors':errors,'geometric_position_formula_matches':True,'smoke_result':row}
(out/'results/stability-fm-smoke-validation.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))
