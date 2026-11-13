#!/usr/bin/env bash
set -euxo pipefail
root=/home/hanyu/flash_fr3/flash_sim_reproduction_20261028
"$root/.venv/bin/python" - "$root" <<'PY'
import csv,json,sys,numpy as np
from pathlib import Path
root=Path(sys.argv[1]);out=root/'comparison_20261028';rows=[]
for d in sorted((root/'data-formal/success').glob('demo_*')):
    m=json.loads((d/'metadata.json').read_text());q=np.asarray(m['joint_qpos']);a=np.asarray(m['joint_qpos_target'])
    rows.append({'source_reference_index':int(d.name.split('_')[1]),'frames':len(q),
                 'final_actual_finger1_m':q[-1,0],'final_actual_finger2_m':q[-1,1],
                 'final_target_finger1_m':a[-1,0],'final_target_finger2_m':a[-1,1],
                 'both_actual_fingers_ge_03m_at_end':bool((q[-1,:2]>=.03).all()),
                 'both_target_fingers_ge_03m_at_end':bool((a[-1,:2]>=.03).all()),
                 'both_actual_open_samples_last20':int((q[-20:,:2]>=.03).all(axis=1).sum()),
                 'both_target_open_samples_last20':int((a[-20:,:2]>=.03).all(axis=1).sum())})
with (out/'results/demo-release-audit.csv').open('w',newline='') as f:
    w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
summary={'demonstrations':len(rows),'actual_both_fingers_open_at_end':sum(r['both_actual_fingers_ge_03m_at_end'] for r in rows),
         'target_both_fingers_open_at_end':sum(r['both_target_fingers_ge_03m_at_end'] for r in rows),
         'interpretation':'Finger opening audit only. Raw metadata does not establish the full release/contact/velocity/2-second metric. Demonstrations are collected around first geometric success; lack of opening at their end limits what the learned policy is being supervised to complete.'}
(out/'results/demo-release-audit.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
PY
