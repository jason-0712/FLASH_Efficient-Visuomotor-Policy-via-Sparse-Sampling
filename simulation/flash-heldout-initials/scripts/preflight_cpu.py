import argparse,json,os,time
from pathlib import Path
import numpy as np
from cpu_scene import Scene

p=argparse.ArgumentParser();p.add_argument('--split',required=True);p.add_argument('--output',required=True);a=p.parse_args()
out=Path(a.output);out.mkdir(parents=True,exist_ok=False)
source=json.loads(Path(a.split).read_text())
base=next(r for r in source['initial_states'] if r['reference_index']==min(source['optimization_reference_ids']))
scene=Scene();obs=scene.reset(base['initial_state']);rows=[scene.measure(obs)]
initial_contacts=scene.contact_list()
for i in range(200):
    obs=scene.step();rows.append(scene.measure(obs))
arrays={k:np.asarray([r[k] for r in rows]) for k in rows[0]}
np.savez_compressed(out/'baseline.npz',**arrays)
report={'utc':time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),'base_reference_id':base['reference_index'],
        'CUDA_VISIBLE_DEVICES':os.environ['CUDA_VISIBLE_DEVICES'],'geometries':scene.names,
        'support_geometry_ids':sorted(scene.support),'initial_contacts':initial_contacts,'final_contacts':scene.contact_list()}
report['summary']={}
for key,v in arrays.items():
    if v.ndim==1:report['summary'][key]={'initial':float(v[0]),'final':float(v[-1]),'min':float(v.min()),'max':float(v.max())}
for n in scene.body:
    report['summary'][n]={'initial_position':arrays[n+'_pose'][0,:3].tolist(),'final_position':arrays[n+'_pose'][-1,:3].tolist(),
        'max_linear_speed_last_50':float(np.linalg.norm(arrays[n+'_linear_velocity'][-50:],axis=1).max()),
        'max_angular_speed_last_50':float(np.linalg.norm(arrays[n+'_angular_velocity'][-50:],axis=1).max())}
report['joints']={name:{'initial':float(arrays['joint_positions'][0,i]),'final':float(arrays['joint_positions'][-1,i])} for i,name in enumerate(scene.joints)}
(out/'baseline.json').write_text(json.dumps(report,indent=2))
scene.close();print(json.dumps({k:v for k,v in report.items() if k not in ['geometries','initial_contacts','final_contacts']},indent=2))
print('CPU_PREFLIGHT_COMPLETE')
