"""Generate frozen unseen object configurations without executing any policy."""
import argparse,copy,hashlib,json,os,platform,subprocess,time
from pathlib import Path
import numpy as np
import torch
import mujoco
from cpu_scene import Scene


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def state_hash(state):return hashlib.sha256(json.dumps(state,sort_keys=True,allow_nan=False).encode()).hexdigest()
def xy(state):return np.array([state[n]['pos'][k] for n in ['akita_black_bowl','plate'] for k in [0,1]])


def evaluate_physical(arrays,p):
    reasons=[];metrics={};a=p['acceptance'];tail=p['tail_samples_for_stability']
    if not all(np.isfinite(v).all() for v in arrays.values()):reasons.append('nonfinite_array')
    if bool(arrays['geometric_success'].any()):reasons.append('already_satisfies_goal')
    metrics['minimum_object_contact_distance_m']=float(arrays['minimum_object_contact_distance'].min())
    if metrics['minimum_object_contact_distance_m'] < -a['object_contact_penetration_max_m']:
        reasons.append('object_penetration_exceeds_limit')
    for name in ['akita_black_bowl','plate']:
        q=arrays[name+'_pose'][:,:3];v=arrays[name+'_linear_velocity'][-tail:];w=arrays[name+'_angular_velocity'][-tail:]
        m={'initial_support_contacts':int(arrays[name+'_support'][0]),
           'tail_min_support_contacts':int(arrays[name+'_support'][-tail:].min()),
           'max_xy_drift_m':float(np.linalg.norm(q[:,:2]-q[0,:2],axis=1).max()),
           'max_xyz_drift_m':float(np.linalg.norm(q-q[0],axis=1).max()),
           'tail_max_linear_speed_m_s':float(np.linalg.norm(v,axis=1).max()),
           'tail_max_angular_speed_rad_s':float(np.linalg.norm(w,axis=1).max()),
           'tail_z_range_m':float(np.ptp(q[-tail:,2])),
           'max_robot_contacts':int(arrays[name+'_robot'].max()),
           'max_cabinet_contacts':int(arrays[name+'_cabinet'].max())}
        metrics[name]=m
        limits=[('initial_support_contacts',a['initial_support_contacts_per_object_min'],'min'),
                ('tail_min_support_contacts',a['tail_support_contacts_per_object_min'],'min'),
                ('max_xy_drift_m',a['object_xy_drift_max_m'],'max'),('max_xyz_drift_m',a['object_xyz_drift_max_m'],'max'),
                ('tail_max_linear_speed_m_s',a['tail_linear_speed_max_m_s'],'max'),
                ('tail_max_angular_speed_rad_s',a['tail_angular_speed_max_rad_s'],'max'),
                ('tail_z_range_m',a['tail_z_range_max_m'],'max'),
                ('max_robot_contacts',a['forbidden_object_contacts_max'],'max'),
                ('max_cabinet_contacts',a['forbidden_object_contacts_max'],'max')]
        for key,bound,kind in limits:
            if (m[key] < bound if kind=='min' else m[key] > bound):reasons.append(name+':'+key)
    metrics['max_bowl_plate_contacts']=int(arrays['bowl_plate'].max())
    if metrics['max_bowl_plate_contacts']>a['forbidden_object_contacts_max']:reasons.append('bowl_plate_contact')
    return reasons,metrics


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--protocol',required=True);ap.add_argument('--split',required=True);ap.add_argument('--output',required=True);a=ap.parse_args()
    out=Path(a.output);out.mkdir(parents=True,exist_ok=False)
    (out/'candidate-traces').mkdir();(out/'negative-controls').mkdir()
    p=json.loads(Path(a.protocol).read_text());split=json.loads(Path(a.split).read_text())
    assert digest(a.split)==p['source_reference_split_sha256']
    base=copy.deepcopy(next(r['initial_state'] for r in split['initial_states'] if r['reference_index']==p['canonical_base_reference_id']))
    original_xy=np.array([xy(r['initial_state']) for r in split['initial_states']]);original_hashes={state_hash(r['initial_state']) for r in split['initial_states']}
    for r in split['initial_states']:assert state_hash(r['initial_state'])==r['initial_state_sha256']
    scene=Scene()
    env_info={'start_utc':time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),'pid':os.getpid(),'hostname':platform.node(),
              'python':platform.python_version(),'numpy':np.__version__,'mujoco':mujoco.__version__,'torch':torch.__version__,
              'CUDA_VISIBLE_DEVICES':os.environ['CUDA_VISIBLE_DEVICES'],'MUJOCO_GL':os.environ['MUJOCO_GL'],
              'torch_cuda_available':torch.cuda.is_available(),'policy_or_checkpoint_loaded':False,
              'source_commit':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
              'source_diff_sha256':hashlib.sha256(subprocess.check_output(['git','diff','--binary'])).hexdigest(),
              'script_sha256':digest(__file__),'cpu_scene_sha256':digest(Path(__file__).with_name('cpu_scene.py')),
              'generation_protocol_sha256':digest(a.protocol),'source_split_sha256':digest(a.split),
              'geom_map':scene.names,'support_geometry_ids':sorted(scene.support),
              'physics_dt_s':float(scene.m.opt.timestep),'decimation':scene.handler.decimation}
    (out/'environment.json').write_text(json.dumps(env_info,indent=2))
    def run(state):
        obs=scene.reset(state);samples=[scene.measure(obs)]
        for _ in range(p['passive_check_control_steps']):samples.append(scene.measure(scene.step()))
        arrays={k:np.asarray([r[k] for r in samples]) for k in samples[0]}
        arrays['time_s']=np.arange(len(samples))*.015
        return arrays
    # Controls validate the frozen filters, and never enter the candidate stream.
    controls=[]
    for name in p['negative_controls']:
        state=copy.deepcopy(base)
        if name=='already_at_goal':
            state['akita_black_bowl']['pos']=copy.deepcopy(state['plate']['pos']);state['akita_black_bowl']['pos'][2]+=.01
        elif name=='bowl_floating_10cm':state['akita_black_bowl']['pos'][2]+=.1
        elif name=='bowl_interpenetrating_floor_2cm':state['akita_black_bowl']['pos'][2]-=.02
        arrays=run(state);reasons,metrics=evaluate_physical(arrays,p)
        dist=float(np.linalg.norm(original_xy-xy(state),axis=1).min())
        if dist < p['minimum_distance_to_all_original_50_m']:reasons.append('too_close_to_original_object_configuration')
        np.savez_compressed(out/'negative-controls'/f'{name}.npz',**arrays)
        record={'name':name,'input_state':state,'rejection_reasons':reasons,'metrics':metrics}
        controls.append(record)
        assert reasons,('negative control accepted',name)
        print(json.dumps({'negative_control':name,'rejected':True,'reasons':reasons}),flush=True)
    (out/'negative-controls.json').write_text(json.dumps(controls,indent=2))
    rng=np.random.Generator(np.random.PCG64(p['seed']))
    accepted=[];seen=set();all_candidates=[]
    for candidate_id in range(p['maximum_candidates']):
        state=copy.deepcopy(base)
        for name in ['akita_black_bowl','plate']:
            for k,axis in enumerate(['x','y']):state[name]['pos'][k]=float(rng.uniform(*p['coordinate_bounds_m'][name][axis]))
        h=state_hash(state);distances=np.linalg.norm(original_xy-xy(state),axis=1);nearest=int(np.argmin(distances));dist=float(distances[nearest])
        reasons=[];metrics=None
        if h in original_hashes:reasons.append('exact_original_state')
        if h in seen:reasons.append('duplicate_test_state')
        if dist < p['minimum_distance_to_all_original_50_m']:reasons.append('too_close_to_original_object_configuration')
        trace=None
        if not reasons:
            arrays=run(state);reasons,metrics=evaluate_physical(arrays,p)
            trace=f'candidate-traces/{candidate_id:04d}.npz';np.savez_compressed(out/trace,**arrays)
        record={'candidate_id':candidate_id,'accepted':not reasons,'state_sha256':h,'initial_state':state,
                'nearest_original_reference_id':split['initial_states'][nearest]['reference_index'],
                'nearest_original_xy4_distance_m':dist,'rejection_reasons':reasons,'physical_metrics':metrics,'passive_trace':trace}
        all_candidates.append(record)
        with (out/'all-candidates.jsonl').open('a') as f:f.write(json.dumps(record)+'\n')
        if not reasons:
            test_id=len(accepted);record['test_id']=test_id;accepted.append(record);seen.add(h)
            print(json.dumps({'test_id':test_id,'candidate_id':candidate_id,'nearest_original_xy4_distance_m':dist}),flush=True)
        if len(accepted)==p['target_count']:break
    assert len(accepted)==p['target_count'],f'Only {len(accepted)} legal states; protocol is not auto-relaxed'
    dataset={'schema_version':1,'protocol_id':p['protocol_id'],'generation_protocol_sha256':digest(a.protocol),
             'source_reference_split_sha256':digest(a.split),'task':p['task'],'count':len(accepted),
             'state_format':'Flat reference-v2 state dictionary. Load with cpu_scene.nested for env.reset.',
             'intended_use':'Evaluation only; never convert into a training Zarr or fit a normalizer on these states.',
             'states':[{'test_id':r['test_id'],'candidate_id':r['candidate_id'],'state_sha256':r['state_sha256'],
                        'initial_state':r['initial_state']} for r in accepted]}
    (out/'test-initials.json').write_text(json.dumps(dataset,indent=2))
    (out/'selected-candidates.json').write_text(json.dumps(accepted,indent=2))
    summary={'candidate_count':len(all_candidates),'selected_count':len(accepted),'rejected_count':len(all_candidates)-len(accepted),
             'negative_controls_rejected':len(controls),'selected_state_hashes_unique':len(seen)==len(accepted),
             'minimum_selected_distance_to_original_50_m':min(r['nearest_original_xy4_distance_m'] for r in accepted),
             'test_initials_sha256':digest(out/'test-initials.json'),
             'known_robot_state_note':p['known_inherited_robot_issue']}
    (out/'summary.json').write_text(json.dumps(summary,indent=2))
    scene.close();env_info['end_utc']=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime());env_info['torch_cuda_initialized']=torch.cuda.is_initialized()
    assert not env_info['torch_cuda_initialized']
    (out/'environment.json').write_text(json.dumps(env_info,indent=2))
    print(json.dumps(summary,indent=2));print('CPU_HELDOUT_GENERATION_COMPLETE',flush=True)


if __name__=='__main__':main()
