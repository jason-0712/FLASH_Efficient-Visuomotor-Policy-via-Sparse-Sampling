"""CPU-only MuJoCo scene helpers; rendering and policy loading prohibited."""
import copy
import os
assert os.environ.get('CUDA_VISIBLE_DEVICES') == ''
assert os.environ.get('MUJOCO_GL') == 'disable'
import mujoco
import numpy as np
import torch
from metasim.task.registry import get_task_class
from metasim.scenario.cameras import PinholeCameraCfg


class NoRenderer:
    def __init__(self,*a,**kw):pass
    def close(self):pass
    def update_scene(self,*a,**kw):raise RuntimeError('No rendering in CPU scene preparation')
    def render(self,*a,**kw):raise RuntimeError('No rendering in CPU scene preparation')


def nested(flat):
    s=copy.deepcopy(flat)
    for obj in s.values():
        for key in ['pos','rot']:
            obj[key]=torch.tensor(obj[key],dtype=torch.float64)
        if 'dof_pos' in obj:
            obj['dof_pos']={k:float(np.asarray(v).reshape(-1)[0]) for k,v in obj['dof_pos'].items()}
    return {'robots':{'franka':s.pop('franka')},'objects':s}


class Scene:
    def __init__(self):
        assert not torch.cuda.is_available()
        torch.set_num_threads(2)
        mujoco.Renderer=NoRenderer
        cls=get_task_class('libero_90.kitchen_scene1_put_the_black_bowl_on_the_plate')
        camera=PinholeCameraCfg(name='camera0',data_types=[],width=256,height=256,pos=(2.,0.,2.),look_at=(0.,0.,0.))
        self.env=cls(cls.scenario.update(robots=['franka'],simulator='mujoco',renderer='mujoco',
                      cameras=[camera],num_envs=1,headless=True),device='cpu')
        self.handler=self.env.handler;self.physics=self.handler.physics
        self.m,self.d=self.physics.model.ptr,self.physics.data.ptr
        self.joints=self.handler._get_joint_names('franka',sort=True)
        self.body={n:self.physics.model.body(f'{self.handler.mj_objects[n].model}/').id for n in ['akita_black_bowl','plate']}
        self.groups={}
        self.names=[]
        for i in range(self.m.ngeom):
            body=self.physics.model.body(int(self.m.geom_bodyid[i])).name
            name=self.physics.model.geom(i).name
            self.names.append({'id':i,'geom':name,'body':body,'type':int(self.m.geom_type[i]),
                               'contype':int(self.m.geom_contype[i]),'conaffinity':int(self.m.geom_conaffinity[i])})
            for n in ['akita_black_bowl','plate','franka','wooden_cabinet']:
                prefix=self.handler.mj_objects[n].model+'/'
                if body.startswith(prefix):self.groups.setdefault(n,set()).add(i)
        self.support=set(range(self.m.ngeom))-set().union(*self.groups.values())
        assert abs(self.m.opt.timestep*self.handler.decimation-.015)<1e-12
        self.target=None

    def reset(self,flat):
        mujoco.mj_resetData(self.m,self.d)
        self.handler._current_vel_target=None
        state=nested(flat)
        self.target={'franka':{'dof_pos_target':copy.deepcopy(state['robots']['franka']['dof_pos'])}}
        obs,_=self.env.reset(states=[state])
        return obs

    def step(self):
        return self.env.step([self.target])[0]

    def measure(self,obs):
        row={}
        for n,body in self.body.items():
            v=np.zeros(6)
            mujoco.mj_objectVelocity(self.m,self.d,mujoco.mjtObj.mjOBJ_BODY,body,v,0)
            row[n+'_pose']=obs.objects[n].root_state[0,:7].numpy().copy()
            row[n+'_linear_velocity']=v[3:].copy()
            row[n+'_angular_velocity']=v[:3].copy()
        counts={n+'_support':0 for n in self.body}
        counts.update({n+'_robot':0 for n in self.body})
        counts.update({n+'_cabinet':0 for n in self.body})
        counts['bowl_plate']=0
        min_dist=0.
        for c in self.d.contact[:self.d.ncon]:
            a,b=int(c.geom1),int(c.geom2)
            relevant=any(a in self.groups[n] or b in self.groups[n] for n in self.body)
            if relevant:min_dist=min(min_dist,float(c.dist))
            for n in self.body:
                pair=lambda group:(a in self.groups[n] and b in group) or (b in self.groups[n] and a in group)
                if pair(self.support):counts[n+'_support']+=1
                if pair(self.groups['franka']):counts[n+'_robot']+=1
                if pair(self.groups['wooden_cabinet']):counts[n+'_cabinet']+=1
            if (a in self.groups['akita_black_bowl'] and b in self.groups['plate']) or (b in self.groups['akita_black_bowl'] and a in self.groups['plate']):counts['bowl_plate']+=1
        row.update(counts)
        row['minimum_object_contact_distance']=min_dist
        row['geometric_success']=bool(self.env._terminated(obs)[0])
        row['joint_positions']=obs.robots['franka'].joint_pos[0].numpy().copy()
        row['native_qpos']=self.d.qpos.copy();row['native_qvel']=self.d.qvel.copy()
        return row

    def contact_list(self):
        return [{'a':self.names[int(c.geom1)],'b':self.names[int(c.geom2)],'distance':float(c.dist)} for c in self.d.contact[:self.d.ncon]]

    def close(self):
        self.env.close()
        assert not torch.cuda.is_initialized()
