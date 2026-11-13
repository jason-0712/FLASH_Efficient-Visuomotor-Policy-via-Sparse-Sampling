"""Dataset-bounded FLASH task segments. Pure math; no hardware interfaces."""
import numpy as np
from numpy.polynomial import Polynomial,Legendre
from flash_fr3_reference import DelayedJointReference
from flash_fr3_model_plan import evaluate_segment
from flash_fr3_stream_plan import absolute_bound
TASK_LOWER=np.array([-.40,-.86,-.31,-2.57,-.08,1.48,.22])
TASK_UPPER=np.array([.21,.58,.23,-1.83,.11,2.90,1.00])
TASK_LIMITS=np.array([.20,.20,1.5,30.])[:,None]
def make_task_segments(curve, age, start_q, source_prefix=.6, time_scale=4., stop_seconds=.75):
    q = np.asarray(start_q, dtype=float)
    if q.shape != (7,) or not np.isfinite(q).all():
        raise ValueError("起始关节位置异常")
    if not np.isfinite([source_prefix, time_scale, stop_seconds]).all() or not (
            .2 <= source_prefix <= .6 and 4 <= time_scale <= 8 and stop_seconds == .75):
        raise ValueError("任务分段的时间参数异常")
    initial = np.stack([q, np.zeros(7), np.zeros(7)])
    reference = DelayedJointReference(curve, age, initial, blend_seconds=1.2)
    if source_prefix >= reference.blend or source_prefix > reference.remaining_seconds:
        raise ValueError("任务段超过预测或连续过渡的定义范围")
    # Exact polynomial composition, not a sampled waypoint fit. x is affine in
    # u; physical playback duration is source_prefix * time_scale.
    duration = float(curve["polynomial_duration_seconds"])
    x = Polynomial([2*(curve["anchor_s"] + age/duration)-1, 2*source_prefix/duration])
    correction_u = Polynomial([0, source_prefix/reference.blend])
    coefficients = np.zeros((2, 7, 7))
    for joint in range(7):
        raw = Legendre(np.asarray(curve["coefficients"])[:, joint]).convert(kind=Polynomial)(x)
        correction = Polynomial(reference.correction[:, joint])(correction_u)
        relative = raw + correction - q[joint]
        coefficients[0, :len(relative.coef), joint] = relative.coef
    # Numerical cancellation can leave roundoff; require a truly static origin.
    if np.abs(coefficients[0, :3]).max() > 1e-10:
        raise ValueError("轨迹起点未满足静止 C2 条件")
    coefficients[0, :3] = 0
    durations = [source_prefix*time_scale, float(stop_seconds)]
    end = np.asarray([evaluate_segment(coefficients[0], durations[0], durations[0], k)
                      for k in range(3)])
    # Minimum-integrated-squared-jerk stopping position for this quintic family.
    destination = end[0] + .5*stop_seconds*end[1] + stop_seconds**2*end[2]/12
    c0, c1, c2 = end * np.array([1, stop_seconds, stop_seconds**2/2])[:, None]
    delta = destination-c0
    coefficients[1, :6] = np.stack([c0, c1, c2, 10*delta-6*c1-3*c2,
                                   -15*delta+8*c1+3*c2, 6*delta-3*c1-c2])
    # Cross-check independent evaluation against the original Legendre adapter.
    error = 0.
    for u in np.linspace(0, 1, 101):
        for order in range(4):
            actual = evaluate_segment(coefficients[0], durations[0], u*durations[0], order)
            expected = reference.evaluate_with_jerk(u*source_prefix)[order] / time_scale**order
            if order == 0:
                expected = expected-q
            error = max(error, float(np.abs(actual-expected).max()))
    if error > 1e-9:
        raise ValueError("模型多项式转换与独立求值不一致")
    return {"segment_durations": durations, "coefficients": coefficients.tolist(),
            "source_prefix_seconds": source_prefix, "time_scale": time_scale,
            "nominal_stop_seconds": stop_seconds, "conversion_max_abs_error": error,
            "total_motion_seconds": sum(durations), "observation_age_at_plan_seconds": age,
            "expected_start_q": q.tolist()}


def task_bounds(plan):
    from math import comb
    c=np.asarray(plan['coefficients'],float);d=np.asarray(plan['segment_durations'],float)
    q=np.asarray(plan['expected_start_q'],float)
    if c.shape!=(2,7,7) or d.shape!=(2,) or q.shape!=(7,) or not np.isfinite(np.r_[c.ravel(),d,q]).all():
        raise ValueError('任务曲线结构异常')
    if not .8<=d[0]<=4.8+1e-9 or d[1]!=.75:raise ValueError('任务段时长异常')
    initial=np.array([evaluate_segment(c[0],d[0],0,k) for k in range(3)])
    seam=np.array([evaluate_segment(c[0],d[0],d[0],k)-evaluate_segment(c[1],d[1],0,k) for k in range(3)])
    tail=np.array([evaluate_segment(c[1],d[1],d[1],k) for k in (1,2)])
    if abs(initial).max()>1e-10 or abs(seam).max()>1e-9 or abs(tail).max()>1e-9:raise ValueError('任务 C2 或静止边界异常')
    bounds=np.max([[absolute_bound(p,t,k) for k in range(4)] for p,t in zip(c,d)],axis=0)
    v,a=np.zeros((7,7)),np.zeros((7,7));b=.5
    v[1]=b;v[3]=-b;v[4]=.5*b
    a[2]=.5*b*b;a[3]=-2*b*b/3;a[4]=.25*b*b
    brake=np.array([(bounds[0] if k==0 else 0.)+absolute_bound(v,b,k)*bounds[1]+absolute_bound(a,b,k)*bounds[2] for k in range(4)])
    drift=absolute_bound(v,b,0)*bounds[1]+absolute_bound(a,b,0)*bounds[2]
    positions=[]
    for p in c:
        bernstein=np.array([sum(comb(i,k)/comb(6,k)*p[k] for k in range(i+1)) for i in range(7)])
        positions.append([q+bernstein.min(axis=0)-drift,q+bernstein.max(axis=0)+drift])
    lo=np.min(np.array(positions)[:,0],axis=0);hi=np.max(np.array(positions)[:,1],axis=0)
    return {'pass':bool((bounds<TASK_LIMITS-1e-10).all() and (brake<TASK_LIMITS-1e-10).all() and
                       (lo>=TASK_LOWER).all() and (hi<=TASK_UPPER).all()),
            'nominal_bounds':bounds.tolist(),'watchdog_brake_bounds':brake.tolist(),
            'q_lower_with_brake':lo.tolist(),'q_upper_with_brake':hi.tolist()}


def select_task_plan(curve,age,q):
    for source in (.6,.5,.4,.3,.2):
        for scale in (4.,6.,8.):
            plan=make_task_segments(curve,age,q,source_prefix=source,time_scale=scale)
            bounds=task_bounds(plan)
            if bounds['pass']:
                plan['bounds']=bounds
                return plan
    raise ValueError('当前预测没有通过任务曲线及停车界；不裁剪或放宽限值')


class TaskSequence:
    def __init__(self,grasp_q,release_q):
        self.mode='approach';self.grasp_q=np.asarray(grasp_q,float);self.release_q=np.asarray(release_q,float)
        for poses in (self.grasp_q,self.release_q):
            if poses.ndim!=2 or poses.shape[1]!=7 or len(poses)==0 or not np.isfinite(poses).all():
                raise ValueError('缺少示教抓取/释放姿态')
        self.last_stamp=-np.inf;self.pending=None;self.count=0;self.carry_segments=0;self.grasp_position=None
    def observe(self,closure,q,raw_q,stamp):
        q,raw_q=np.asarray(q,float),np.asarray(raw_q,float)
        if not np.isfinite(np.r_[closure,q,raw_q,stamp]).all() or q.shape!=(7,) or raw_q.shape!=(7,):
            raise ValueError('任务观测异常')
        if stamp<=self.last_stamp:raise ValueError('不能重复使用旧图像决定夹爪动作')
        self.last_stamp=stamp
        desired='grasp' if self.mode=='approach' and closure>=.7 else (
            'release' if self.mode=='carrying' and self.carry_segments>=1 and closure<=.3 else None)
        poses=self.grasp_q if desired=='grasp' else self.release_q
        aligned=np.max(abs(q-raw_q))<=.06
        near=desired is not None and np.min(np.max(abs(poses-q),axis=1))<=.12
        if desired is None or not aligned or not near:
            self.pending=None;self.count=0;return 'arm'
        self.count=self.count+1 if self.pending==desired else 1;self.pending=desired
        return desired if self.count>=2 else 'confirm'
    def grasped(self,width,success):
        if self.mode!='approach' or self.pending!='grasp' or self.count<2:
            raise ValueError('未确认抓取决策')
        if not success or not .003<width<.075:raise ValueError('夹爪抓取失败或疑似空抓；停止任务')
        self.mode='carrying';self.pending=None;self.count=0
    def released(self,success):
        if self.mode!='carrying' or self.pending!='release' or self.count<2 or not success:
            raise ValueError('释放失败或时序异常')
        self.mode='released'
