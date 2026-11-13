#!/usr/bin/env python3
"""Full frozen FLASH block + nominal stop, numerical/FK preview only.

This program has no ROS imports, controller clients or robot command outputs.
Training distributions are descriptive; they are not collision-free volumes.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from numpy.polynomial import polynomial

from flash_fr3_kinematics import UrdfKinematics
from flash_fr3_reference import DelayedJointReference
from flash_fr3_shadow_math import validate_reply


def nearest_distances(points, cloud):
    result=[]
    for start in range(0,len(points),128):
        delta=points[start:start+128,None,:]-cloud[None,:,:]
        result.extend(np.sqrt(np.square(delta).sum(axis=2).min(axis=1)).tolist())
    return np.asarray(result)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot",type=Path,required=True)
    parser.add_argument("--cache",type=Path,required=True)
    parser.add_argument("--urdf",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--time-scale",type=float,default=8.)
    args=parser.parse_args()
    if args.output.exists() or not np.isfinite(args.time_scale) or not 1<=args.time_scale<=20:
        parser.error("需使用新目录；只读时间缩放范围为 [1,20]")
    plan=json.loads((args.snapshot/"plan.json").read_text())
    reply=json.loads((args.snapshot/"prediction.json").read_text())
    with np.load(args.snapshot/"observation.npz",allow_pickle=False) as observation:
        metadata=json.loads(str(observation["metadata_json"]))
        images={key:observation[key][-1].copy() for key in ("primary_cam","wrist_cam")}
    anchor=metadata["samples"][-1]["target_stamp_seconds"]
    validate_reply(reply,metadata["request_id"],anchor,15000)
    if plan.get("robot_motion_commanded") is not False or plan.get("checkpoint_step")!=15000:
        raise ValueError("输入必须是明确未执行动作的第15000步模型快照")
    q0=np.asarray(plan["expected_start_q"])
    reference=DelayedJointReference(reply["curve"],plan["observation_age_at_plan_seconds"],
                                    np.stack([q0,np.zeros(7),np.zeros(7)]))
    scale=args.time_scale; duration=reference.remaining_seconds*scale
    times=np.linspace(0,duration,int(np.ceil(duration/.01))+1)
    source=np.minimum(times/scale,reference.remaining_seconds)
    values=np.asarray([reference.evaluate_with_jerk(float(t)) for t in source])
    values/=np.array([1,scale,scale**2,scale**3])[None,:,None]
    # Explicit nominal stop for a reviewable endpoint, rather than extrapolation.
    stop_seconds=.75
    start=values[-1,:3]
    c0,c1,c2=start*np.array([1,stop_seconds,stop_seconds**2/2])[:,None]
    destination=c0+.5*stop_seconds*start[1]+stop_seconds**2*start[2]/12
    delta=destination-c0
    c=np.stack([c0,c1,c2,10*delta-6*c1-3*c2,-15*delta+8*c1+3*c2,6*delta-3*c1-c2])
    stop_times=np.linspace(0,stop_seconds,76)
    stop_values=np.stack([polynomial.polyval(stop_times/stop_seconds,
        polynomial.polyder(c,m=k,axis=0)).T/stop_seconds**k for k in range(4)],axis=1)
    if np.abs(stop_values[0,:3]-start).max()>1e-10 or np.abs(stop_values[-1,1:3]).max()>1e-10:
        raise ValueError("名义停车段 C2 边界未通过")
    # Retain duplicate join timestamp to expose both one-sided jerk values.
    times=np.r_[times,duration+stop_times];values=np.concatenate([values,stop_values])
    manifest=json.loads((args.cache/"manifest.json").read_text())
    states=np.load(args.cache/"state.npy",mmap_mode="r")
    train_episodes=set(manifest["splits"]["train"])
    indices=np.concatenate([np.arange(ep["start"],ep["end"]) for ep in manifest["episodes"]
                            if ep["episode_index"] in train_episodes])
    train_q=np.asarray(states[indices,:7],dtype=float)
    fk=UrdfKinematics(args.urdf)
    check_q=np.concatenate([q0[None,:],values[::max(1,len(values)//12),0],train_q[::max(1,len(train_q)//12)]])
    kdl_error=fk.check_against_kdl(check_q)
    xyz=fk.positions(values[:,0]);train_xyz=fk.positions(train_q);initial_xyz=fk.positions(q0[None,:])[0]
    nearest=nearest_distances(xyz,train_xyz)
    train_lo,train_hi=train_q.min(axis=0),train_q.max(axis=0)
    outside=(values[:,0]<train_lo)|(values[:,0]>train_hi)
    args.output.mkdir(parents=True)
    with (args.output/"reference.csv").open("w",newline="") as stream:
        writer=csv.writer(stream)
        writer.writerow(["playback_time_s"]+[f"{key}{j}" for key in ("q","dq","ddq","jerk") for j in range(1,8)]+["flange_x_m","flange_y_m","flange_z_m"])
        for t,v,p in zip(times,values,xyz):writer.writerow([t,*v.ravel().tolist(),*p.tolist()])
    np.savez_compressed(args.output/"trajectory.npz",time_seconds=times,reference=values,flange_xyz=xyz,
                         training_flange_xyz=train_xyz,initial_flange_xyz=initial_xyz)
    report={"format":"flash_fr3_workspace_preview_v1","status":"READONLY_WORKSPACE_PREVIEW_COMPLETE",
        "robot_motion_commanded":False,"gripper_motion_commanded":False,"execution_eligible":False,
        "snapshot":str(args.snapshot),"checkpoint_step":15000,"weights":"ema_model",
        "source_remaining_seconds":reference.remaining_seconds,"time_scale":scale,
        "total_preview_seconds":float(times[-1]),"nominal_stop_seconds":stop_seconds,
        "sample_count":len(times),"max_playback_sample_dt_seconds":float(np.diff(times).max()),
        "q_min_rad":values[:,0].min(axis=0).tolist(),"q_max_rad":values[:,0].max(axis=0).tolist(),
        "per_joint_excursion_deg":np.rad2deg(np.abs(values[:,0]-q0).max(axis=0)).tolist(),
        "peak_velocity_rad_s":np.abs(values[:,1]).max(axis=0).tolist(),
        "peak_acceleration_rad_s2":np.abs(values[:,2]).max(axis=0).tolist(),
        "peak_jerk_rad_s3":np.abs(values[:,3]).max(axis=0).tolist(),
        "flange_frame":{"base":fk.base,"tip":fk.tip,"meaning":"URDF flange center, not calibrated fingertip TCP"},
        "urdf_sha256":hashlib.sha256(args.urdf.read_bytes()).hexdigest(),"fk_kdl_max_abs_error":kdl_error,
        "flange_start_m":initial_xyz.tolist(),"flange_end_m":xyz[-1].tolist(),
        "flange_min_m":xyz.min(axis=0).tolist(),"flange_max_m":xyz.max(axis=0).tolist(),
        "flange_max_displacement_from_start_m":float(np.linalg.norm(xyz-initial_xyz,axis=1).max()),
        "training_frames":len(train_q),"training_episodes":len(train_episodes),
        "training_flange_min_m":train_xyz.min(axis=0).tolist(),"training_flange_max_m":train_xyz.max(axis=0).tolist(),
        "flange_nearest_training_distance_m":{"median":float(np.median(nearest)),"p95":float(np.quantile(nearest,.95)),"max":float(nearest.max())},
        "joints_outside_training_observed_range_1_based":(np.flatnonzero(outside.any(axis=0))+1).tolist(),
        "training_joint_min_rad":train_lo.tolist(),"training_joint_max_rad":train_hi.tolist(),
        "scope":"Numerical preview of one frozen prediction with C2 start/stop and 8x default playback time. No robot dynamics, online replanning, table/box/self collision, fingertip TCP or hardware braking validation. Training ranges do not establish safety. Sampled values are not certified extrema."}
    (args.output/"report.json").write_text(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False)+"\n")
    fig=plt.figure(figsize=(13,10),layout="constrained")
    ax=fig.add_subplot(221,projection="3d")
    ax.scatter(*train_xyz.T,s=2,c=".75",alpha=.15,label="Training demonstrations (66 episodes)")
    ax.plot(*xyz.T,c="tab:red",linewidth=2,label="Full frozen candidate + stop")
    ax.scatter(*initial_xyz,c="tab:green",s=45,label="Current flange")
    ax.set(xlabel="base X [m]",ylabel="base Y [m]",zlabel="base Z [m]",title="Flange path: fr3_link0 -> fr3_link8")
    ax.legend(fontsize=7)
    for position,(a,b) in zip((222,223),((0,1),(0,2))):
        ax=fig.add_subplot(position);ax.scatter(train_xyz[:,a],train_xyz[:,b],s=2,c=".75",alpha=.2)
        ax.plot(xyz[:,a],xyz[:,b],c="tab:red");ax.scatter(initial_xyz[a],initial_xyz[b],c="tab:green",s=45)
        ax.set(xlabel=f"base {'XYZ'[a]} [m]",ylabel=f"base {'XYZ'[b]} [m]");ax.axis("equal");ax.grid(alpha=.2)
    ax=fig.add_subplot(224)
    for j in range(7):ax.plot(times,np.rad2deg(values[:,0,j]-q0[j]),label=f"joint {j+1}")
    ax.axvline(duration,c=".4",linestyle="--",label="Nominal stop begins")
    ax.set(xlabel="Preview playback time [s]",ylabel="Joint change [deg]");ax.legend(fontsize=7);ax.grid(alpha=.2)
    fig.suptitle("READ-ONLY preview: no motion commands; no collision validation",fontsize=13)
    fig.savefig(args.output/"workspace.png",dpi=160);plt.close(fig)
    for key,image in images.items():plt.imsave(args.output/(key+".png"),image)
    print("FLASH_WORKSPACE_PREVIEW="+json.dumps({k:report[k] for k in ("status","sample_count","per_joint_excursion_deg","flange_max_displacement_from_start_m","flange_nearest_training_distance_m","fk_kdl_max_abs_error","execution_eligible","robot_motion_commanded")}),flush=True)


if __name__=="__main__":main()
