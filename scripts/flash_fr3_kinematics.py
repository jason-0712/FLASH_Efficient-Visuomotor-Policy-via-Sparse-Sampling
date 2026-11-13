"""URDF forward kinematics for workspace diagnostics; no robot IO."""
import xml.etree.ElementTree as ET

import numpy as np


def vector(text, default):
    result = np.asarray([float(v) for v in text.split()] if text else default)
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError("URDF 三维向量异常")
    return result


def rpy_rotation(rpy):
    r, p, y = rpy
    cr, cp, cy = np.cos([r,p,y]); sr, sp, sy = np.sin([r,p,y])
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                     [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])


class UrdfKinematics:
    def __init__(self, path, base="fr3_link0", tip="fr3_link8"):
        self.base, self.tip = base, tip
        self.root = ET.parse(path).getroot()
        by_child = {j.find("child").get("link"):j for j in self.root.findall("joint")}
        chain, current, seen = [], tip, set()
        while current != base:
            if current in seen or current not in by_child:
                raise ValueError("URDF 无法连接指定基座与末端")
            seen.add(current)
            joint = by_child[current]; chain.append(joint)
            current = joint.find("parent").get("link")
        self.chain = []
        for j in reversed(chain):
            origin = j.find("origin")
            xyz = vector(origin.get("xyz") if origin is not None else None,[0,0,0])
            rpy = vector(origin.get("rpy") if origin is not None else None,[0,0,0])
            transform = np.eye(4); transform[:3,:3]=rpy_rotation(rpy); transform[:3,3]=xyz
            kind=j.get("type"); name=j.get("name")
            if kind not in ("fixed","revolute","continuous"):
                raise ValueError("该法兰链只支持固定或旋转关节")
            axis_el=j.find("axis")
            axis=vector(axis_el.get("xyz") if axis_el is not None else None,[1,0,0])
            if np.linalg.norm(axis)<1e-12:raise ValueError("URDF 关节轴长度为零")
            axis=axis/np.linalg.norm(axis)
            self.chain.append({"name":name,"kind":kind,"xyz":xyz,"rpy":rpy,"axis":axis,"origin":transform})
        self.joint_names=[j["name"] for j in self.chain if j["kind"]!="fixed"]
        if self.joint_names != [f"fr3_joint{i}" for i in range(1,8)]:
            raise ValueError("法兰链关节名称或顺序异常")

    def transforms(self, q):
        q=np.asarray(q,dtype=float)
        if q.ndim!=2 or q.shape[1]!=7 or not np.isfinite(q).all():
            raise ValueError("FK 输入必须为有限的 N×7 关节位置")
        out=np.broadcast_to(np.eye(4),(len(q),4,4)).copy()
        index=0
        for joint in self.chain:
            out=out@joint["origin"]
            if joint["kind"]=="fixed":continue
            x,y,z=joint["axis"]
            skew=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
            outer=np.outer(joint["axis"],joint["axis"])
            cos=np.cos(q[:,index])[:,None,None]; sin=np.sin(q[:,index])[:,None,None]
            motion=np.broadcast_to(np.eye(4),(len(q),4,4)).copy()
            motion[:,:3,:3]=cos*np.eye(3)+(1-cos)*outer+sin*skew
            out=out@motion;index+=1
        return out

    def positions(self, q):
        return self.transforms(q)[:,:3,3]

    def check_against_kdl(self, q):
        """Independent FK implementation and RPY construction from installed Orocos KDL."""
        import PyKDL as kdl
        chain=kdl.Chain()
        for j in self.chain:
            frame=kdl.Frame(kdl.Rotation.RPY(*j["rpy"]),kdl.Vector(*j["xyz"]))
            chain.addSegment(kdl.Segment(j["name"]+"_origin",kdl.Joint(kdl.Joint.Fixed),frame))
            if j["kind"]!="fixed":
                rotation=kdl.Joint(j["name"],kdl.Vector(0,0,0),kdl.Vector(*j["axis"]),kdl.Joint.RotAxis)
                chain.addSegment(kdl.Segment(j["name"]+"_rotation",rotation))
        solver=kdl.ChainFkSolverPos_recursive(chain)
        expected=self.transforms(q); error=0.
        for values,matrix in zip(q,expected):
            joints=kdl.JntArray(7)
            for i,v in enumerate(values):joints[i]=float(v)
            frame=kdl.Frame()
            if solver.JntToCart(joints,frame)<0:raise ValueError("KDL FK 求解失败")
            actual=np.eye(4)
            actual[:3,:3]=[[frame.M[i,j] for j in range(3)] for i in range(3)]
            actual[:3,3]=[frame.p[i] for i in range(3)]
            error=max(error,float(np.abs(actual-matrix).max()))
        if error>1e-10:raise ValueError("FK 与独立 KDL 求解结果不一致")
        return error
