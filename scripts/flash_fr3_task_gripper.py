"""Franka action transport. Supports nonblocking goals alongside arm control."""
import time
import rclpy
from action_msgs.msg import GoalStatus
from franka_msgs.action import Grasp,Move
from rclpy.action import ActionClient
from std_srvs.srv import Trigger


class TaskGripper:
    def __init__(self,node):
        self.node=node;self.may_be_active=False;self.last_action=None
        self.future=None;self.handle=None;self.deadline=0.
        self.grasp=ActionClient(node,Grasp,'/franka_gripper/grasp')
        self.move=ActionClient(node,Move,'/franka_gripper/move')
        self.stop=node.create_client(Trigger,'/franka_gripper/stop')
    def ready(self):
        if not (self.grasp.wait_for_server(timeout_sec=3) and self.move.wait_for_server(timeout_sec=3)
                and self.stop.wait_for_service(timeout_sec=3)):
            raise RuntimeError('夹爪 grasp/move/stop 接口未就绪')
    def wait(self,future,seconds):
        deadline=time.monotonic()+seconds
        while not future.done() and time.monotonic()<deadline:rclpy.spin_once(self.node,timeout_sec=.01)
        if not future.done():raise RuntimeError('夹爪动作或响应超时')
        return future.result()
    def begin(self,kind,config):
        if self.may_be_active:raise RuntimeError('夹爪已有未完成动作')
        speed=float(config.get('gripper_speed_m_s',.02))
        if kind=='grasp':
            force=float(config.get('gripper_force_N',20.))
            goal=Grasp.Goal(width=float(config['gripper_target_width_m']),speed=speed,force=force)
            goal.epsilon.inner=goal.epsilon.outer=float(config.get('gripper_epsilon_m',.012));client=self.grasp
        elif kind in ('release','open'):
            goal=Move.Goal(width=.08,speed=speed);client=self.move
        else:raise ValueError('未知夹爪任务')
        self.last_action={'kind':kind,'success':False,'stage':'goal_sent','goal_width_m':goal.width,
                          'speed_m_s':goal.speed,'force_N':float(goal.force) if kind=='grasp' else None}
        self.may_be_active=True;self.handle=None
        self.future=client.send_goal_async(goal);self.deadline=time.monotonic()+3.
    def poll(self):
        if not self.may_be_active:return None
        if time.monotonic()>self.deadline:raise RuntimeError('夹爪动作或响应超时')
        if not self.future.done():return None
        if self.handle is None:
            self.handle=self.future.result()
            self.last_action['goal_accepted']=self.handle.accepted
            if not self.handle.accepted:
                self.last_action['stage']='goal_rejected';self.may_be_active=False
                raise RuntimeError('夹爪拒绝动作')
            self.future=self.handle.get_result_async();self.deadline=time.monotonic()+8.
            return None
        result=self.future.result();self.may_be_active=False
        self.last_action.update(stage='completed',action_status=int(result.status),
                                sdk_success=bool(result.result.success),error=result.result.error)
        if result.status!=GoalStatus.STATUS_SUCCEEDED or not result.result.success:
            raise RuntimeError('夹爪动作失败: status='+str(result.status)+
                               ', success='+str(result.result.success)+', '+
                               (result.result.error or '驱动未提供错误文本'))
        self.last_action['success']=True
        return dict(self.last_action)
    def perform(self,kind,config):
        self.begin(kind,config)
        while True:
            rclpy.spin_once(self.node,timeout_sec=.01)
            result=self.poll()
            if result is not None:return result
    def stop_if_needed(self):
        if not self.may_be_active:return
        if self.handle is None and self.future is not None:
            try:
                handle=self.wait(self.future,3)
                if handle.accepted:self.handle=handle
            except RuntimeError:
                # A lost goal response must not prevent the independent stop.
                pass
        if self.handle is not None and self.handle.accepted:
            self.handle.cancel_goal_async()
        response=self.wait(self.stop.call_async(Trigger.Request()),2)
        if not response.success:raise RuntimeError('夹爪 stop 服务未确认停止')
        self.may_be_active=False
