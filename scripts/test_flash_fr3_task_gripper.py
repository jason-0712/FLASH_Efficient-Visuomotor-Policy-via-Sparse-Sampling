"""Isolated real ROS actions; no Franka driver, dynamics or physical gripper."""
import os
import threading
import time
import unittest
import rclpy
from rclpy.action import ActionServer,GoalResponse
from rclpy.executors import SingleThreadedExecutor
from franka_msgs.action import Grasp,Move
from std_srvs.srv import Trigger
from flash_fr3_task_gripper import TaskGripper

class GripperTransportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.environ.get('ROS_DOMAIN_ID')!='229' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
            raise RuntimeError('需要隔离 domain229 与 localhost only')
        rclpy.init()
    @classmethod
    def tearDownClass(cls):rclpy.shutdown()
    def setUp(self):
        self.fixture=rclpy.create_node('isolated_task_hand_fixture');self.client_node=rclpy.create_node('isolated_task_hand_client')
        self.fail=False;self.reject=False;self.calls=[];self.stops=0
        def grasp(handle):
            self.calls.append(('grasp',handle.request));result=Grasp.Result(success=not self.fail,error='fixture' if self.fail else '')
            if self.fail:handle.abort()
            else:handle.succeed()
            return result
        def move(handle):self.calls.append(('move',handle.request));handle.succeed();return Move.Result(success=True)
        def stop(request,response):self.stops+=1;response.success=True;return response
        self.g=ActionServer(self.fixture,Grasp,'/franka_gripper/grasp',execute_callback=grasp,
                    goal_callback=lambda request:GoalResponse.REJECT if self.reject else GoalResponse.ACCEPT)
        self.m=ActionServer(self.fixture,Move,'/franka_gripper/move',execute_callback=move)
        self.s=self.fixture.create_service(Trigger,'/franka_gripper/stop',stop)
        self.executor=SingleThreadedExecutor();self.executor.add_node(self.fixture)
        self.thread=threading.Thread(target=self.executor.spin);self.thread.start()
        self.hand=TaskGripper(self.client_node);self.hand.ready()
        self.config={'gripper_target_width_m':.029}
    def tearDown(self):
        self.executor.shutdown();self.thread.join();self.g.destroy();self.m.destroy()
        self.fixture.destroy_node();self.client_node.destroy_node()
    def test_actual_grasp_and_release_goals(self):
        self.hand.perform('grasp',self.config);self.hand.perform('release',self.config)
        self.assertEqual([x[0] for x in self.calls],['grasp','move'])
        goal=self.calls[0][1];self.assertEqual(goal.width,.029);self.assertEqual(goal.force,20.)
        self.assertEqual(goal.epsilon.outer,.012);self.assertEqual(self.calls[1][1].width,.08)
    def test_failure_does_not_send_release(self):
        self.fail=True
        with self.assertRaises(RuntimeError):self.hand.perform('grasp',self.config)
        self.assertEqual([x[0] for x in self.calls],['grasp'])
        self.assertFalse(self.hand.last_action['sdk_success'])
        self.assertEqual(self.hand.last_action['stage'],'completed')
    def test_rejected_goal_and_explicit_stop_cleanup(self):
        self.reject=True
        with self.assertRaises(RuntimeError):self.hand.perform('grasp',self.config)
        self.assertFalse(self.calls)
        self.hand.may_be_active=True;self.hand.stop_if_needed();self.assertEqual(self.stops,1)
    def test_default_does_not_send_a_gripper_goal(self):
        self.hand.stop_if_needed();self.assertFalse(self.calls);self.assertEqual(self.stops,0)

if __name__=='__main__':unittest.main()
