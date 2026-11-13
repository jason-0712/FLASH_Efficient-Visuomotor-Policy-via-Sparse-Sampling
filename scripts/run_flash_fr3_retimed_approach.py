#!/usr/bin/env python3
"""Retimed-history approach trial; gripper goals are disabled in this entry.

Default checks the new history schema and configures an inactive arm plan.
--execute is an explicitly requested bounded arm-only approach experiment.
No claim of a completed grasp or paper-equivalent control timing is made.
"""
import argparse
import json
import math
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions

from flash_fr3_history_clock import arm_interval_from_status
from flash_fr3_retimed_schema import OBSERVATION_FORMAT
from flash_fr3_return_plan import load_pose
from flash_fr3_sensor_process import RetimedIsolatedObservationBuffer
from flash_fr3_stream_io import connection
from run_flash_fr3_pickplace import TaskRunner, save


class StaleRetimedPrediction(ValueError):
    """A confirmed prediction that must be replaced while the arm is inactive."""


class RetimedApproachRunner(TaskRunner):
    def __init__(self,args):
        super().__init__(args,buffer_factory=RetimedIsolatedObservationBuffer)
        self.result_prefix='FLASH_RETIMED_APPROACH_RESULT'
        self.report.update(gripper_goals_enabled=False,
                           scope='arm-only approach with executed-motion history; not a grasp task',
                           execution_style='slowed motion; completed arm intervals advance history clock; idle gaps freeze it')

    def run(self):
        self.inactive()
        _,health=connection(self.args.server)
        self.report['server']=health
        if health.get('retimed_observations') is not True:
            raise RuntimeError('需要支持重定时观测的新推理服务；请先启动 10132 端口服务')
        target=load_pose(json.loads(self.args.home.read_text()))
        q=self.spin_static(seconds=3)['q']
        gap=float(abs(q-target).max())
        self.report['saved_start_gap_rad']=gap
        # A process started near the cube has already lost its approach history.
        # Do not pretend it can reconstruct the past from stationary frames.
        if gap>.02:
            self.report.update(status='RETURN_TO_SAVED_START_REQUIRED',target_q_rad=target.tolist(),
                               current_q_rad=q.tolist())
            return
        super().run()

    def reference_age(self,result):
        if result.get('observation_format')!=OBSERVATION_FORMAT or not result.get('history_clock'):
            raise RuntimeError('缺少服务器确认的执行历史时钟；拒绝使用预测')
        wall_age=time.monotonic()-result['observation_monotonic']
        if not math.isfinite(wall_age) or wall_age<0:
            raise RuntimeError('观测单调时钟异常；拒绝使用预测')
        self.report['latest_observation_wall_age_seconds']=wall_age
        if wall_age>.7:
            raise StaleRetimedPrediction(f'最新观测已过期：{wall_age:.3f} 秒，限制为 0.700 秒')
        # The arm is static during IO/configuration. That idle time advances
        # neither the input history clock nor the predicted source trajectory.
        return 0.

    def get_prediction(self,request_id,after):
        deadline=time.monotonic()+3.
        for attempt in range(1,4):
            result=super().get_prediction(request_id,after)
            try:
                # Reject before any task decision or trajectory preparation.
                # Idle time does not advance the recorded motion history.
                self.reference_age(result)
                arrays,metadata=self.buffer.last_observation
                if metadata['request_id']!=request_id:
                    raise RuntimeError('归档观测与本次预测编号不一致')
                path=self.args.output/f'observation_{request_id:03d}.npz'
                np.savez(path,**arrays,metadata_json=np.asarray(json.dumps(metadata)))
                self.reference_age(result)
            except StaleRetimedPrediction:
                discarded={'request_id':request_id,'attempt':attempt,
                           'observation_wall_age_seconds':self.report['latest_observation_wall_age_seconds'],
                           'observation_target_stamp_seconds':result['observation_target_stamp_seconds'],
                           'observation_to_response_ms':result['observation_to_response_ms'],
                           'prepare_ms':result['prepare_ms'],'rpc_ms':result['rpc_ms'],
                           'maximum_age_seconds':.7}
                self.report.setdefault('discarded_stale_predictions',[]).append(discarded)
                print('FLASH_RETIMED_PREDICTION_DISCARDED='+json.dumps(discarded),flush=True)
                self.inactive()
                if attempt==3 or time.monotonic()>=deadline:
                    raise RuntimeError('新鲜预测获取失败：观测超过 0.700 秒时效，有限重采集后仍未通过；机械臂保持停用')
                continue
            result['observation_archive']=str(path)
            self.report['latest_history_clock']=metadata['history_clock']
            return result

    def arm_segment(self,plan,directory):
        finished=super().arm_segment(plan,directory)
        interval=arm_interval_from_status(self.rows,plan)
        clock=self.buffer.record_arm(**interval)
        self.report['completed_history_clock']=clock
        save(directory/'executed_history_interval.json',{'interval':interval,'clock_after_segment':clock})
        return finished

    def close(self):
        if self.buffer.coverage_failures:
            self.report['sensor_history_coverage_failures']=self.buffer.coverage_failures
        if self.buffer.transport_gap_contexts:
            self.report['sensor_transport_gap_contexts']=self.buffer.transport_gap_contexts
            save(self.args.output/'sensor_transport_diagnostics.json',self.buffer.transport_gap_contexts)
        super().close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--events',type=Path,required=True)
    parser.add_argument('--home',type=Path,required=True)
    parser.add_argument('--server',default='http://192.168.1.113:10132')
    parser.add_argument('--execute',action='store_true')
    args=parser.parse_args();args.arm_only=True
    if args.output.exists():parser.error('输出目录须不存在')
    args.output.mkdir(parents=True);cv2.setNumThreads(1)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    runner=RetimedApproachRunner(args)
    try:runner.run()
    except (Exception,KeyboardInterrupt) as error:
        runner.report.update(status='TASK_FAILED',error=str(error) or '用户中断')
    finally:runner.close();rclpy.shutdown()
    if runner.report['status'] not in ('TASK_PREFLIGHT_PASS','APPROACH_GEOMETRY_REACHED'):
        raise SystemExit(1)


if __name__=='__main__':main()
