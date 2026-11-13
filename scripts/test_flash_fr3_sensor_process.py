"""Check actual parent/child RMW selection in an isolated, subscriber-only domain."""
import argparse
import json
import multiprocessing
import os
from pathlib import Path
from unittest.mock import Mock

import rclpy

from flash_fr3_sensor_process import IsolatedObservationBuffer, _start_sensor_process


def old_startup_order_probe(pipe):
    # Reproduce the old target: the spawned interpreter has already imported
    # this entry point (and rclpy) before assigning the RMW environment here.
    os.environ['RMW_IMPLEMENTATION']='rmw_cyclonedds_cpp'
    pipe.send({'requested_after_import':os.environ['RMW_IMPLEMENTATION'],
               'actual':rclpy.get_rmw_implementation_identifier()})
    pipe.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='229' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        parser.error('需要隔离 domain229 与 localhost only')
    if rclpy.get_rmw_implementation_identifier()!='rmw_fastrtps_cpp':
        parser.error('父进程必须在启动时加载 Fast DDS')
    args.output.mkdir(parents=True,exist_ok=False)
    original=os.environ.get('RMW_IMPLEMENTATION')
    rclpy.init();node=rclpy.create_node('flash_sensor_process_selection_test')
    cases=[]
    try:
        os.environ['RMW_IMPLEMENTATION']='rmw_fastrtps_cpp'
        context=multiprocessing.get_context('spawn');reader,writer=context.Pipe()
        probe=context.Process(target=old_startup_order_probe,args=(writer,))
        probe.start();writer.close()
        try:
            assert reader.poll(5),'old-order probe timed out'
            old=reader.recv();assert old['actual']=='rmw_fastrtps_cpp',old
        finally:
            probe.join(timeout=2)
            if probe.is_alive():probe.terminate();probe.join(timeout=2)
            reader.close()
        assert probe.exitcode==0,probe.exitcode
        cases.append({'case':'old_spawn_order_loads_wrong_rmw','status':'PASS',**old})
        for retimed,previous in ((False,'rmw_fastrtps_cpp'),(True,None)):
            if previous is None:os.environ.pop('RMW_IMPLEMENTATION',None)
            else:os.environ['RMW_IMPLEMENTATION']=previous
            buffer=IsolatedObservationBuffer(node,local_keys=('arm',),retimed=retimed)
            try:
                assert os.environ.get('RMW_IMPLEMENTATION')==previous
                assert rclpy.get_rmw_implementation_identifier()=='rmw_fastrtps_cpp'
                assert buffer.receiver_info['actual_rmw']=='rmw_cyclonedds_cpp'
                assert buffer.receiver_info['udp_sockets']
                cases.append({'case':'retimed_child' if retimed else 'legacy_child','status':'PASS',
                              'parent_actual_rmw':rclpy.get_rmw_implementation_identifier(),
                              'parent_environment_restored':True,'receiver':buffer.receiver_info})
            finally:buffer.close()
            assert buffer.process.exitcode==0,buffer.process.exitcode
        failed=Mock();failed.start.side_effect=RuntimeError('prescribed spawn failure')
        before=os.environ.get('RMW_IMPLEMENTATION')
        try:_start_sensor_process(failed)
        except RuntimeError as error:assert str(error)=='prescribed spawn failure'
        else:raise AssertionError('expected prescribed start failure')
        assert os.environ.get('RMW_IMPLEMENTATION')==before
        cases.append({'case':'restore_environment_after_spawn_error','status':'PASS'})
    finally:
        if original is None:os.environ.pop('RMW_IMPLEMENTATION',None)
        else:os.environ['RMW_IMPLEMENTATION']=original
        node.destroy_node();rclpy.try_shutdown()
    result={'status':'SENSOR_PROCESS_SELECTION_PASS','cases':cases,
            'hardware_driver_loaded':False,'robot_motion_commanded':False,'gripper_motion_commanded':False}
    (args.output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result),flush=True)


if __name__=='__main__':main()
