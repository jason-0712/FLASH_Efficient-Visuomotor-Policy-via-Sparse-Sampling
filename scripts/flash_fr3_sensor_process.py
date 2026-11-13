"""Cyclone camera reception isolated from the Fast DDS control client.

The child subscribes only. It has no controller, command publisher, service
client, heartbeat or gripper action. Full observations cross a local pipe.
"""
import multiprocessing
import os
import threading


_SPAWN_ENVIRONMENT_LOCK=threading.Lock()
SENSOR_RMW='rmw_cyclonedds_cpp'


def _start_sensor_process(process):
    # spawn imports the entry point before calling _sensor_worker. That entry
    # point imports rclpy, which already selects an RMW. Set the child's inherited
    # environment before spawn, then restore the control client's environment.
    # Its already-loaded Fast DDS implementation remains unchanged.
    with _SPAWN_ENVIRONMENT_LOCK:
        previous=os.environ.get('RMW_IMPLEMENTATION')
        try:
            os.environ['RMW_IMPLEMENTATION']=SENSOR_RMW
            process.start()
        finally:
            if previous is None:os.environ.pop('RMW_IMPLEMENTATION',None)
            else:os.environ['RMW_IMPLEMENTATION']=previous


def _sensor_worker(pipe,retimed=False,camera_time_basis='ros_header'):
    import cv2
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from run_flash_fr3_shadow_client import ObservationBuffer
    actual_rmw=rclpy.get_rmw_implementation_identifier()
    receiver_info={'pid':os.getpid(),'requested_rmw':SENSOR_RMW,'actual_rmw':actual_rmw,
                   'camera_time_basis':camera_time_basis}
    if actual_rmw!=SENSOR_RMW:
        pipe.send(('startup_error',receiver_info));pipe.close()
        return
    cv2.setNumThreads(1)
    # Parent owns orderly shutdown through the pipe. Leave SIGTERM available
    # for the parent's bounded fallback if the subscriber stops responding.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('flash_fr3_dedicated_observation_receiver')
    if retimed:
        from flash_fr3_retimed_buffer import RetimedObservationBuffer
        buffer=RetimedObservationBuffer(node)
    else:buffer=ObservationBuffer(node,camera_time_basis=camera_time_basis)
    from flash_fr3_transport_diagnostics import own_udp_sockets
    receiver_info['udp_sockets']=own_udp_sockets()
    stop=threading.Event();errors=[];transport_gap_recorded=False
    def spin():
        try:
            while not stop.is_set():rclpy.spin_once(node,timeout_sec=.002)
        except Exception as error:errors.append(str(error))
    worker=threading.Thread(target=spin);worker.start()
    try:
        pipe.send(('ready',receiver_info))
        while True:
            operation,request_id=pipe.recv()
            if operation=='stop':break
            try:
                if errors:raise RuntimeError(str(errors))
                if operation=='record_arm' and retimed:
                    pipe.send(('clock',buffer.record_arm(**request_id)))
                    continue
                if operation!='observation':raise ValueError('未知观测请求')
                arrays,metadata=buffer.observation(request_id)
                metadata['receiver']='dedicated_cyclone_process'
                metadata['receiver_rmw_implementation']=actual_rmw
                pipe.send(('ok',(arrays,metadata)))
            except Exception as error:
                diagnostic=getattr(error,'diagnostic',None)
                if diagnostic is not None:
                    if retimed and buffer.execution_clock is not None and not transport_gap_recorded:
                        # Preserve the actual receiver's first failure context,
                        # independently of the bounded warm-up/retry log below.
                        try:context=buffer.transport_gap_context(diagnostic)
                        except Exception as capture_error:
                            context={'status':'TRANSPORT_CONTEXT_CAPTURE_FAILED','error':str(capture_error)}
                        diagnostic=dict(diagnostic,transport_context=context)
                        transport_gap_recorded=True
                    pipe.send(('coverage_error',{'message':str(error),'diagnostic':diagnostic}))
                else:pipe.send(('error',str(error)))
    except (EOFError,BrokenPipeError):pass
    finally:
        stop.set();worker.join(timeout=2)
        node.destroy_node();rclpy.try_shutdown();pipe.close()


class IsolatedObservationBuffer:
    def __init__(self,node,local_keys=('arm',),retimed=False,camera_time_basis='ros_header'):
        if retimed and camera_time_basis!='ros_header':
            raise ValueError('重定时入口不接受原始策略的相机时钟配置')
        from run_flash_fr3_shadow_client import ObservationBuffer
        # Small joint messages remain local to the control client. The child
        # acquires its own synchronized arm/hand/image history for inference.
        self.local=ObservationBuffer(node,keys=local_keys)
        self.lock,self.history=self.local.lock,self.local.history
        self.coverage_failures=[]
        self.transport_gap_contexts=[]
        context=multiprocessing.get_context('spawn')
        self.pipe,child=context.Pipe()
        self.rpc_lock=threading.Lock()
        self.process=context.Process(target=_sensor_worker,args=(child,retimed,camera_time_basis),daemon=True)
        try:_start_sensor_process(self.process)
        except Exception:
            child.close();self.pipe.close()
            raise
        child.close()
        if not self.pipe.poll(8):
            self.process.terminate();self.process.join(timeout=2)
            raise RuntimeError('独立相机接收进程启动超时')
        kind,detail=self.pipe.recv()
        if kind!='ready':
            self.process.join(timeout=2);self.pipe.close()
            raise RuntimeError('相机接收进程未加载所需 DDS: '+str(detail))
        self.receiver_info=detail
    def observation(self,request_id):
        with self.rpc_lock:
            if not self.process.is_alive():raise ValueError('独立相机接收进程已退出')
            self.pipe.send(('observation',request_id))
            if not self.pipe.poll(.5):raise ValueError('本机观测进程响应超时')
            kind,value=self.pipe.recv()
            if kind=='coverage_error':
                self.coverage_failures.append(value['diagnostic'])
                self.coverage_failures=self.coverage_failures[-32:]
                context=value['diagnostic'].get('transport_context')
                if context is not None:self.transport_gap_contexts.append(context)
                raise ValueError(value['message'])
            if kind!='ok':raise ValueError(value)
            if value[1].get('request_id')!=request_id:
                raise ValueError('本机观测响应编号不匹配')
            return value
    def close(self):
        if self.process.is_alive():
            try:self.pipe.send(('stop',0))
            except (BrokenPipeError,EOFError,OSError):pass
            self.process.join(timeout=5)
            if self.process.is_alive():self.process.terminate();self.process.join(timeout=2)
        self.pipe.close()


class RetimedIsolatedObservationBuffer(IsolatedObservationBuffer):
    def __init__(self,node,local_keys=('arm',)):
        super().__init__(node,local_keys=local_keys,retimed=True)
        self.last_observation=None

    def observation(self,request_id):
        result=super().observation(request_id)
        self.last_observation=result
        return result

    def record_arm(self,**interval):
        with self.rpc_lock:
            if not self.process.is_alive():raise ValueError('独立相机接收进程已退出')
            self.pipe.send(('record_arm',interval))
            if not self.pipe.poll(.5):raise ValueError('历史时钟进程响应超时')
            kind,value=self.pipe.recv()
            if kind!='clock':raise ValueError(str(value))
            return value
