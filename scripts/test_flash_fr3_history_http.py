"""Test actual HTTP/schema dispatch with prescribed inference, no torch/robot."""
import importlib.util
import io
import json
import sys
import threading
import types
import unittest
import urllib.error
import urllib.request
from pathlib import Path
from unittest.mock import patch

import numpy as np

from test_flash_fr3_history import fixture_buffer, observe


class ArrayResult:
    def __init__(self,value):self.value=value
    def __getitem__(self,key):return ArrayResult(self.value[key])
    def detach(self):return self
    def cpu(self):return self
    def numpy(self):return self.value


class HTTPTests(unittest.TestCase):
    def setUp(self):
        self.arrays,self.metadata=observe(fixture_buffer())
        self.predictions=[];self.legacy_reads=[]
        module=types.ModuleType('infer_flash_fr3_observation')
        def legacy_read(stream):
            self.legacy_reads.append(True)
            with np.load(stream,allow_pickle=False) as archive:
                return ({key:archive[key].copy() for key in ('agent_pos','primary_cam','wrist_cam')},
                        json.loads(str(archive['metadata_json'].item())))
        def predict(model,arrays):
            self.predictions.append(arrays)
            return {'action':ArrayResult(np.zeros((1,8,8),np.float32))}
        module.load_policy=lambda *args:None
        module.predict_arrays=predict;module.read_observation=legacy_read
        path=Path(__file__).with_name('serve_flash_fr3_shadow.py')
        spec=importlib.util.spec_from_file_location('history_http_fixture',path)
        self.server_module=importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules,{'torch':types.ModuleType('torch'),'infer_flash_fr3_observation':module}):
            spec.loader.exec_module(self.server_module)
        self.server_module.physical_curve=lambda *args:{'diagnostic_stub':True}
        stats={'min':ArrayResult(np.full(8,-10.)),'max':ArrayResult(np.full(8,10.))}
        normalizer={'agent_pos':types.SimpleNamespace(get_input_stats=lambda:stats)}
        self.model=types.SimpleNamespace(normalizer=normalizer)
        self.servers=[]
        self.opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def start(self,retimed):
        server=self.server_module.make_server(('127.0.0.1',0),self.model,
            {'physical_time':{},'checkpoint_step':15000},{'127.0.0.1'},allow_retimed=retimed)
        worker=threading.Thread(target=server.serve_forever,kwargs={'poll_interval':.01})
        worker.start();self.servers.append((server,worker))
        return 'http://127.0.0.1:'+str(server.server_port)

    def post(self,url,metadata):
        blob=io.BytesIO();np.savez(blob,**self.arrays,metadata_json=np.asarray(json.dumps(metadata)))
        request=urllib.request.Request(url+'/predict',data=blob.getvalue())
        with self.opener.open(request,timeout=5) as response:return json.load(response)

    def tearDown(self):
        for server,worker in self.servers:
            server.shutdown();worker.join(timeout=2);server.server_close()

    def test_new_server_validates_retimed_metadata_before_inference_and_echoes_clock(self):
        base=self.start(True)
        with self.opener.open(base+'/health',timeout=5) as response:
            self.assertIs(json.load(response)['retimed_observations'],True)
        reply=self.post(base,self.metadata)
        self.assertEqual(reply['history_clock'],self.metadata['history_clock'])
        self.assertEqual(reply['observation_format'],self.metadata['format'])
        self.assertEqual(len(self.predictions),1)
        for key in self.arrays:np.testing.assert_array_equal(self.predictions[0][key],self.arrays[key])
        bad=json.loads(json.dumps(self.metadata));bad['history_clock']['cursor_seconds']+=1
        with self.assertRaises(urllib.error.HTTPError) as error:self.post(base,bad)
        self.assertEqual(error.exception.code,400)
        self.assertEqual(len(self.predictions),1)
        self.assertFalse(self.legacy_reads)

    def test_old_mode_rejects_retimed_input_and_retains_legacy_dispatch(self):
        base=self.start(False)
        with self.assertRaises(urllib.error.HTTPError) as error:self.post(base,self.metadata)
        self.assertEqual(error.exception.code,400);self.assertFalse(self.predictions)
        legacy=dict(self.metadata);legacy['format']='flash_fr3_live_observation_v1'
        reply=self.post(base,legacy)
        self.assertEqual(reply['observation_format'],'flash_fr3_live_observation_v1')
        self.assertEqual(len(self.legacy_reads),1)


if __name__=='__main__':unittest.main()
