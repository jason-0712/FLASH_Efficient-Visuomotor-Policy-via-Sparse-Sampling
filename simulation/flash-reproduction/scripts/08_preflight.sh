#!/usr/bin/env bash
set -euxo pipefail
root=/home/hanyu/flash_fr3/flash_sim_reproduction_20261028
cd "$root/FLASH-Policy"
export CUDA_VISIBLE_DEVICES=GPU-f16a68c8-8e24-9ac3-0713-b9bdbc3af6f6
export MUJOCO_GL=egl MUJOCO_EGL_DEVICE_ID=3 PYOPENGL_PLATFORM=egl
export OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8
export PYTHONPATH="$PWD"
export FLASH_REPRO_ROOT="$root"
"$root/.venv/bin/python" -u - <<'PY' 2>&1 | tee "$root/logs/preflight.log"
import os, sys, json, importlib.metadata as md
from pathlib import Path
import numpy as np
import torch, mujoco, imageio.v3 as iio
root = Path(os.environ['FLASH_REPRO_ROOT'])
report = {'python': sys.version, 'executable': sys.executable,
          'versions': {p: md.version(p) for p in ['torch','torchvision','numpy','mujoco','dm-control','zarr','numcodecs']},
          'cuda_visible_devices': os.environ['CUDA_VISIBLE_DEVICES'], 'egl_device_id': os.environ['MUJOCO_EGL_DEVICE_ID'],
          'cuda_available': torch.cuda.is_available(), 'cuda_device_count': torch.cuda.device_count(),
          'cuda_device_name': torch.cuda.get_device_name(0)}
print(json.dumps(report, indent=2))
model = mujoco.MjModel.from_xml_string('''<mujoco><visual><global offwidth="320" offheight="240"/></visual><worldbody><light pos="0 0 3"/><geom type="plane" size="2 2 .1" rgba=".7 .7 .7 1"/><body pos="0 0 .15"><geom type="box" size=".15 .15 .15" rgba="1 .2 .1 1"/></body></worldbody></mujoco>''')
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
with mujoco.Renderer(model, height=240, width=320) as renderer:
    renderer.update_scene(data)
    rgb = renderer.render().copy()
    from OpenGL import GL
    report['gl_vendor'] = str(GL.glGetString(GL.GL_VENDOR))
    report['gl_renderer'] = str(GL.glGetString(GL.GL_RENDERER))
report['rgb_shape'] = list(rgb.shape)
report['rgb_std'] = float(rgb.std())
assert rgb.shape == (240,320,3) and rgb.std() > 5
iio.imwrite(root/'videos/egl_probe.png', rgb)
(root/'environment/preflight.json').write_text(json.dumps(report, indent=2))
print('EGL_RENDER_PASS', report)
from metasim.randomization import DomainRandomizationManager, DRConfig
print('DOMAIN_RANDOMIZATION_IMPORT_PASS')
from metasim.sim.mujoco import MujocoHandler
print('MUJOCO_HANDLER_IMPORT_PASS')
from roboverse_pack.tasks.libero_90.libero_kitchen_scene1_put_the_black_bowl_on_the_plate import LiberoKitchen1PutBowlOnPlateTask
print('TARGET_TASK_IMPORT_PASS', LiberoKitchen1PutBowlOnPlateTask.traj_filepath)
from roboverse_learn.il.runners.default_runner import DefaultRunner
print('TRAIN_EVAL_RUNNER_IMPORT_PASS')
PY
