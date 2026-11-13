#!/usr/bin/env bash
set -euxo pipefail
root=/home/hanyu/flash_fr3/flash_sim_reproduction_20261028
cd "$root"
export OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4
"$root/.venv/bin/python" -m pip freeze > "$root/environment/effective-pip-freeze-neutral-cwd.txt"
"$root/.venv/bin/python" -m pip freeze --local > "$root/environment/venv-layer-pip-freeze.txt"
"$root/.venv/bin/python" -m pip list --editable > "$root/environment/editable-packages.txt"
cd "$root/FLASH-Policy"
dataset="data_policy/libero_90.kitchen_scene1_put_the_black_bowl_on_the_plateFrankaL0_obs:joint_pos_act:joint_pos_ds4_40.zarr"
"$root/.venv/bin/python" "$root/scripts/validate_demos.py" --root "$root/data-formal" --expected 40 --zarr "$dataset" --output "$root/manifests/formal-zarr-image-audit.json"
cp -a "$root/data-formal/success/demo_0001/rgb.mp4" "$root/videos/reference-success-demo0001.mp4"
cp -a "$root/data-formal/failed/demo_0000/rgb.mp4" "$root/videos/reference-failure-demo0000.mp4"
"$root/.venv/bin/python" - "$root" <<'PY'
import sys,json
from pathlib import Path
import cv2
from PIL import Image,ImageDraw
root=Path(sys.argv[1])
sheet=Image.new('RGB',(768,560),'white')
for row,idx in enumerate([7,25]):
    path=root/f'data-formal/success/demo_{idx:04d}/rgb.mp4'
    cap=cv2.VideoCapture(str(path))
    n=int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    for col,f in enumerate([0,n//2,n-1]):
        cap.set(cv2.CAP_PROP_POS_FRAMES,f)
        ok,bgr=cap.read()
        assert ok
        rgb=cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB)
        sheet.paste(Image.fromarray(rgb),(col*256,row*280+24))
        ImageDraw.Draw(sheet).text((col*256+5,row*280+6),f'source {idx} frame {f} (0 extra)',fill='black')
    cap.release()
sheet.save(root/'videos/recovered-reference-contact.png')
report=json.loads((root/'manifests/formal-zarr-image-audit.json').read_text())
print('ALL_DEMO_VIDEO_ZARR_PIXEL_MAX_DIFF',max(v['zarr_rgb_abs_diff_max'] for v in report['demonstrations']))
PY
