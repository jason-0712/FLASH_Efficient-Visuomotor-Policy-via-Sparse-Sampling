#!/usr/bin/env bash
set -euxo pipefail
root=/home/hanyu/flash_fr3/flash_sim_reproduction_20261028
"$root/.venv/bin/python" - "$root/comparison_20261028" <<'PY'
import sys,cv2,numpy as np
from pathlib import Path
from PIL import Image,ImageDraw
out=Path(sys.argv[1]);canvas=Image.new('RGB',(5*256,2*320),'#fafafa');draw=ImageDraw.Draw(canvas)
for row,policy in enumerate(['flash','fm_dit']):
    folder=out/f'stability-{policy}-10000';video=next((folder/'official-style/eval').rglob('0.mp4'))
    cap=cv2.VideoCapture(str(video));data=np.load(folder/'episodes/0000.npz')
    for col,frame in enumerate([0,74,149,224,299]):
        cap.set(cv2.CAP_PROP_POS_FRAMES,frame);ok,bgr=cap.read();assert ok
        x,y=col*256,row*320
        canvas.paste(Image.fromarray(cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB)),(x,y+20))
        speed=np.linalg.norm(data['bowl_linear_velocity_world'][frame]);angular=np.linalg.norm(data['bowl_angular_velocity_world'][frame]);q=data['finger_positions'][frame]
        draw.text((x+5,y+3),f'{policy} | t={(frame+1)*.015:.3f} s',fill='#111111')
        draw.text((x+5,y+279),f'geom={int(data["geometric_success"][frame])} contacts={data["bowl_hand_contact_count"][frame]}',fill='#111111')
        draw.text((x+5,y+292),f'q=({q[0]:.3f}, {q[1]:.3f}) m',fill='#111111')
        draw.text((x+5,y+305),f'v={speed:.3f} m/s w={angular:.3f} rad/s',fill='#111111')
    cap.release();data.close()
canvas.save(out/'results/scene0-contact-sheet.png')
PY
