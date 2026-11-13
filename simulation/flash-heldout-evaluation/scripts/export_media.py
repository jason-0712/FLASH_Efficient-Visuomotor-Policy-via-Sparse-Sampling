"""Export the preselected first episode; preserve the official pre-action video."""
import argparse
import json
from pathlib import Path
import shutil
import imageio.v2 as imageio
from PIL import Image, ImageDraw, ImageFont

ap = argparse.ArgumentParser()
ap.add_argument('--root', required=True, type=Path)
root = ap.parse_args().root
(root / 'videos').mkdir(exist_ok=True)
canvas = Image.new('RGB', (1280, 610), 'white')
draw = ImageDraw.Draw(canvas)
font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 14)
draw.text((12, 5), 'Frozen test_id=0 (preselected); official videos show observations BEFORE each action.', fill='black', font=font)
for row, policy in enumerate(['flash', 'fm_dit']):
    run = root / 'runs' / policy
    official, = [p for p in (run / 'official-style/eval').rglob('10000_*.ckpt_*') if p.is_dir()]
    source = official / '0.mp4'
    shutil.copyfile(source, root / 'videos' / f'{policy}-test0.mp4')
    reader = imageio.get_reader(source)
    count = reader.count_frames()
    indices = sorted({0, count // 4, count // 2, 3 * count // 4, count - 1})
    assert len(indices) == 5
    for column, index in enumerate(indices):
        frame = Image.fromarray(reader.get_data(index)).convert('RGB').resize((256, 256))
        y = 53 + row * 278
        canvas.paste(frame, (column * 256, y))
        name = 'FLASH' if policy == 'flash' else 'FM-DiT'
        draw.text((column * 256 + 6, y - 22), f'{name} | action {index+1} | t={index*.015:.3f}s', fill='black', font=font)
    reader.close()
draw.text((12, 589), 'Final post-action object states and success are verified in episodes/*.npz; video/container time differs.', fill='#384c60', font=font)
canvas.save(root / 'results/test0-contact-sheet.png')
print('MEDIA_EXPORTED_FIXED_TEST_ID_0')
