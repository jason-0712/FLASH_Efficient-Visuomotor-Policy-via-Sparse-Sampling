"""Scientific summary of the frozen held-out paired evaluation."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument('--root', required=True, type=Path)
root = ap.parse_args().root
summary = json.loads((root / 'results/summary.json').read_text())
fig, axes = plt.subplots(1, 2, figsize=(10, 4.9), gridspec_kw={'width_ratios': [1.2, 1]})
names = ['flash', 'fm_dit']
scores = [summary['policies'][n]['success_rate'] * 100 for n in names]
ci = np.asarray([summary['policies'][n]['descriptive_wilson_95'] for n in names]).T * 100
axes[0].barh([1, 0], scores, height=.5, color=['#087f8c', '#bb683a'], alpha=.9)
axes[0].errorbar(scores, [1, 0], xerr=np.vstack((np.array(scores) - ci[0], ci[1] - np.array(scores))),
                 fmt='none', ecolor='#283947', capsize=5, linewidth=1.3)
for y, name, score in zip([1, 0], names, scores):
    axes[0].text(2, y, f"{summary['policies'][name]['successes']}/50 ({score:.0f}%)", va='center', color='white', fontsize=12, fontweight='bold')
axes[0].set(yticks=[1, 0], yticklabels=['FLASH', 'FM-DiT'], xlim=[0, 105], ylim=[-.6, 1.6],
            xlabel='Original geometric SuccessOnce (%)', title='Same frozen 50 inputs')
axes[0].grid(axis='x', alpha=.2)
axes[0].set_axisbelow(True)
paired = summary['paired_counts']
matrix = np.asarray([[paired['both_success'], paired['flash_only_success']],
                     [paired['fm_dit_only_success'], paired['both_fail']]])
axes[1].imshow(matrix, cmap='Blues', vmin=0, vmax=50)
axes[1].set(xticks=[0, 1], xticklabels=['FM-DiT success', 'FM-DiT failure'],
            yticks=[0, 1], yticklabels=['FLASH success', 'FLASH failure'], title='Paired outcomes (counts)')
for row in range(2):
    for col in range(2):
        axes[1].text(col, row, str(matrix[row, col]), ha='center', va='center', fontsize=22,
                     color='white' if matrix[row, col] > 25 else '#23384d')
axes[1].tick_params(length=0)
fig.suptitle('Unseen object positions | 10,000-step EMA | training seed 42', y=.97, fontsize=14)
fig.text(.5, .045, 'Bars: descriptive 95% Wilson intervals. Physical GPU 2; original early-stop metric.\n'
         'Fixed robot; object XY within training bounds. Release and 2 s stable hold were not evaluated.',
         ha='center', fontsize=9, color='#485465')
fig.subplots_adjust(left=.09, right=.98, top=.79, bottom=.24, wspace=.5)
fig.savefig(root / 'results/heldout-comparison.png', dpi=180)
fig.savefig(root / 'results/heldout-comparison.pdf')
plt.close(fig)
