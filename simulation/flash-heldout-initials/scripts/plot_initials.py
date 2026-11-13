"""Plot frozen initial-state coverage; no policy results are plotted."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--root', required=True, type=Path)
    root = ap.parse_args().root
    split = json.loads((root / 'configs/reference-split.json').read_text())
    protocol = json.loads((root / 'configs/generation-protocol.json').read_text())
    new = json.loads((root / 'frozen-v1/test-initials.json').read_text())['states']
    selected = json.loads((root / 'frozen-v1/selected-candidates.json').read_text())
    fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.6))
    groups = [('optimization_reference_ids', 'Training (39)', '#40566c', 'o', 25),
              ('validation_reference_ids', 'Internal validation (1)', '#d1972c', 's', 55),
              ('evaluation_ids_not_in_collected_dataset', 'Other original (10)', '#888888', 'x', 35)]
    for ax, obj, title in zip(axes[:2], ['akita_black_bowl', 'plate'], ['Bowl initial position', 'Plate initial position']):
        bounds = protocol['coordinate_bounds_m'][obj]
        ax.add_patch(Rectangle((1000 * bounds['x'][0], 1000 * bounds['y'][0]),
                              1000 * np.ptp(bounds['x']), 1000 * np.ptp(bounds['y']),
                              fill=False, ls='--', lw=1, ec='#95a2ad'))
        for key, label, color, marker, size in groups:
            xy = np.asarray([r['initial_state'][obj]['pos'][:2] for r in split['initial_states'] if r['reference_index'] in split[key]]) * 1000
            ax.scatter(xy[:, 0], xy[:, 1], label=label, c=color, marker=marker, s=size, alpha=.85)
        xy = np.asarray([r['initial_state'][obj]['pos'][:2] for r in new]) * 1000
        ax.scatter(xy[:, 0], xy[:, 1], s=45, facecolors='none', edgecolors='#087f8c', linewidths=1.1, label='Frozen test (50)')
        ax.set(title=title, xlabel='World x (mm)', ylabel='World y (mm)')
        ax.set_aspect('equal', adjustable='box')
        ax.grid(alpha=.18)
    distance = np.asarray([r['nearest_original_xy4_distance_m'] for r in selected]) * 1000
    axes[2].hist(distance, bins=np.linspace(0, max(25, distance.max() + 1), 11), color='#087f8c', alpha=.85, edgecolor='white')
    axes[2].axvline(5, color='#b43c36', ls='--', lw=1.4, label='Exclusion threshold: 5 mm')
    axes[2].set(title='Distance from original 50', xlabel='Nearest original configuration (4D, mm)', ylabel='Frozen test count')
    axes[2].legend(loc='upper right', fontsize=8)
    axes[2].grid(axis='y', alpha=.18)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, ncol=4, loc='upper center', bbox_to_anchor=(.5, .90), frameon=False, fontsize=9)
    fig.suptitle('Frozen unseen object positions | CPU preparation only', y=.99, fontsize=15)
    fig.text(.5, .025, 'Robot, cabinet, object heights and rotations are fixed. Dashed boxes: training coordinate bounds.\n'
             '4D distance uses [bowl x, bowl y, plate x, plate y]; it is not an individual-object clearance.',
             ha='center', fontsize=9, color='#485465')
    fig.subplots_adjust(left=.06, right=.98, top=.77, bottom=.23, wspace=.4)
    fig.savefig(root / 'results/initial-state-coverage.png', dpi=180)
    fig.savefig(root / 'results/initial-state-coverage.pdf')
    plt.close(fig)


if __name__ == '__main__':
    main()
