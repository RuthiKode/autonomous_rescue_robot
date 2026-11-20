#!/usr/bin/env python3
"""Render week-5 evidence from results/phase4: map + pins vs truth, coverage curves, path GIF."""
import csv, glob, os, yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

ROOT = 'results/phase4'
WORLDS = 'src/rescue_simulation/worlds'
OUT = 'docs/img/phase4'
os.makedirs(OUT, exist_ok=True)


def run_dir(world, method):
    return sorted(glob.glob(f'{ROOT}/runs/{world}_{method}_seed1_*'))[-1]


def load(d):
    meta = yaml.safe_load(open(f'{d}/map.yaml'))
    img = np.array(Image.open(f'{d}/{meta["image"]}'))
    res, (ox, oy, _) = meta['resolution'], meta['origin']
    traj = np.loadtxt(f'{d}/trajectory_slam.tum')
    pins = [r for r in csv.DictReader(open(f'{d}/survivors.csv'))]
    return img, res, ox, oy, traj, pins


def draw(ax, world, d):
    img, res, ox, oy, traj, pins = load(d)
    h, w = img.shape
    ax.imshow(img, cmap='gray', origin='upper', extent=(ox, ox + w * res, oy, oy + h * res))
    ax.plot(traj[:, 1], traj[:, 2], '-', lw=0.8, color='tab:blue', label='robot path')
    truth = yaml.safe_load(open(f'{WORLDS}/{world}_survivors.yaml'))['survivors']
    ax.scatter([s['position']['x'] for s in truth], [s['position']['y'] for s in truth],
               marker='o', s=160, facecolors='none', edgecolors='lime', linewidths=2,
               label='true survivor')
    ax.scatter([float(p['x']) for p in pins], [float(p['y']) for p in pins],
               marker='x', s=60, color='red', label='confirmed pin')
    ax.set_title(world); ax.set_aspect('equal'); ax.legend(loc='lower right', fontsize=7)


fig, axes = plt.subplots(1, 3, figsize=(16, 5.5))
for ax, w in zip(axes, ['collapsed', 'rooms', 'open_plan']):
    draw(ax, w, run_dir(w, 'oracle_pipeline'))
fig.suptitle('Oracle detector, seed 1: final SLAM map, path, pins vs truth')
fig.tight_layout(); fig.savefig(f'{OUT}/pins_vs_truth.png', dpi=110); plt.close(fig)

fig, ax = plt.subplots(figsize=(7, 4))
for d in sorted(glob.glob(f'{ROOT}/runs/*seed*')):
    name = os.path.basename(d).rsplit('_seed', 1)[0]
    rows = list(csv.DictReader(open(f'{d}/coverage.csv')))
    ax.plot([float(r['sim_time_s']) for r in rows], [float(r['coverage']) for r in rows],
            lw=1, label=name + ' s' + d.split('_seed')[1][0])
ax.axhline(0.9, color='gray', ls=':'); ax.set_xlabel('sim time (s)'); ax.set_ylabel('coverage')
ax.legend(fontsize=5, ncol=2); fig.tight_layout(); fig.savefig(f'{OUT}/coverage_curves.png', dpi=130)

# GIF: the path being driven over the final map (pins are the final ones; they carry no timestamps)
d = run_dir('collapsed', 'oracle_pipeline')
img, res, ox, oy, traj, pins = load(d)
frames = []
for k in np.linspace(5, len(traj), 40).astype(int):
    fig, ax = plt.subplots(figsize=(5, 5))
    h, w = img.shape
    ax.imshow(img, cmap='gray', extent=(ox, ox + w * res, oy, oy + h * res))
    ax.plot(traj[:k, 1], traj[:k, 2], color='tab:blue', lw=1)
    ax.plot(traj[k - 1, 1], traj[k - 1, 2], 'o', color='orange')
    ax.set_title(f'collapsed, oracle: path to t={traj[k-1,0]-traj[0,0]:.0f} s (final map shown)')
    ax.set_aspect('equal'); fig.canvas.draw()
    frames.append(Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[..., :3]))
    plt.close(fig)
frames[0].save(f'{OUT}/collapsed_path.gif', save_all=True, append_images=frames[1:], duration=120, loop=0)
print('rendered to', OUT)
