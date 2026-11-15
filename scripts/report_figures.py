#!/usr/bin/env python3
"""Simple, large report figures: one message per figure, the message is the title.

    python3 scripts/report_figures.py      # -> results/stats/simple_*.pdf / .png  (run stats_analysis.py first)

  simple_1_scatter      the robot's estimate vs the tape (scatter, y = x)
  simple_2_per_trial    error of every trial: EKF vs wheels alone
  simple_3_spread       how big the errors are: EKF, camera, wheels (strip plot)
  simple_4_correlation  what is related to what (5 x 5 correlation matrix)
  simple_5_agreement    Bland-Altman: bias and limits of agreement of the EKF
  simple_6_contrib      what each part of the filter contributes (offline replay)
"""
import csv
import glob
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'results' / 'stats'
BLUE, ORANGE, GREEN, GREY, PURPLE, GOLD, INK = '#2A78D6', '#EB6834', '#0B9A55', '#7A828C', '#7B3FA0', '#D4A017', '#21252A'

import matplotlib                                         # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt                           # noqa: E402

plt.rcParams.update({
    'font.family': ['Lato', 'DejaVu Sans'], 'font.size': 15, 'axes.titlesize': 19, 'axes.titleweight': 'bold',
    'axes.titlelocation': 'left', 'axes.titlepad': 30, 'axes.labelsize': 15, 'axes.labelcolor': INK,
    'xtick.labelsize': 13.5, 'ytick.labelsize': 13.5, 'axes.spines.top': False, 'axes.spines.right': False,
    'axes.edgecolor': '#9AA1A9', 'xtick.color': '#555B62', 'ytick.color': '#555B62', 'legend.frameon': False,
    'legend.fontsize': 13, 'savefig.bbox': 'tight', 'mathtext.fontset': 'dejavusans'})


def subtitle(ax, text):
    ax.text(0, 1.015, text, transform=ax.transAxes, fontsize=13.5, color=GREY, va='bottom')


def save(fig, name):
    for ext in ('pdf', 'png'):
        fig.savefig(OUT / f'{name}.{ext}', dpi=200)
    plt.close(fig)


def load():
    rows = list(csv.DictReader(open(ROOT / 'results/summary_table.csv')))
    g = lambda k: np.array([float(r[k]) for r in rows])   # noqa: E731
    glitch = []
    for r in rows:
        f = sorted(glob.glob(str(ROOT / f"results/trials/trial_{int(r['trial']):02d}_*/summary.json")))[-1]
        glitch.append(json.load(open(f)).get('n_odom_glitches', 0))
    return dict(trial=g('trial').astype(int), est=g('d_est_m'), meas=g('d_meas_m'), e=100 * g('error_m'),
                ecam=100 * g('error_cam_m'), eodo=100 * g('error_odom_m'), glitch=np.array(glitch))


def main():
    d = load()
    st = json.loads((OUT / 'stats_analysis.json').read_text())
    rp = json.loads((ROOT / 'results/replay/replay.json').read_text())
    OUT.mkdir(parents=True, exist_ok=True)

    # 1 ---------------------------------------------------------------- scatter
    fig, ax = plt.subplots(figsize=(8.4, 7.6))
    lo, hi = 47.4, 50.2
    xx = np.array([lo, hi])
    ax.fill_between(xx, xx - 0.5, xx + 0.5, color=BLUE, alpha=0.08, lw=0)
    ax.plot(xx, xx, color=GREY, ls='--', lw=1.6)
    ax.text(49.35, 49.98, 'perfect: estimate = tape', color=GREY, fontsize=12.5, ha='right', va='center', rotation=45,
            rotation_mode='anchor')
    ax.text(lo + 0.08, lo + 0.62, '±0.5 cm', color=BLUE, fontsize=12.5, alpha=0.8)
    ax.scatter(100 * d['meas'], 100 * d['est'], s=170, color=BLUE, edgecolor='white', lw=1.8, zorder=3)
    for k in range(10):
        ax.annotate(str(d['trial'][k]), (100 * d['meas'][k], 100 * d['est'][k]), xytext=(9, -4),
                    textcoords='offset points', fontsize=12.5, color=INK)
    e = st['ekf']
    ax.text(0.97, 0.05, f"average error (MAE)   {e['mae_cm']:.2f} cm\nrange of errors   {e['range_cm']:.2f} cm\n"
            f"correlation  r = {st['regression']['pearson_r']:.2f}", transform=ax.transAxes, ha='right',
            va='bottom', fontsize=14, linespacing=1.6,
            bbox=dict(boxstyle='round,pad=0.6', facecolor='white', edgecolor='#D5D9DE'))
    ax.set_xlim(lo, hi), ax.set_ylim(lo, hi), ax.set_aspect('equal')
    ax.set_xlabel('measured with the tape (cm)')
    ax.set_ylabel('estimated by the robot, EKF (cm)')
    ax.set_title("The robot's estimate matches the tape")
    subtitle(ax, 'each dot is one trial; on the dashed line the estimate is exact')
    ax.grid(alpha=0.25)
    save(fig, 'simple_1_scatter')

    # 2 ---------------------------------------------------------------- per trial
    fig, ax = plt.subplots(figsize=(11.5, 6.2))
    x = d['trial']
    ax.axhspan(-0.5, 0.5, color=BLUE, alpha=0.08, lw=0)
    ax.bar(x - 0.19, d['eodo'], width=0.36, color=GREEN, label='wheels alone (prediction only)')
    ax.bar(x + 0.19, d['e'], width=0.36, color=BLUE, label='EKF (prediction + camera correction)')
    ax.axhline(0, color=INK, lw=1)
    for k in range(10):
        if d['glitch'][k]:
            ax.text(x[k] - 0.19, d['eodo'][k] + 0.15, f"{d['glitch'][k]} odometry\nglitches", ha='center',
                    va='bottom', fontsize=11, color=GREEN)
    ax.set_xticks(x)
    ax.set_xlabel('trial')
    ax.set_ylabel('final error  (estimate − tape, cm)')
    ax.set_ylim(-1.2, 7.4)
    ax.legend(loc='upper left')
    ax.set_title('Every EKF error stays within ±0.5 cm; the wheels alone do not')
    subtitle(ax, 'blue band = ±0.5 cm; the wheels jump whenever /odom glitches')
    ax.grid(axis='y', alpha=0.25)
    save(fig, 'simple_2_per_trial')

    # 3 ---------------------------------------------------------------- spread (strip plot)
    fig, ax = plt.subplots(figsize=(10.5, 5.6))
    groups = [('EKF', d['e'], BLUE), ('camera alone', d['ecam'], ORANGE), ('wheels alone', d['eodo'], GREEN)]
    rng = np.random.default_rng(3)
    for j, (name, v, col) in enumerate(groups):
        y = np.full(len(v), 2 - j) + rng.uniform(-0.12, 0.12, len(v))
        ax.scatter(v, y, s=110, color=col, edgecolor='white', lw=1.4, zorder=3, alpha=0.95)
        mae = np.abs(v).mean()
        ax.text(7.3, 2 - j, f'MAE {mae:.2f} cm', va='center', ha='right', fontsize=14.5, color=col,
                fontweight='bold')
    ax.axvline(0, color=INK, lw=1)
    ax.set_yticks([2, 1, 0], [g[0] for g in groups], fontsize=15)
    ax.set_xlabel('final error (cm)')
    ax.set_xlim(-1.5, 7.4), ax.set_ylim(-0.6, 2.6)
    ax.set_title('How big are the errors?')
    subtitle(ax, 'each dot is one trial; the closer to 0, the better')
    ax.grid(axis='x', alpha=0.25)
    save(fig, 'simple_3_spread')

    # 4 ---------------------------------------------------------------- correlation (5 x 5)
    names = ['tape (true)', 'EKF estimate', 'EKF error', 'wheels error', 'odometry\nglitches']
    M = np.corrcoef(np.vstack([d['meas'], d['est'], d['e'], d['eodo'], d['glitch']]))
    fig, ax = plt.subplots(figsize=(8.6, 7.4))
    mask = np.triu(np.ones_like(M, bool), 1)
    Mm = np.ma.array(M, mask=mask)
    im = ax.imshow(Mm, cmap='RdBu_r', vmin=-1, vmax=1)
    for i in range(5):
        for j in range(i + 1):
            ax.text(j, i, f'{M[i, j]:.2f}', ha='center', va='center', fontsize=16, fontweight='bold',
                    color='white' if abs(M[i, j]) > 0.6 else INK)
    ax.set_xticks(range(5), names, rotation=0, fontsize=12.5)
    ax.set_yticks(range(5), names, fontsize=12.5)
    ax.spines[:].set_visible(False)
    ax.tick_params(length=0)
    cb = fig.colorbar(im, ax=ax, fraction=0.045, pad=0.03)
    cb.set_label('correlation r   (−1 opposite · 0 unrelated · +1 move together)', fontsize=12)
    ax.set_title('What moves together?')
    subtitle(ax, 'glitches drive the wheels error (0.95) but not the EKF error (−0.13)')
    save(fig, 'simple_4_correlation')

    # 5 ---------------------------------------------------------------- Bland-Altman (EKF)
    fig, ax = plt.subplots(figsize=(10.5, 6.0))
    mean = 100 * (d['est'] + d['meas']) / 2
    m, s = d['e'].mean(), d['e'].std(ddof=1)
    ax.axhspan(m - 1.96 * s, m + 1.96 * s, color=BLUE, alpha=0.08, lw=0)
    ax.axhline(0, color=INK, lw=1)
    ax.axhline(m, color=BLUE, lw=2)
    for yv, lab in [(m, f'average bias  {m:+.2f} cm'), (m + 1.96 * s, f'upper limit  {m + 1.96 * s:+.2f} cm'),
                    (m - 1.96 * s, f'lower limit  {m - 1.96 * s:+.2f} cm')]:
        ax.axhline(yv, color=BLUE, lw=1.4 if yv != m else 2, ls='--' if yv != m else '-')
        ax.text(47.55, yv + 0.03, lab, color=BLUE, ha='left', va='bottom', fontsize=13, fontweight='bold')
    ax.scatter(mean, d['e'], s=150, color=BLUE, edgecolor='white', lw=1.6, zorder=3)
    for k in range(10):
        ax.annotate(str(d['trial'][k]), (mean[k], d['e'][k]), xytext=(8, -4), textcoords='offset points',
                    fontsize=12, color=INK)
    ax.set_xlim(47.5, 50.0), ax.set_ylim(-1.0, 1.0)
    ax.set_xlabel('distance, average of robot and tape (cm)')
    ax.set_ylabel('robot − tape (cm)')
    ax.set_title('Agreement between the robot and the tape (Bland–Altman)')
    subtitle(ax, '95 % of trials are expected inside the band; the bias is not different from 0 '
             f"(p = {st['ekf']['p_bias']:.2f})")
    ax.grid(alpha=0.25)
    save(fig, 'simple_5_agreement')

    # 6 ---------------------------------------------------------------- contributions (replay)
    fig, ax = plt.subplots(figsize=(11.5, 5.4))
    bars = [('full', BLUE, 'Full EKF'), ('dropout_0.9', PURPLE, 'camera lost for the last 40 cm'),
            ('bearing_only', GOLD, 'camera angle only, no distance'), ('no_camera', GREEN, 'no camera while driving')]
    for j, (k, col, lab) in enumerate(bars):
        v = rp[k]['mae_cm']
        ax.barh(3 - j, v, color=col, height=0.62)
        ax.text(v + 0.05, 3 - j, f'{v:.2f} cm', va='center', fontsize=15, fontweight='bold', color=INK)
    ax.set_yticks([3, 2, 1, 0], [b[2] for b in bars], fontsize=14.5)
    ax.set_xlim(0, 3.2)
    ax.set_xlabel('average final error, MAE (cm)')
    ax.set_title("Remove the camera's distance and the error grows 7×")
    subtitle(ax, 'same 10 trials, same logged data, replayed offline with one part removed')
    ax.grid(axis='x', alpha=0.25)
    save(fig, 'simple_6_contrib')
    print('written to', OUT)


if __name__ == '__main__':
    main()
