#!/usr/bin/env python3
"""All scenes of the demo video except the live run (build_scene3.py), and the final cut.

    env -u PYTHONPATH ~/phase3_video_venv/bin/python video_tools/build_scenes.py              # all + final cut
    env -u PYTHONPATH ~/phase3_video_venv/bin/python video_tools/build_scenes.py --only 3     # one scene
    env -u PYTHONPATH ~/phase3_video_venv/bin/python video_tools/build_scenes.py --still 3 4.5   # one frame (PNG)

Story (each chapter answers part of the spec):
  1 title · 2 the task (spec checklist) · 3 the method (predict / correct, Gaussian intuition)
  LIVE RUN (scene3.mp4) · 4 why correct (trial 6) · 5 what each part contributes (offline replay)
  6 results (10 trials, numbers with meaning) · 7 end card.        Output: results/video/phase3_demo.mp4
"""
import argparse
import csv
import glob
import json
import math
import multiprocessing as mp
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(ROOT / 'video_tools'))
OUT = ROOT / 'results' / 'video'
FPS = 30
W, H = 1920, 1080

import matplotlib                                          # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt                            # noqa: E402
import numpy as np                                         # noqa: E402
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Rectangle   # noqa: E402

import make_video as mv                                    # noqa: E402
from make_video import PANEL, EDGE, TEXT, MUTED, BLUE, GREEN, ORANGE, PURPLE, MONO   # noqa: E402

GOLD = '#ffcc4d'
AUTHOR = 'Ziad Aboelkhir'
REPO = 'github.com/Ziad-Behairy/SCE672-Phase3-RoboMaster-Odometry'
CHAPTERS = ['THE TASK', 'THE METHOD', 'LIVE RUN', 'WHY CORRECT', 'RESULTS']


# ------------------------------------------------------------------ helpers
def ease(x):
    x = min(max(x, 0.0), 1.0)
    return x * x * (3 - 2 * x)


def window(t, t0, t1, fade=0.4):
    """0 -> 1 from t0, 1 -> 0 before t1 (fades)."""
    return ease((t - t0) / fade) * (1 - ease((t - (t1 - fade)) / fade))


_VIG = None


def backdrop(fig):
    """Deep navy radial gradient with a soft vignette."""
    global _VIG
    if _VIG is None:
        yy, xx = np.mgrid[0:270, 0:480]
        r = np.sqrt(((xx - 240) / 300) ** 2 + ((yy - 120) / 210) ** 2)
        c0, c1 = np.array([16, 27, 44]) / 255, np.array([3, 6, 11]) / 255
        k = np.clip(r, 0, 1)[..., None] ** 1.3
        _VIG = c0 * (1 - k) + c1 * k
    ax = fig.add_axes([0, 0, 1, 1], zorder=-10)
    ax.imshow(_VIG, aspect='auto', interpolation='bilinear')
    ax.axis('off')


def fig_new():
    fig = plt.figure(figsize=(19.2, 10.8), dpi=100)
    backdrop(fig)
    return fig


def fig_img(fig):
    fig.canvas.draw()
    img = np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy()
    plt.close(fig)
    return img


def box(fig, x, y, w, h, color, alpha=1.0, lw=2.0, face=PANEL, top=False):
    fig.patches.append(FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0.004,rounding_size=0.012',
                                      transform=fig.transFigure, facecolor=face, edgecolor=color, lw=lw,
                                      alpha=alpha))
    if top:
        fig.patches.append(Rectangle((x + 0.004, y + h - 0.006), w - 0.008, 0.006, transform=fig.transFigure,
                                     facecolor=color, edgecolor='none', alpha=alpha))


def chapter(fig, idx, t, title, subtitle=''):
    """Chapter number + title (slides in) and the progress bar at the bottom."""
    a = ease(t / 0.6)
    dx = 0.02 * (1 - a)
    fig.text(0.035 - dx, 0.905, f'{idx + 1:02d}', color=BLUE, fontsize=46, fontweight='black', alpha=a,
             va='center')
    fig.text(0.088 - dx, 0.918, title, color=TEXT, fontsize=36, fontweight='heavy', alpha=a, va='center')
    if subtitle:
        fig.text(0.088 - dx, 0.872, subtitle, color=MUTED, fontsize=19, alpha=a, va='center')
    progress(fig, idx)


def progress(fig, active):
    x0, w = 0.035, 0.93
    seg = w / len(CHAPTERS)
    for j, name in enumerate(CHAPTERS):
        col = BLUE if j == active else ('#3a4a60' if j > active else '#5f7ea8')
        fig.patches.append(Rectangle((x0 + j * seg + 0.002, 0.030), seg - 0.004, 0.005, transform=fig.transFigure,
                                     facecolor=col, edgecolor='none'))
        fig.text(x0 + j * seg + 0.002, 0.046, name, color=TEXT if j == active else MUTED, fontsize=11.5,
                 fontweight='bold' if j == active else 'normal')
    fig.text(0.965, 0.046, 'SCE-672  ·  PROJECT PHASE 3', color=MUTED, fontsize=11.5, ha='right')


def gauss(x, mu, s):
    return np.exp(-0.5 * ((x - mu) / s) ** 2) / (s * math.sqrt(2 * math.pi))


# ------------------------------------------------------------------ 1 title
_SC = {}


def s_title(k, n):
    from ep3d import EPScene
    if 'ep' not in _SC:
        _SC['ep'] = EPScene(width=1920, height=1080, eye=(0.9, -0.9, 0.55), target=(0, 0, 0.12), yfov_deg=30)
    sc = _SC['ep']
    t = k / FPS
    ang = -1.25 + 0.45 * k / n                     # slow orbit
    r = 1.20 - 0.08 * k / n                        # slow push-in
    right = np.array([-math.sin(ang), math.cos(ang), 0.0])
    tgt = np.array([0.0, 0.0, 0.12]) - 0.30 * right     # aim left of the robot: it sits in the right third
    eye = np.array([r * math.cos(ang), r * math.sin(ang), 0.50]) - 0.30 * right
    sc.set_camera(tuple(eye), tuple(tgt))
    img = sc.render(0.0, 0.0, 0.0, follow=False, decor=False)
    img = np.clip(img.astype(np.float32) * 1.30 + 4, 0, 255).astype(np.uint8)     # lift the dark model a little
    fig = plt.figure(figsize=(19.2, 10.8), dpi=100)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.imshow(img)
    ax.axis('off')
    # cinematic letterbox + gradient for the type
    for y0 in (0.0, 0.93):
        ax.add_patch(Rectangle((0, y0), 1, 0.07, transform=ax.transAxes, color='#000000'))
    grad = np.linspace(0, 0.88, 256)[:, None] ** 1.6 * np.ones((1, 4))
    ax.imshow(np.dstack([np.zeros((256, 4, 3)), grad[..., None]]), extent=(0, 1920, 1080, 560), aspect='auto')
    ax.set_xlim(0, 1920), ax.set_ylim(1080, 0)
    a1, a2, a3 = ease((t - 0.4) / 0.8), ease((t - 1.0) / 0.8), ease((t - 1.6) / 0.8)
    fig.text(0.06, 0.315, 'PROJECT PHASE 3', color=BLUE, fontsize=20, fontweight='bold', alpha=a1)
    fig.text(0.06 - 0.01 * (1 - a1), 0.235, 'RoboMaster Odometry', color=TEXT, fontsize=62, fontweight='black',
             alpha=a1)
    fig.text(0.06, 0.175, 'Predict with the wheels.  Correct with the camera.  Know where you are.', color=TEXT,
             fontsize=25, fontweight='light', alpha=a2)
    fig.text(0.06, 0.115, f'{AUTHOR}   ·   SCE-672 Advanced Robotics   ·   KFUPM   ·   Prof. Aamer Iqbal Bhatti',
             color=MUTED, fontsize=17, alpha=a3)
    return fig_img(fig)


# ------------------------------------------------------------------ 2 the task
SPEC = [('Detect a fixed tag', 'AprilTag ID 3, flat on the floor', 'LIVE RUN'),
        ('Drive straight to it, stop before it', 'start 1.20 m  ·  stop 0.50 m', 'LIVE RUN'),
        ('Predict with odometry, correct with a sensor', '5-state EKF  ·  wheels + camera', 'THE METHOD'),
        ('Repeat 8 to 10 times, same start', '10 trials from the same marks', 'RESULTS'),
        ('Error = estimated − measured;  MAE and range', 'tape from the same point on the robot', 'RESULTS'),
        ('Discuss consistency, error sources, roles', 'prediction vs correction, offline replay',
         'WHY CORRECT')]


def s_task(k, n):
    t = k / FPS
    fig = fig_new()
    chapter(fig, 0, t, 'The task', 'What the project asks for, and where this video shows it')
    # schematic (left)
    ax = fig.add_axes([0.04, 0.16, 0.42, 0.62])
    ax.set_xlim(-0.25, 1.55), ax.set_ylim(-0.75, 0.75), ax.set_aspect('equal'), ax.axis('off')
    ax.add_patch(Rectangle((-0.25, -0.62), 1.80, 1.24, facecolor='#0b121c', edgecolor=EDGE, lw=1.5))
    for gx in np.arange(-0.2, 1.55, 0.1):
        ax.plot([gx, gx], [-0.62, 0.62], color='#16212e', lw=0.8)
    ax.add_patch(Rectangle((1.36 - 0.10, -0.10), 0.20, 0.20, facecolor='white'))
    ax.add_patch(Rectangle((1.36 - 0.06, -0.06), 0.12, 0.12, facecolor='black'))
    ax.text(1.36, -0.17, 'tag', color=TEXT, ha='center', fontsize=13)
    p = ease((t - 1.2) / 3.2)
    xf = 0.16 + p * (0.86 - 0.16)
    ax.plot([0.16, 0.16], [-0.5, 0.5], color=MUTED, lw=2.5)
    ax.plot([0.86, 0.86], [-0.5, 0.5], color=PURPLE, lw=2.5, ls='--')
    ax.text(0.16, 0.54, 'START', color=MUTED, ha='center', fontsize=13, fontweight='bold')
    ax.text(0.86, 0.54, 'STOP', color=PURPLE, ha='center', fontsize=13, fontweight='bold')
    ax.annotate('', xy=(1.36, -0.40), xytext=(0.16, -0.40), arrowprops=dict(arrowstyle='<->', color=MUTED, lw=1.5))
    ax.text(0.76, -0.47, '1.20 m', color=TEXT, ha='center', va='top', fontsize=15)
    ax.annotate('', xy=(1.36, 0.36), xytext=(0.86, 0.36), arrowprops=dict(arrowstyle='<->', color=PURPLE, lw=1.5))
    ax.text(1.11, 0.40, '0.50 m', color=PURPLE, ha='center', va='bottom', fontsize=15)
    ax.add_patch(Rectangle((xf - 0.32, -0.12), 0.32, 0.24, facecolor='#3a4656', edgecolor=TEXT, lw=1.5))
    ax.plot([xf], [0], 'o', color=ORANGE, ms=9)
    ax.plot([0.16, xf - 0.16], [0, 0], color=BLUE, lw=3, alpha=0.8)
    a = ease((t - 4.4) / 0.6)
    ax.text(0.70, -0.70, 'graded:  e = d̂ (robot)  −  d (tape)', color=GOLD, ha='center', fontsize=17,
            fontweight='bold', alpha=a)
    # checklist (right)
    for j, (head, sub, where) in enumerate(SPEC):
        a = ease((t - 0.5 - 0.45 * j) / 0.5)
        y = 0.74 - j * 0.095
        box(fig, 0.50, y - 0.035, 0.455, 0.080, BLUE if a > 0.99 else EDGE, alpha=a, lw=1.4)
        fig.text(0.515, y + 0.018, '✓', color=GREEN, fontsize=24, fontweight='bold', alpha=a, va='center',
                 fontfamily='DejaVu Sans')
        fig.text(0.545, y + 0.020, head, color=TEXT, fontsize=17, fontweight='bold', alpha=a, va='center')
        fig.text(0.545, y - 0.013, sub, color=MUTED, fontsize=13.5, alpha=a, va='center')
        fig.text(0.948, y + 0.020, f'▸ {where}', color=BLUE, fontsize=11.5, ha='right', alpha=a, va='center',
                 fontfamily='DejaVu Sans')
    return fig_img(fig)


# ------------------------------------------------------------------ 3 the method (Gaussian intuition)
def s_method(k, n):
    t = k / FPS
    fig = fig_new()
    chapter(fig, 1, t, 'The method', 'An Extended Kalman Filter: two imperfect sources, one better estimate')
    ax = fig.add_axes([0.05, 0.17, 0.56, 0.60])
    ax.set_facecolor('none')
    for s_ in ax.spines.values():
        s_.set_visible(False)
    ax.spines['bottom'].set_visible(True), ax.spines['bottom'].set_color(EDGE)
    ax.tick_params(colors=MUTED, labelsize=13)
    ax.set_yticks([])
    x = np.linspace(0.70, 0.96, 600)
    ax.set_xlim(0.70, 0.96)
    ax.set_xlabel('distance to the tag (m)', color=MUTED, fontsize=14)
    # phase 1: prediction slides and widens
    p = ease((t - 0.8) / 2.4)
    mu_p, s_p = 0.885 - 0.085 * p, 0.007 + 0.019 * p
    ax.fill_between(x, gauss(x, mu_p, s_p), color=GREEN, alpha=0.20)
    ax.plot(x, gauss(x, mu_p, s_p), color=GREEN, lw=3)
    ax.text(mu_p + 1.1 * s_p, gauss(mu_p, mu_p, s_p) * 0.62, 'PREDICT', color=GREEN, ha='left', fontsize=17,
            fontweight='heavy')
    peaks = [gauss(mu_p, mu_p, s_p)]
    # phase 2: camera reading
    c = ease((t - 3.4) / 0.6)
    mu_c, s_c = 0.775, 0.020
    if c > 0:
        ax.fill_between(x, gauss(x, mu_c, s_c) * c, color=ORANGE, alpha=0.18)
        ax.plot(x, gauss(x, mu_c, s_c) * c, color=ORANGE, lw=3, alpha=c)
        ax.text(mu_c - 1.1 * s_c, gauss(mu_c, mu_c, s_c) * 0.62, 'CAMERA', color=ORANGE, ha='right', fontsize=17,
                fontweight='heavy', alpha=c)
        peaks.append(gauss(mu_c, mu_c, s_c) * c)
    # phase 3: fused
    f = ease((t - 4.8) / 0.8)
    kgain = s_p ** 2 / (s_p ** 2 + s_c ** 2)
    mu_f = mu_p + kgain * (mu_c - mu_p)
    s_f = math.sqrt((1 - kgain) * s_p ** 2)
    if f > 0:
        ax.fill_between(x, gauss(x, mu_f, s_f) * f, color=BLUE, alpha=0.28)
        ax.plot(x, gauss(x, mu_f, s_f) * f, color=BLUE, lw=4, alpha=f)
        ax.text(mu_f, gauss(mu_f, mu_f, s_f) * f * 1.04, 'EKF', color=BLUE, ha='center', va='bottom', fontsize=20,
                fontweight='black', alpha=f)
        peaks.append(gauss(mu_f, mu_f, s_f) * f)
    ax.set_ylim(0, 1.18 * max(peaks))
    if f <= 0:
        ax.set_ylim(0, 1.18 * max(peaks))
    # right column: meaning
    rows = [(0.8, GREEN, 'PREDICT  ·  wheels, 20× per second',
             'The wheels say how far we moved. Smooth and fast,\nbut slip and glitches add up: the curve widens.',
             r'$\hat{x} \leftarrow f(\hat{x},u)\qquad P \leftarrow FPF^{T}+Q$'),
            (3.4, ORANGE, 'CORRECT  ·  camera, 15× per second',
             'The camera measures the distance to the tag directly.\nIt never drifts, but each reading is noisy.',
             r'$z = [\,d,\ \varphi\,]$  (range and bearing to the tag)'),
            (4.8, BLUE, 'COMBINE  ·  Kalman gain',
             f'Each source is weighted by how sure it is: here the\ncamera gets {100 * kgain:.0f} %. Result: narrower than both.',
             r'$K = PH^{T}(HPH^{T}+R)^{-1}\qquad \hat{x} \leftarrow \hat{x}+K(z-h(\hat{x}))$')]
    for j, (t0, col, head, body, eq) in enumerate(rows):
        a = ease((t - t0) / 0.6)
        y = 0.70 - j * 0.20
        box(fig, 0.64, y - 0.11, 0.325, 0.17, col, alpha=a, lw=2, top=True)
        fig.text(0.655, y + 0.030, head, color=col, fontsize=16.5, fontweight='heavy', alpha=a)
        fig.text(0.655, y - 0.030, body, color=TEXT, fontsize=13.5, alpha=a, linespacing=1.35)
        fig.text(0.655, y - 0.090, eq, color=MUTED, fontsize=13, alpha=a)
    a = ease((t - 6.6) / 0.6)
    fig.text(0.05, 0.10, 'On the robot this loop runs every 50 ms:  predict with /odom (v, ω), correct with the tag '
             '(d, φ), stop when the EKF says 0.50 m.', color=TEXT, fontsize=16.5, alpha=a)
    return fig_img(fig)


# ------------------------------------------------------------------ live-run intro: how to read the dashboard
def s_live_intro(k, n):
    t = k / FPS
    fig = fig_new()
    chapter(fig, 2, t, 'Live run', 'One filmed trial. Every panel is drawn from the robot\'s own 20 Hz log, in sync with the phone')
    items = [(BLUE, 'Blue outline on the floor', 'where the EKF thinks the robot is, drawn on the real video'),
             (ORANGE, 'Orange flash', 'a camera reading arrives and corrects the estimate'),
             (GREEN, 'Green path', 'where the wheels alone think the robot is (no correction)'),
             (PURPLE, 'STOP line, 0.50 m', 'the robot brakes when the EKF reaches it'),
             (GOLD, 'Ruler', 'virtual ruler next to the real tape, 0 at the start, 120 cm at the tag')]
    for j, (col, head, body) in enumerate(items):
        a = ease((t - 0.4 - 0.35 * j) / 0.5)
        y = 0.72 - j * 0.115
        fig.patches.append(Rectangle((0.09, y - 0.022), 0.012, 0.05, transform=fig.transFigure, facecolor=col,
                                     edgecolor='none', alpha=a))
        fig.text(0.115, y + 0.012, head, color=TEXT, fontsize=21, fontweight='heavy', alpha=a, va='center')
        fig.text(0.115, y - 0.022, body, color=MUTED, fontsize=15.5, alpha=a, va='center')
    cards = [(GREEN, 'PREDICT', 'what the wheels say'), (ORANGE, 'CORRECT', 'what the camera sees'),
             (BLUE, 'ESTIMATE', 'the EKF\'s best guess ± how sure')]
    for j, (col, head, body) in enumerate(cards):
        a = ease((t - 2.2 - 0.3 * j) / 0.5)
        y = 0.66 - j * 0.16
        box(fig, 0.63, y - 0.05, 0.30, 0.12, col, alpha=a, top=True)
        fig.text(0.645, y + 0.025, head, color=col, fontsize=19, fontweight='heavy', alpha=a)
        fig.text(0.645, y - 0.020, body, color=TEXT, fontsize=16, alpha=a)
    fig.text(0.63, 0.80, 'The three number cards', color=MUTED, fontsize=15, alpha=ease((t - 2.0) / 0.5))
    return fig_img(fig)


# ------------------------------------------------------------------ 4 why correct (trial 6)
def s_why(k, n):
    if 'd6' not in _SC:
        _SC['d6'] = mv.load_trial(sorted(glob.glob(str(ROOT / 'results/trials/trial_06_*')))[-1])
    d = _SC['d6']
    T = d['t'][-1]
    replay = n - int(3.4 * FPS)
    tt = k / FPS
    t = min(k / replay, 1.0) * T
    i = mv.idx_at(d, t)
    fig = fig_new()
    chapter(fig, 3, tt, 'Why correct?', 'Trial 6 replayed: in this run the wheel odometry glitched 4 times')
    mv.draw_3d(fig.add_axes([0.035, 0.10, 0.56, 0.72]), d, t, i)
    mv.draw_distance(fig.add_axes([0.635, 0.43, 0.33, 0.38]), d, t, i)
    a = ease((k - replay) / (0.6 * FPS))
    meas = 0.488
    eo, ee = d['d_odom_only'][-1] - meas, d['d_est'][-1] - meas
    box(fig, 0.625, 0.155, 0.165, 0.20, GREEN, alpha=a, top=True)
    fig.text(0.7075, 0.315, 'WHEELS ALONE', color=GREEN, fontsize=15, fontweight='heavy', ha='center', alpha=a)
    fig.text(0.7075, 0.245, f'{100 * eo:+.1f} cm', color=TEXT, fontsize=40, fontweight='black', ha='center', alpha=a)
    fig.text(0.7075, 0.185, 'lost track after the glitches', color=MUTED, fontsize=13, ha='center', alpha=a)
    box(fig, 0.80, 0.155, 0.165, 0.20, BLUE, alpha=a, top=True)
    fig.text(0.8825, 0.315, 'EKF', color=BLUE, fontsize=15, fontweight='heavy', ha='center', alpha=a)
    fig.text(0.8825, 0.245, f'{100 * ee:+.1f} cm', color=TEXT, fontsize=40, fontweight='black', ha='center', alpha=a)
    fig.text(0.8825, 0.185, '176 camera fixes kept it true', color=MUTED, fontsize=13, ha='center', alpha=a)
    fig.text(0.795, 0.115, f'final error against the tape ({meas:.3f} m)', color=MUTED, fontsize=13, ha='center',
             alpha=a)
    return fig_img(fig)


# ------------------------------------------------------------------ 5 what each part contributes
def s_parts(k, n):
    t = k / FPS
    st = json.loads((ROOT / 'results/replay/replay.json').read_text())
    fig = fig_new()
    chapter(fig, 3, t, 'What each part contributes',
            'Same 10 trials, same logged data, same EKF code, replayed offline with one ingredient removed')
    bars = [('full', BLUE, 'Full EKF', 'prediction + camera range and bearing'),
            ('dropout_0.9', PURPLE, 'Camera lost for the last 40 cm', 'prediction carries the estimate alone'),
            ('bearing_only', GOLD, 'Camera angle only, no distance', 'bearing sees sideways error, not distance'),
            ('no_camera', GREEN, 'No camera while driving', 'prediction and wheel odometry only')]
    vmax = 2.8
    for j, (key, col, head, sub) in enumerate(bars):
        a = ease((t - 0.6 - 0.5 * j) / 0.5)
        g = ease((t - 0.8 - 0.5 * j) / 1.0)
        v = st[key]['mae_cm']
        y = 0.69 - j * 0.13
        fig.text(0.05, y + 0.012, head, color=TEXT, fontsize=19, fontweight='bold', alpha=a, va='center')
        fig.text(0.05, y - 0.030, sub, color=MUTED, fontsize=13.5, alpha=a, va='center')
        x0, wmax = 0.36, 0.50
        fig.patches.append(Rectangle((x0, y - 0.035), wmax, 0.07, transform=fig.transFigure, facecolor='#101a28',
                                     edgecolor='none', alpha=a))
        fig.patches.append(Rectangle((x0, y - 0.035), wmax * v / vmax * g, 0.07, transform=fig.transFigure,
                                     facecolor=col, edgecolor='none', alpha=0.95 * a))
        fig.text(x0 + wmax * v / vmax * g + 0.010, y, f'{v * g:.2f} cm', color=TEXT, fontsize=24, fontweight='black',
                 va='center', alpha=a, fontfamily=MONO)
    fig.text(0.36, 0.80, 'mean absolute error of the final distance', color=MUTED, fontsize=13)
    a = ease((t - 3.6) / 0.6)
    ratio = st['bearing_only']['mae_cm'] / st['full']['mae_cm']
    fig.text(0.05, 0.155, f'Remove the camera\'s distance reading and the error grows {ratio:.0f}×.', color=TEXT,
             fontsize=27, fontweight='heavy', alpha=a)
    fig.text(0.05, 0.105, 'Driving straight at the tag, the angle barely changes with distance, so only the range can '
             'fix the drift along the path (Lecture 12A).', color=MUTED, fontsize=16, alpha=a)
    return fig_img(fig)


# ------------------------------------------------------------------ 6 results
def load_results():
    rows = list(csv.DictReader(open(ROOT / 'results/summary_table.csv')))
    st = json.loads((ROOT / 'results/statistics.json').read_text())
    return rows, st


def s_results(k, n):
    if 'res' not in _SC:
        _SC['res'] = load_results()
    rows, st = _SC['res']
    t = k / FPS
    shown = min(10, int((t - 0.5) / 0.38) + 1) if t > 0.5 else 0
    fig = fig_new()
    chapter(fig, 4, t, 'Results  ·  10 trials',
            'Same start every time  ·  error = estimated − measured (tape, chassis front to tag centre)')
    ax = fig.add_axes([0.05, 0.36, 0.30, 0.44])
    mv.style_axes(ax, 'Estimate vs tape  (dashed = perfect)')
    ax.plot([0.465, 0.51], [0.465, 0.51], color=MUTED, ls='--', lw=1.2)
    ax.set_xlim(0.472, 0.503), ax.set_ylim(0.472, 0.503)
    ax.set_xlabel('measured by tape (m)', color=MUTED, fontsize=12)
    ax.set_ylabel('estimated by the robot (m)', color=MUTED, fontsize=12)
    ax.grid(color=EDGE, lw=0.6)
    for j, r in enumerate(rows[:shown]):
        ax.plot(float(r['d_meas_m']), float(r['d_est_m']), 'o', ms=13 if j == shown - 1 else 10, color=BLUE,
                mec='white', mew=1.2)
    ax2 = fig.add_axes([0.40, 0.36, 0.56, 0.44])
    mv.style_axes(ax2, 'Final error per trial (cm)')
    x = np.arange(1, 11)
    eo = np.array([100 * float(r['error_odom_m']) for r in rows])
    ee = np.array([100 * float(r['error_m']) for r in rows])
    m = x <= shown
    ax2.bar(x[m] - 0.2, eo[m], width=0.38, color=GREEN, label='wheels alone (prediction only)')
    ax2.bar(x[m] + 0.2, ee[m], width=0.38, color=BLUE, label='EKF (prediction + correction)')
    ax2.axhspan(-0.48, 0.48, color=BLUE, alpha=0.08)
    ax2.axhline(0, color=MUTED, lw=1)
    ax2.set_xlim(0.4, 10.6), ax2.set_ylim(-1.2, 6.5), ax2.set_xticks(x)
    ax2.grid(axis='y', color=EDGE, lw=0.6)
    ax2.set_xlabel('trial', color=MUTED, fontsize=12)
    ax2.legend(loc='upper left', fontsize=12.5, facecolor=PANEL, edgecolor=EDGE, labelcolor=TEXT)
    e, o = st['ekf'], st['odom']
    driven = 100 * (1.20 - 0.49)
    cards = [('MAE', f"{100 * e['mae']:.2f} cm", f"average miss · {e['mae'] * 100 / driven * 100:.1f} % of the "
              f"{driven:.0f} cm driven", BLUE),
             ('RANGE', f"{100 * e['range_of_errors']:.2f} cm", 'all 10 trials inside one centimetre', BLUE),
             ('BIAS', f"{100 * e['mean_error_bias']:+.2f} cm", 'no systematic offset', BLUE),
             ('σ', f"{100 * e['std_sample']:.2f} cm", 'same answer every run', BLUE),
             ('WHEELS ALONE', f"{100 * o['mae']:.2f} cm", f"{o['mae'] / e['mae']:.0f}× worse without the camera",
              GREEN)]
    for j, (lab, val, mean, col) in enumerate(cards):
        a = ease((t - 4.6 - 0.25 * j) / 0.6)
        x0 = 0.05 + j * 0.183
        box(fig, x0, 0.09, 0.172, 0.20, col, alpha=a, lw=2, top=True)
        fig.text(x0 + 0.086, 0.245, lab, color=col, fontsize=15, fontweight='heavy', ha='center', alpha=a)
        fig.text(x0 + 0.086, 0.170, val, color=TEXT, fontsize=36, fontweight='black', ha='center', alpha=a,
                 fontfamily=MONO)
        fig.text(x0 + 0.086, 0.115, mean, color=MUTED, fontsize=12.5, ha='center', alpha=a)
    return fig_img(fig)


# ------------------------------------------------------------------ 7 end
def s_end(k, n):
    t = k / FPS
    fig = fig_new()
    a1, a2, a3 = ease(t / 0.7), ease((t - 0.8) / 0.7), ease((t - 1.6) / 0.7)
    fig.text(0.5, 0.66, 'Prediction keeps the estimate smooth.', color=TEXT, fontsize=40, fontweight='heavy',
             ha='center', alpha=a1)
    fig.text(0.5, 0.585, 'Correction keeps it true.', color=BLUE, fontsize=40, fontweight='heavy', ha='center',
             alpha=a1)
    fig.text(0.5, 0.47, '0.28 cm average error with the EKF   ·   1.98 cm with the wheels alone   ·   10 / 10 trials',
             color=TEXT, fontsize=20, ha='center', alpha=a2)
    fig.text(0.5, 0.30, REPO, color=MUTED, fontsize=18, ha='center', alpha=a3, fontfamily=MONO)
    fig.text(0.5, 0.25, f'{AUTHOR}   ·   SCE-672 Advanced Robotics   ·   KFUPM', color=MUTED, fontsize=16,
             ha='center', alpha=a3)
    return fig_img(fig)


SCENES = {1: (s_title, 5.0), 2: (s_task, 7.5), 3: (s_method, 9.0), 8: (s_live_intro, 5.0), 4: (s_why, 10.0),
          5: (s_parts, 8.0), 6: (s_results, 11.0), 7: (s_end, 5.0)}
ORDER = ['scene1.mp4', 'scene2.mp4', 'scene3.mp4', 'scene8.mp4', 'live.mp4', 'scene4.mp4', 'scene5.mp4',
         'scene6.mp4', 'scene7.mp4']


def _job(args):
    sid, k, n, tmp = args
    import cv2
    img = SCENES[sid][0](k, n)
    cv2.imwrite(f'{tmp}/{k:05d}.png', cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
    return k


def build(sid, workers):
    import imageio_ffmpeg
    fn, dur = SCENES[sid]
    n = int(dur * FPS)
    tmp = str(OUT / f'scene{sid}_frames')
    os.makedirs(tmp, exist_ok=True)
    for f in glob.glob(tmp + '/*.png'):
        os.remove(f)
    with mp.get_context('spawn').Pool(workers) as pool:
        list(pool.imap_unordered(_job, [(sid, k, n, tmp) for k in range(n)], chunksize=4))
    out = str(OUT / f'scene{sid}.mp4')
    subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(), '-y', '-loglevel', 'error', '-framerate', str(FPS), '-i',
                    f'{tmp}/%05d.png', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '17', '-preset', 'slow', out],
                   check=True)
    print(out, flush=True)


def final_cut():
    """Concatenate with 0.5 s cross-fades; the live run (live.mp4 = scene3 of build_scene3.py) sits after the method."""
    import imageio_ffmpeg
    ff = imageio_ffmpeg.get_ffmpeg_exe()
    files = [OUT / f for f in ORDER]

    def dur(f):
        r = subprocess.run([ff, '-i', str(f)], capture_output=True, text=True).stderr
        hh, mm, ss = r.split('Duration: ')[1].split(',')[0].split(':')
        return int(hh) * 3600 + int(mm) * 60 + float(ss)
    xf = 0.5
    inputs, chains = [], []
    for j, f in enumerate(files):
        inputs += ['-i', str(f)]
        chains.append(f'[{j}:v]fps={FPS},scale={W}:{H},setsar=1,format=yuv420p[v{j}]')
    acc, last = 0.0, 'v0'
    for j in range(1, len(files)):
        acc += dur(files[j - 1]) - xf
        chains.append(f'[{last}][v{j}]xfade=transition=fade:duration={xf}:offset={acc:.3f}[x{j}]')
        last = f'x{j}'
    chains.append(f'[{last}]fade=t=in:st=0:d=0.6,fade=t=out:st={acc + dur(files[-1]) - 0.8:.3f}:d=0.8[final]')
    out = str(OUT / 'phase3_demo.mp4')
    subprocess.run([ff, '-y', '-loglevel', 'error', *inputs, '-filter_complex', ';'.join(chains), '-map', '[final]',
                    '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '17', '-preset', 'slow', '-movflags',
                    '+faststart', out], check=True)
    print(out, f'{acc + dur(files[-1]):.1f} s', flush=True)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--only', type=int, nargs='*')
    ap.add_argument('--workers', type=int, default=10)
    ap.add_argument('--final', action='store_true', help='only concatenate existing scenes')
    ap.add_argument('--still', type=float, nargs=2, metavar=('SCENE', 'T'))
    a = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    if a.still:
        sid, t = int(a.still[0]), a.still[1]
        fn, dur = SCENES[sid]
        img = fn(int(t * FPS), int(dur * FPS))
        p = OUT / f'still_s{sid}_{t:.1f}.png'
        plt.imsave(p, img)
        print(p)
        sys.exit()
    if not a.final:
        for sid in (a.only or sorted(SCENES)):
            build(sid, a.workers)
    if a.final or not a.only:
        final_cut()
