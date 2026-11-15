#!/usr/bin/env python3
"""Render the Phase 3 demo dashboard from a trial's logs, synced to the phone video.

Every panel is drawn from what the robot actually logged at 20 Hz (steps.csv, the launch log, the camera frames the
tag detector saved), so the dashboard moves exactly with the robot in the phone footage.

    # still frames to check the design (no phone video needed)
    python3 scripts/make_video.py --trial results/trials/trial_06_* --stills 1 4 7.4 11.5

    # the live-trial scene, synced to the phone take
    python3 scripts/make_video.py --trial results/video_run/trial_* --phone ~/Videos/phase3/run.mp4 \
        --frames results/video_run/frames --log <launch log> --tape 0.492 --out results/video/scene3.mp4

Panels: phone video | top view (EKF path, wheels-only path, 2-sigma ellipse, camera cone, camera fixes) |
EKF numbers grouped as PREDICT / CORRECT / ESTIMATE | robot's-eye camera | distance vs time | terminal.
"""
import argparse
import csv
import glob
import json
import math
import re
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt                      # noqa: E402
import numpy as np                                   # noqa: E402
from matplotlib.patches import Ellipse, FancyBboxPatch, Polygon, Rectangle   # noqa: E402

ROOT = Path(__file__).resolve().parents[1]

# ---------------------------------------------------------------- look
BG = '#070b12'
PANEL = '#0e1520'
EDGE = '#1c2735'
TEXT = '#eef3f8'
MUTED = '#8c99aa'
BLUE = '#4da3ff'        # EKF (prediction + correction)
GREEN = '#3ddc97'       # wheels only (prediction alone)
ORANGE = '#ffb547'      # camera (correction)
RED = '#ff5d6c'
PURPLE = '#b48cff'
STATE_COLOR = {'WAITING': '#ffcc4d', 'DRIVING': BLUE, 'HOLDING': PURPLE, 'DONE': GREEN}
SANS, MONO = 'Lato', 'Ubuntu Sans Mono'
plt.rcParams['font.family'] = ['Lato', 'DejaVu Sans']
plt.rcParams['mathtext.fontset'] = 'dejavusans'
ROBOT_L, ROBOT_W = 0.32, 0.24      # EP chassis footprint (m)


# ---------------------------------------------------------------- data
def load_trial(folder):
    folder = Path(folder)
    rows = list(csv.DictReader((folder / 'steps.csv').open()))
    summ = json.loads((folder / 'summary.json').read_text())

    def col(name):
        return np.array([float(r[name]) if r.get(name, '') not in ('', None) else np.nan for r in rows])

    d = {k: col(k) for k in ['t', 'speed_cmd', 'odom_speed', 'odom_turn', 'odom_new', 'tag_range', 'tag_bearing',
                             'tag_new', 'tag_used', 'tag_nis', 'x', 'y', 'heading', 'speed', 'turn_rate', 'd_est',
                             'sigma_d', 'd_odom_only', 'P00', 'P01', 'P11']}
    d['mode'] = np.array([r['mode'] for r in rows])
    d['summary'] = summ
    d['folder'] = folder
    start = summ['start_distance']
    # wheels-only path: travel from d_odom_only (exactly what the node integrated), heading from odom turn rate
    travel = start - d['d_odom_only']
    th = np.zeros_like(travel)
    xo, yo = np.zeros_like(travel), np.zeros_like(travel)
    w = np.nan_to_num(d['odom_turn'])
    for i in range(1, len(travel)):
        dt = d['t'][i] - d['t'][i - 1]
        th[i] = th[i - 1] + w[i] * dt
        ds = travel[i] - travel[i - 1]
        xo[i] = xo[i - 1] + ds * math.cos(th[i])
        yo[i] = yo[i - 1] + ds * math.sin(th[i])
    d['xo'], d['yo'] = xo, yo
    # running counts of camera fixes used / rejected
    d['n_used'] = np.cumsum(np.nan_to_num(d['tag_used']) == 1)
    d['n_rej'] = np.cumsum((d['tag_used'] == 0))
    d['tag_x'] = summ['params']['tag_position'][0]
    d['front'] = summ['front_offset']
    d['stop'] = summ['stop_distance']
    d['start'] = start
    # first time of each mode
    d['t_mode'] = {}
    for m in ['WAITING', 'DRIVING', 'HOLDING', 'DONE']:
        idx = np.where(d['mode'] == m)[0]
        if len(idx):
            d['t_mode'][m] = d['t'][idx[0]]
    return d


ANSI = re.compile(r'\x1b\[[0-9;]*m|\[0m|\[33m')


def load_log(path, t_start_unix):
    """Launch log -> [(t, text)] for the terminal panel."""
    if not path or not Path(path).exists():
        return []
    out, t0 = [], t_start_unix
    for line in Path(path).read_text(errors='ignore').splitlines():
        if 'phase3_ekf' not in line or 'frames/s' in line:
            continue
        line = ANSI.sub('', line).replace('[phase3_ekf-2] ', '')
        m = re.search(r'\[(\d{10}\.\d+)\]', line)
        if m:
            stamp = float(m.group(1))
            if t0 is None and 'start' in line and 'Trial' in line:
                t0 = stamp
            text = re.sub(r'^\[(INFO|WARN)\] \[\d+\.\d+\] \[phase3_ekf\]: ', r'[\1] ', line)
            out.append((stamp, text))
            continue
        m = re.match(r't=\s*([\d.]+)s\s+(.*)', line.strip())
        if m:
            out.append((None, f't={float(m.group(1)):5.1f}s {m.group(2)}', float(m.group(1))))
    lines = []
    for item in out:
        if item[0] is None:
            lines.append((item[2], item[1]))
        else:
            lines.append((item[0] - (t0 or item[0]), item[1]))
    lines.sort(key=lambda a: a[0])
    return lines


def load_frames(folder, t_start_unix):
    if not folder or not Path(folder).exists() or t_start_unix is None:
        return []
    files = sorted(Path(folder).glob('*.jpg'))
    return [(int(f.stem) * 1e-9 - t_start_unix, f) for f in files]


# ---------------------------------------------------------------- drawing helpers
def style_axes(ax, title=None, sub=None):
    ax.set_facecolor(PANEL)
    for s in ax.spines.values():
        s.set_color(EDGE)
    ax.tick_params(colors=MUTED, labelsize=9)
    if title:
        ax.set_title(title, loc='left', color=TEXT, fontsize=12, fontweight='bold', fontfamily=SANS, pad=6)
    if sub:
        ax.text(1.0, 1.012, sub, transform=ax.transAxes, ha='right', va='bottom', color=MUTED, fontsize=9)


def blank_panel(ax, title, msg):
    style_axes(ax, title)
    ax.set_xticks([]), ax.set_yticks([])
    ax.text(0.5, 0.5, msg, transform=ax.transAxes, ha='center', va='center', color=MUTED, fontsize=11)


def robot_polygon(x, y, th):
    c, s = math.cos(th), math.sin(th)
    pts = [(-ROBOT_L / 2, -ROBOT_W / 2), (ROBOT_L / 2, -ROBOT_W / 2), (ROBOT_L / 2, ROBOT_W / 2),
           (-ROBOT_L / 2, ROBOT_W / 2)]
    return [(x + c * px - s * py, y + s * px + c * py) for px, py in pts]


def idx_at(d, t):
    return int(np.clip(np.searchsorted(d['t'], t, side='right') - 1, 0, len(d['t']) - 1))


# ---------------------------------------------------------------- panels
def draw_header(fig, d, t, i):
    mode = d['mode'][i] if t >= 0 else 'WAITING'
    fig.text(0.010, 0.966, 'RoboMaster EP  ·  odometry with an Extended Kalman Filter', color=TEXT, fontsize=19,
             fontweight='bold', va='center')
    fig.text(0.010, 0.936, 'SCE-672 Advanced Robotics  ·  Phase 3  ·  live run, every panel drawn from the robot\'s own '
             '20 Hz log', color=MUTED, fontsize=12, va='center')
    col = STATE_COLOR.get(mode, MUTED)
    fig.patches.append(FancyBboxPatch((0.700, 0.937), 0.110, 0.042, boxstyle='round,pad=0.004,rounding_size=0.014',
                                      transform=fig.transFigure, facecolor=col, edgecolor='none'))
    fig.text(0.755, 0.958, mode, color='#0b0f14', fontsize=17, fontweight='bold', ha='center', va='center')
    trial = d['summary']['trial']
    label = 'demo run' if trial == 'demo' else f'trial {trial}'
    fig.text(0.990, 0.958, f'{label}   t = {max(t, 0):4.1f} s', color=TEXT, fontsize=15, ha='right',
             va='center', fontfamily=MONO)


_pct = ''


def draw_phone(ax, frame, caption, result=None, portrait=False):
    style_axes(ax, 'Real robot · AR overlay', 'phone camera')
    ax.set_xticks([]), ax.set_yticks([])
    if frame is not None:
        ax.imshow(frame, aspect='equal' if portrait else 'auto')
    else:
        ax.set_xlim(0, 1), ax.set_ylim(0, 1)
        ax.text(0.5, 0.55, 'phone video', transform=ax.transAxes, ha='center', color=MUTED, fontsize=16)
    if caption:
        step, txt = caption
        hb = 0.115 if portrait else 0.13
        ax.add_patch(Rectangle((0, 0), 1, hb, transform=ax.transAxes, facecolor='#000000', alpha=0.72,
                               edgecolor='none'))
        if portrait:
            if step:
                ax.text(0.04, hb * 0.70, step, transform=ax.transAxes, color=BLUE, fontsize=19, fontweight='bold',
                        va='center')
            ax.text(0.04, hb * 0.28 if step else hb * 0.5, txt, transform=ax.transAxes, color=TEXT, fontsize=12.5,
                    va='center', wrap=True)
        else:
            x0 = 0.02
            if step:
                ax.text(x0, 0.065, step, transform=ax.transAxes, color=BLUE, fontsize=18, fontweight='bold',
                        va='center')
                x0 += 0.025 * len(step) * 0.62 + 0.03
            ax.text(x0, 0.065, txt, transform=ax.transAxes, color=TEXT, fontsize=16, va='center')
    if result:
        if portrait:
            result = result.replace(' · error', '\nerror') + _pct
        ax.text(0.5, 0.90, result, transform=ax.transAxes, ha='center', va='center', color=GREEN,
                fontsize=17 if portrait else 22, fontweight='bold', linespacing=1.4,
                bbox=dict(boxstyle='round,pad=0.45', facecolor='#0f2a1d', edgecolor=GREEN, lw=2))


def draw_topview(ax, d, t, i):
    style_axes(ax, 'Top view', 'odometry frame (m)')
    tag_x, front = d['tag_x'], d['front']
    ax.set_xlim(-0.30, 1.55)
    ax.set_ylim(-0.45, 0.45)
    ax.set_aspect('equal', adjustable='box')
    ax.grid(color=EDGE, lw=0.6, alpha=0.6)
    # start / stop lines (front of the robot)
    for xl, lab, col in [(front, 'START 1.20 m', MUTED), (tag_x - d['stop'], 'STOP 0.50 m', PURPLE)]:
        ax.axvline(xl, color=col, ls='--', lw=1.2, alpha=0.8)
        ax.text(xl + 0.012, 0.40, lab, color=col, fontsize=9, va='top', fontweight='bold')
    # tag
    s = 0.154
    ax.add_patch(Rectangle((tag_x - s / 2, -s / 2), s, s, facecolor='#ffffff', edgecolor='#ffffff'))
    ax.add_patch(Rectangle((tag_x - s / 3, -s / 3), 2 * s / 3, 2 * s / 3, facecolor='#000000'))
    ax.text(tag_x, -s / 2 - 0.03, 'tag ID 3', color=TEXT, fontsize=9, ha='center', va='top')
    if t < 0:
        i = 0
    sl = slice(0, i + 1)
    # paths
    ax.plot(d['xo'][sl], d['yo'][sl], color=GREEN, lw=2.0, alpha=0.9, label='wheels only (prediction)')
    ax.plot(d['x'][sl], d['y'][sl], color=BLUE, lw=2.6, label='EKF (prediction + correction)')
    x, y, th = d['x'][i], d['y'][i], d['heading'][i]
    # camera cone from the chassis front
    cx, cy = x + front * math.cos(th), y + front * math.sin(th)
    half = math.radians(45)
    r = 0.55
    cone = [(cx, cy), (cx + r * math.cos(th + half), cy + r * math.sin(th + half)),
            (cx + r * math.cos(th - half), cy + r * math.sin(th - half))]
    ax.add_patch(Polygon(cone, closed=True, facecolor=ORANGE, alpha=0.07, edgecolor='none'))
    # camera fixes in the last 0.3 s: accepted (orange), rejected by the chi-square gate (red)
    for k in range(max(0, i - 6), i + 1):
        if d['tag_new'][k] == 1 and not np.isnan(d['tag_used'][k]):
            age = d['t'][i] - d['t'][k]
            ok = d['tag_used'][k] == 1
            ax.plot([cx, tag_x], [cy, 0], color=ORANGE if ok else RED, lw=2, alpha=max(0.0, 0.9 - 2.5 * age))
    # 2-sigma ellipse, exaggerated so a ~1 cm uncertainty is visible
    P = np.array([[d['P00'][i], d['P01'][i]], [d['P01'][i], d['P11'][i]]])
    if np.all(np.isfinite(P)):
        vals, vecs = np.linalg.eigh(P)
        vals = np.clip(vals, 1e-12, None)
        ang = math.degrees(math.atan2(vecs[1, 1], vecs[0, 1]))
        k = 10.0
        ax.add_patch(Ellipse((x, y), 2 * 2 * k * math.sqrt(vals[1]), 2 * 2 * k * math.sqrt(vals[0]), angle=ang,
                             facecolor=BLUE, alpha=0.18, edgecolor=BLUE, lw=1.2))
        ax.text(x, y - 0.20, '2σ ×10', color=BLUE, fontsize=8, ha='center', alpha=0.9)
    # robot
    ax.add_patch(Polygon(robot_polygon(x, y, th), closed=True, facecolor='#3a4656', edgecolor=TEXT, lw=1.4,
                         alpha=0.95))
    ax.annotate('', xy=(x + 0.20 * math.cos(th), y + 0.20 * math.sin(th)), xytext=(x, y),
                arrowprops=dict(arrowstyle='-|>', color=ORANGE, lw=2))
    ax.legend(loc='lower left', fontsize=8.5, facecolor=PANEL, edgecolor=EDGE, labelcolor=TEXT, framealpha=0.9)


_SCENE = None


def draw_3d(ax, d, t, i):
    global _SCENE
    import sys
    sys.path.insert(0, str(ROOT / 'video_tools'))
    from ep3d import EPScene
    if _SCENE is None:
        _SCENE = EPScene(width=1280, height=720, tag_x=d['tag_x'], front=d['front'], stop=d['stop'])
    if t < 0:
        i = 0
    P2 = np.array([[d['P00'][i], d['P01'][i]], [d['P01'][i], d['P11'][i]]])
    ray, ok = None, True
    for k in range(max(0, i - 6), i + 1):
        if d['tag_new'][k] == 1 and not np.isnan(d['tag_used'][k]):
            ray, ok = max(0.0, 0.9 - 2.5 * (d['t'][i] - d['t'][k])), d['tag_used'][k] == 1
    img = _SCENE.render(d['x'][i], d['y'][i], d['heading'][i], ekf_path=np.c_[d['x'][:i + 1], d['y'][:i + 1]],
                        odom_path=np.c_[d['xo'][:i + 1], d['yo'][:i + 1]], P2=P2, ray=ray, ray_ok=ok)
    style_axes(ax, '3D replay · real EP model', 'pose = EKF estimate')
    ax.imshow(img, aspect='auto')
    ax.set_xticks([]), ax.set_yticks([])
    ax.legend(handles=[plt.Line2D([], [], color=BLUE, lw=3, label='EKF path (predict + correct)'),
                       plt.Line2D([], [], color=GREEN, lw=3, label='wheels only (predict)'),
                       plt.Line2D([], [], color=ORANGE, lw=3, label='camera fix → correction'),
                       Ellipse((0, 0), 1, 1, facecolor=BLUE, alpha=0.3, edgecolor=BLUE, label='2σ uncertainty ×5')],
              loc='upper left', fontsize=9.5, facecolor=PANEL, edgecolor=EDGE, labelcolor=TEXT, framealpha=0.92)


def card(fig, x, y, w, h, title, color, lines):
    fig.patches.append(FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0.003,rounding_size=0.008',
                                      transform=fig.transFigure, facecolor=PANEL, edgecolor=color, lw=1.8))
    fig.text(x + 0.008, y + h - 0.010, title, color=color, fontsize=12.5, fontweight='heavy', va='top')
    yy = y + h - 0.043
    for label, value in lines:
        fig.text(x + 0.009, yy, label, color=MUTED, fontsize=12, va='top')
        fig.text(x + w - 0.009, yy, value, color=TEXT, fontsize=14.5, fontfamily=MONO, ha='right', va='top')
        yy -= 0.029


def draw_numbers(fig, d, t, i, tape, x=0.342, w=0.178):
    v, om = d['odom_speed'][i], d['odom_turn'][i]
    rng, brg = d['tag_range'][i], d['tag_bearing'][i]
    fmt = lambda val, f: '—' if val is None or not np.isfinite(val) else f.format(val)   # noqa: E731
    h = 0.122
    card(fig, x, 0.295, w, h, 'PREDICT  ·  what the wheels say', GREEN, [
        ('speed  v', fmt(v, '{:+.3f} m/s')),
        ('turn rate  ω', fmt(om, '{:+.2f} rad/s')),
        ('distance, wheels alone', fmt(d['d_odom_only'][i], '{:.3f} m'))])
    card(fig, x, 0.163, w, h, 'CORRECT  ·  what the camera sees', ORANGE, [
        ('distance to tag  d', fmt(rng, '{:.3f} m')),
        ('angle to tag  φ', fmt(math.degrees(brg) if np.isfinite(brg) else np.nan, '{:+.1f}°')),
        ('readings used / rejected', f"{int(d['n_used'][i])} / {int(d['n_rej'][i])}")])
    card(fig, x, 0.031, w, h, 'ESTIMATE  ·  EKF best guess', BLUE, [
        ('distance to tag  d̂', f"{d['d_est'][i]:.3f} m"),
        ('certainty  ±σ', f"{100 * d['sigma_d'][i]:.1f} cm"),
        ('heading  θ', f"{math.degrees(d['heading'][i]):+.1f}°")])


_DET = None


def _private(path):
    """Robot camera frame with only the floor corridor and the tag sharp; the room is blurred and darkened."""
    global _DET
    import cv2
    im = cv2.cvtColor(cv2.imread(str(path)), cv2.COLOR_BGR2RGB)
    h, w = im.shape[:2]
    if _DET is None:
        from pupil_apriltags import Detector
        _DET = Detector(families='tag36h11')
    tags = [r for r in _DET.detect(cv2.cvtColor(im, cv2.COLOR_RGB2GRAY)) if r.tag_id == 3]
    mask = np.zeros((h, w), np.uint8)
    if tags:
        c = tags[0].corners
        cx, cy = c.mean(0)
        r = max(np.ptp(c[:, 0]), np.ptp(c[:, 1])) * 0.9 + 12
        poly = np.array([[cx - r, cy - r * 0.6], [cx + r, cy - r * 0.6], [w * 0.80, h], [w * 0.20, h]], np.int32)
    else:
        poly = np.array([[w * 0.38, h * 0.55], [w * 0.62, h * 0.55], [w * 0.80, h], [w * 0.20, h]], np.int32)
    cv2.fillPoly(mask, [poly], 255)
    m = cv2.GaussianBlur(mask, (41, 41), 0).astype(np.float32)[:, :, None] / 255
    bg = cv2.GaussianBlur(im, (0, 0), 14) * 0.30
    return (im * m + bg * (1 - m)).astype(np.uint8)


def draw_eye(ax, frames, t, flash=0.0):
    if flash > 0:
        for sp in ax.spines.values():
            sp.set_color(ORANGE), sp.set_linewidth(1 + 3 * flash)
    if not frames:
        blank_panel(ax, "Robot's eye", 'robot camera frames\n(saved during the filmed run)')
        return
    k = max(0, np.searchsorted([f[0] for f in frames], t, side='right') - 1)
    img = _private(frames[k][1])
    style_axes(ax, "Robot's eye", 'green box = tag detected')
    ax.imshow(img, aspect='auto')
    if flash > 0:
        for sp in ax.spines.values():
            sp.set_color(ORANGE), sp.set_linewidth(1 + 3 * flash)
    ax.set_xticks([]), ax.set_yticks([])


def draw_distance(ax, d, t, i):
    style_axes(ax, 'Distance to tag', 'chassis front → tag centre')
    T = d['t'][-1]
    ax.set_xlim(0, T + 0.3)
    ax.set_ylim(0.40, 1.30)
    ax.grid(color=EDGE, lw=0.6, alpha=0.6)
    ax.axhline(d['stop'], color=PURPLE, ls='--', lw=1.2)
    ax.text(0.2, d['stop'] - 0.012, 'stop 0.50 m', color=PURPLE, fontsize=9, va='top')
    ax.set_xlabel('time (s)', color=MUTED, fontsize=9)
    ax.set_ylabel('m', color=MUTED, fontsize=9)
    if t < 0:
        return
    sl = slice(0, i + 1)
    tt = d['t'][sl]
    ax.fill_between(tt, d['d_est'][sl] - 2 * d['sigma_d'][sl], d['d_est'][sl] + 2 * d['sigma_d'][sl], color=BLUE,
                    alpha=0.25, lw=0)
    ax.plot(tt, d['d_odom_only'][sl], color=GREEN, lw=1.8, label='wheels only')
    cam = (d['tag_new'][sl] == 1)
    ax.scatter(tt[cam], d['tag_range'][sl][cam], s=9, color=ORANGE, zorder=3, label='camera fixes')
    ax.plot(tt, d['d_est'][sl], color=BLUE, lw=2.4, label='EKF ±2σ')
    ax.axvline(t, color=TEXT, lw=0.8, alpha=0.5)
    ax.legend(loc='upper right', fontsize=8.5, facecolor=PANEL, edgecolor=EDGE, labelcolor=TEXT, framealpha=0.9)


def _short(txt):
    m = re.match(r't=\s*([\d.]+)s (\w+)\s+d_est=([\d.]+)±([\d.]+) m\s+camera=([\d.]+) m.*', txt)
    if m:
        return f"t={float(m.group(1)):4.1f} {m.group(2):7s} d̂={m.group(3)}±{m.group(4)} cam={m.group(5)}"
    return txt.replace('[INFO] ', '').replace('Logging to ', 'log ')


def draw_terminal(ax, lines, t):
    ax.set_facecolor('#05080c')
    for s in ax.spines.values():
        s.set_color(EDGE)
    ax.set_xticks([]), ax.set_yticks([])
    ax.set_title('Terminal · phase3.launch.py', loc='left', color=TEXT, fontsize=12,
                 fontweight='bold', pad=6)
    shown = [_short(txt) for tt, txt in lines if tt <= t][-16:]
    y = 0.96
    for txt in shown:
        col = '#c9d1d9'
        if '->' in txt or 'done' in txt:
            col = '#7ee787'
        elif '[WARN]' in txt:
            col = '#f2cc60'
        ax.text(0.015, y, txt[:52], transform=ax.transAxes, color=col, fontsize=8.4, fontfamily=MONO, va='top')
        y -= 0.06


def caption_for(d, t, i):
    tm = d['t_mode']
    mode = d['mode'][i] if t >= 0 else 'WAITING'
    if mode == 'WAITING':
        return ('1 · DETECT', 'camera locks on tag ID 3 · 10 good readings before moving')
    if mode == 'DRIVING':
        since = t - tm.get('DRIVING', 0)
        if since < 2.0:
            return ('2 · PREDICT', 'wheels report the speed → estimate moves ahead')
        if d['speed_cmd'][i] < 0.145:
            return ('STRAIGHT LINE', 'slows down as it nears the stop · steers the tag dead ahead')
        return ('3 · CORRECT', 'camera measures the true distance 15× per second')
    if mode == 'HOLDING':
        return ('4 · STOP', 'EKF reaches 0.50 m → brake and hold 2 s')
    return ('5 · CHECK', 'independent tape measurement · error = estimated − measured')


# ---------------------------------------------------------------- frame
def render(d, t, phone_img=None, frames=None, log=None, tape=None, portrait=False):
    fig = plt.figure(figsize=(19.2, 10.8), dpi=100, facecolor=BG)
    i = idx_at(d, max(t, 0))
    draw_header(fig, d, t, i)
    result = None
    if tape is not None and d['mode'][i] == 'DONE':
        result = f"tape {tape:.3f} m · EKF {d['d_est'][i]:.3f} m · error {1000 * (d['d_est'][i] - tape):+.1f} mm"
        global _pct
        _pct = f"  ({100 * abs(d['d_est'][i] - tape) / (d['start'] - tape):.0f} % of the {100 * (d['start'] - tape):.0f} cm driven)"
    flash = 0.0
    for k in range(max(0, i - 4), i + 1):
        if d['tag_used'][k] == 1:
            flash = max(flash, 1.0 - 4.0 * (d['t'][i] - d['t'][k]))
    if portrait:
        draw_phone(fig.add_axes([0.008, 0.030, 0.290, 0.875]), phone_img, caption_for(d, t, i), result,
                   portrait=True)
        draw_3d(fig.add_axes([0.310, 0.470, 0.395, 0.435]), d, t, i)
        draw_eye(fig.add_axes([0.715, 0.470, 0.277, 0.435]), frames or [], t, flash)
        draw_numbers(fig, d, t, i, tape, x=0.310, w=0.170)
        draw_distance(fig.add_axes([0.525, 0.075, 0.215, 0.342]), d, t, i)
        draw_terminal(fig.add_axes([0.752, 0.031, 0.240, 0.386]), log or [], t)
    else:
        draw_phone(fig.add_axes([0.008, 0.455, 0.488, 0.452]), phone_img, caption_for(d, t, i), result)
        draw_3d(fig.add_axes([0.504, 0.455, 0.488, 0.452]), d, t, i)
        draw_eye(fig.add_axes([0.008, 0.031, 0.322, 0.386]), frames or [], t, flash)
        draw_numbers(fig, d, t, i, tape)
        draw_distance(fig.add_axes([0.552, 0.075, 0.215, 0.342]), d, t, i)
        draw_terminal(fig.add_axes([0.778, 0.031, 0.214, 0.386]), log or [], t)
    fig.canvas.draw()
    img = np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy()
    plt.close(fig)
    return img


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--trial', required=True, help='trial folder (glob ok)')
    ap.add_argument('--log', help='launch log of that run (terminal panel)')
    ap.add_argument('--frames', help='folder of camera frames saved by tag_detector')
    ap.add_argument('--tape', type=float, help='tape reading at the end (m)')
    ap.add_argument('--stills', type=float, nargs='*', help='render PNG stills at these data times (s)')
    ap.add_argument('--out', default=str(ROOT / 'results' / 'video'))
    a = ap.parse_args()
    folder = sorted(glob.glob(a.trial))[-1]
    d = load_trial(folder)
    t_start = d['summary'].get('t_start_unix')
    log = load_log(a.log, t_start)
    frames = load_frames(a.frames, t_start)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    if a.stills:
        for t in a.stills:
            img = render(d, t, frames=frames, log=log, tape=a.tape)
            p = out / f'still_t{t:05.1f}.png'
            plt.imsave(p, img)
            print(p)


if __name__ == '__main__':
    main()
