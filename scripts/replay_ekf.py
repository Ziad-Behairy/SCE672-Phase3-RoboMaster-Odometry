#!/usr/bin/env python3
"""Offline replay of the EKF on the logged trials: what does each sensor contribute?

Every 50 ms tick of every trial is in steps.csv with the raw inputs the filter saw (commanded speed, /odom speed and
turn rate, camera range and bearing, whether the camera reading was offered). This script re-runs the SAME RobotEKF
code on those inputs, first exactly as on the robot (to check the replay reproduces the logged estimate), then with
one ingredient removed or changed, and compares the final error against the tape measurement of each trial.

    python3 scripts/replay_ekf.py          # writes results/replay/{replay_table.md, replay.json, fig_*.png/pdf}

Variants
  full           as run on the robot (prediction + /odom + camera range and bearing)
  bearing_only   camera bearing only, no range (Lecture 12A: bearing does not observe along-track distance)
  no_camera      camera used only before the robot moves (prediction + /odom only while driving)
  dropout_0.9    camera lost once the estimate passes 0.90 m (prediction carries the last 40 cm)
  camera_only    distance straight from the last camera reading (no filter): for comparison
  r_tag_x3       camera trusted 3x less (r_d = 9 cm)
  r_tag_div3     camera trusted 3x more (r_d = 1 cm)
"""
import csv
import glob
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'rm_odom_ekf'))
from rm_odom_ekf.ekf import RobotEKF, IDX_SPEED, IDX_TURN_RATE   # noqa: E402

VARIANTS = ['full', 'bearing_only', 'no_camera', 'dropout_0.9', 'camera_only', 'r_tag_x3', 'r_tag_div3']
LABEL = {'full': 'EKF as run (predict + odom + range + bearing)', 'bearing_only': 'EKF, camera bearing only',
         'no_camera': 'EKF, no camera while driving', 'dropout_0.9': 'EKF, camera lost below 0.90 m',
         'camera_only': 'camera only (last reading, no filter)', 'r_tag_x3': 'EKF, camera noise R x3',
         'r_tag_div3': 'EKF, camera noise R / 3'}


def f(x):
    return float(x) if x not in ('', None) else None


def load(folder):
    rows = list(csv.DictReader((folder / 'steps.csv').open()))
    summ = json.loads((folder / 'summary.json').read_text())
    return rows, summ, summ.get('d_meas')


def replay(rows, summ, variant):
    p = summ['params']
    r_d, r_phi = p['r_tag_d'], p['r_tag_phi']
    if variant == 'r_tag_x3':
        r_d, r_phi = 3 * r_d, 3 * r_phi
    if variant == 'r_tag_div3':
        r_d, r_phi = r_d / 3, r_phi / 3
    ekf = RobotEKF(np.zeros(5), np.diag(np.square(p['start_std'])), p['tag_position'],
                   (p['sigma_accel'] ** 2, p['sigma_alpha'] ** 2), (p['q_pos'], p['q_pos'], p['q_theta']),
                   np.diag([p['r_odom_v'] ** 2, p['r_odom_w'] ** 2]), np.diag([r_d ** 2, r_phi ** 2]),
                   outlier_limit=p['tag_gate'])
    front, tau = p['front_offset'], p['input_tau']
    use_range = variant != 'bearing_only'
    prev_t, cmd_v, cmd_w = None, 0.0, 0.0
    out, last_cam, driving = [], None, False
    for r in rows:
        t = float(r['t'])
        dt = 0.05 if prev_t is None else min(max(t - prev_t, 1e-3), 0.2)
        prev_t = t
        ekf.predict(((cmd_v - ekf.state[IDX_SPEED]) / tau, (cmd_w - ekf.state[IDX_TURN_RATE]) / tau), dt)
        if r['odom_new'] in ('1', 'True') and f(r['odom_speed']) is not None:
            ekf.correct_with_odom(f(r['odom_speed']), f(r['odom_turn']))
        d_now, _ = ekf.distance_from_front(front)
        driving = driving or r['mode'] != 'WAITING'
        offered = r['tag_used'] not in ('', None)            # a fresh camera reading reached the filter
        if offered:
            last_cam = f(r['tag_range'])
            allow = True
            if variant == 'no_camera' and driving:
                allow = False
            if variant == 'dropout_0.9' and d_now < 0.90:
                allow = False
            if allow and variant != 'camera_only':
                ekf.correct_with_tag(f(r['tag_range']) + front, f(r['tag_bearing']), use_range)
        d, s = ekf.distance_from_front(front)
        if variant == 'camera_only':
            d = last_cam if last_cam is not None else summ['start_distance']
        out.append((t, d, s))
        cmd_v, cmd_w = float(r['speed_cmd']), float(r['turn_cmd'])
    return np.array(out)


def main():
    trials = sorted(Path(p) for p in glob.glob(str(ROOT / 'results/trials/trial_*')))
    data = []
    for folder in trials:
        rows, summ, meas = load(folder)
        if meas is None:
            continue
        data.append((summ['trial'], rows, summ, meas))
    res = {v: [] for v in VARIANTS}
    traces = {}
    check = []
    for n, rows, summ, meas in data:
        d_meas = meas
        logged = np.array([float(r['d_est']) for r in rows])
        for v in VARIANTS:
            tr = replay(rows, summ, v)
            res[v].append(tr[-1, 1] - d_meas)
            traces[(n, v)] = tr
            if v == 'full':
                check.append(np.max(np.abs(tr[:, 1] - logged)))
    out = ROOT / 'results' / 'replay'
    out.mkdir(parents=True, exist_ok=True)
    stats = {}
    for v in VARIANTS:
        e = np.array(res[v])
        stats[v] = dict(label=LABEL[v], mae_cm=100 * np.mean(np.abs(e)), bias_cm=100 * np.mean(e),
                        std_cm=100 * np.std(e, ddof=1), range_cm=100 * (e.max() - e.min()),
                        max_abs_cm=100 * np.max(np.abs(e)), errors_cm=[round(100 * x, 3) for x in e])
    stats['_replay_check_max_diff_mm'] = 1000 * float(max(check))
    (out / 'replay.json').write_text(json.dumps(stats, indent=1))
    lines = ['| Variant | MAE (cm) | bias (cm) | σ (cm) | range (cm) | worst (cm) |', '|---|---:|---:|---:|---:|---:|']
    for v in VARIANTS:
        s = stats[v]
        lines.append(f"| {s['label']} | {s['mae_cm']:.2f} | {s['bias_cm']:+.2f} | {s['std_cm']:.2f} | "
                     f"{s['range_cm']:.2f} | {s['max_abs_cm']:.2f} |")
    (out / 'replay_table.md').write_text('\n'.join(lines) + '\n')
    print('\n'.join(lines))
    print(f"replay check: full replay vs logged d_est, max difference {stats['_replay_check_max_diff_mm']:.2f} mm")
    figures(traces, stats, data, out)


def figures(traces, stats, data, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.size': 13, 'axes.spines.top': False, 'axes.spines.right': False,
                         'font.family': ['Lato', 'DejaVu Sans'], 'axes.titlesize': 15, 'axes.titleweight': 'bold'})
    # bar chart of MAE per variant
    order = ['full', 'camera_only', 'r_tag_div3', 'r_tag_x3', 'dropout_0.9', 'bearing_only', 'no_camera']
    col = {'full': '#2A78D6', 'camera_only': '#EB6834', 'r_tag_div3': '#6BA6E8', 'r_tag_x3': '#6BA6E8',
           'dropout_0.9': '#7B3FA0', 'bearing_only': '#D4A017', 'no_camera': '#0B7A3E'}
    fig, ax = plt.subplots(figsize=(7.6, 3.4))
    y = np.arange(len(order))[::-1]
    for yy, v in zip(y, order):
        ax.barh(yy, stats[v]['mae_cm'], color=col[v], height=0.62)
        ax.text(stats[v]['mae_cm'] + 0.05, yy, f"{stats[v]['mae_cm']:.2f} cm", va='center', fontsize=9)
    ax.set_yticks(y, [stats[v]['label'] for v in order], fontsize=8.5)
    ax.set_xlabel('mean absolute final-distance error over the 10 trials (cm)')
    ax.grid(axis='x', alpha=0.3)
    fig.tight_layout()
    for ext in ('png', 'pdf'):
        fig.savefig(out / f'fig_replay_mae.{ext}', dpi=200)
    plt.close(fig)
    # trial 6: error and sigma vs time for full / no_camera / dropout
    n6 = 6
    d_meas = [m for n, r, s, m in data if n == n6][0]
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(12.5, 4.6))
    for v, c in [('no_camera', '#0B7A3E'), ('dropout_0.9', '#7B3FA0'), ('bearing_only', '#D4A017'), ('full', '#2A78D6')]:
        tr = traces[(n6, v)]
        a1.plot(tr[:, 0], tr[:, 1], color=c, lw=1.8, label=LABEL[v].replace('EKF, ', ''))
        a2.plot(tr[:, 0], 100 * tr[:, 2], color=c, lw=1.8)
    a1.axhline(d_meas, color='k', ls=':', lw=1)
    a1.text(0.3, d_meas + 0.012, f'tape {d_meas:.3f} m', fontsize=12)
    a1.set_xlabel('time (s)'), a1.set_ylabel('estimated distance d̂ (m)')
    a1.legend(fontsize=11.5, frameon=False)
    a2.set_xlabel('time (s)'), a2.set_ylabel('EKF 1σ of d̂ (cm)')
    a1.set_title('Trial 6: estimated distance', loc='left'), a2.set_title('Trial 6: how sure the filter is (1σ)', loc='left')
    for a in (a1, a2):
        a.grid(alpha=0.3)
    fig.tight_layout()
    for ext in ('png', 'pdf'):
        fig.savefig(out / f'fig_replay_trial6.{ext}', dpi=200)
    plt.close(fig)


if __name__ == '__main__':
    main()
