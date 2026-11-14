#!/usr/bin/env python3
"""SIMULATED trials, for testing the pipeline on Windows before the robot is available.

Runs the real ekf.py, controller.py and trial_logger.py against a fake robot with the problems the real one
has: wheel slip (odometry over-reads speed by ~6 %), a lagging speed loop, a slightly crooked start, noisy and
sometimes missing camera readings. Writes the same files a real trial writes, to results/_demo/trials
(ignored by git). Never mix these with real results: every summary has "simulated": true.

    python scripts/simulate_trials.py --n 10
    python scripts/analyze_trials.py --trials results/_demo/trials --out results/_demo
"""
import argparse
import math
import shutil
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'rm_odom_ekf'))
from rm_odom_ekf.controller import DONE, HOLDING, ControllerParams, StraightLineController  # noqa: E402
from rm_odom_ekf.ekf import EKF, PX, PY, TH, V, W, h_tag  # noqa: E402
from rm_odom_ekf.trial_logger import TrialLogger, matrix_to_list  # noqa: E402

START, STOP, FRONT = 1.50, 0.40, 0.16          # same defaults as config/robot.yaml
DT, TAU = 0.05, 0.3


def simulate(trial, out_dir, rng):
    tag = (START + FRONT, 0.0)
    # true robot: slightly misplaced and crooked at the start
    s = np.array([rng.normal(0, 0.01), rng.normal(0, 0.01), rng.normal(0, 0.02), 0.0, 0.0])
    slip = rng.normal(0.06, 0.02)               # odometry over-reads speed by this fraction
    cam_bias = rng.normal(0.0, 0.008)           # calibration left-over, per trial

    ekf = EKF(x0=np.zeros(5), P0=np.diag(np.square([0.02, 0.02, 0.035, 0.01, 0.01])), tag_xy=tag,
              q_accel=(0.5 ** 2, 1.0 ** 2), q_pose=(1e-4, 1e-4, 1e-4),
              R_odom=np.diag([0.02 ** 2, 0.03 ** 2]), R_tag=np.diag([0.03 ** 2, 0.02 ** 2]))
    cp = ControllerParams(stop_distance=STOP)
    cp.max_travel = START - STOP + 0.15
    ctrl = StraightLineController(cp)
    log = TrialLogger(out_dir, trial, {'start_distance': START, 'stop_distance': STOP, 'front_offset': FRONT,
                                       'simulated': True})
    s_odom, n_odom, n_tag, n_rej, last_K, hold_cam, w_cmd = 0.0, 0, 0, 0, None, [], 0.0
    t, next_cam = 0.0, 0.0
    for k in range(int(60 / DT)):
        t = k * DT
        # --- true robot moves (speed loop lag TAU, small heading wobble)
        s[V] += (ctrl.v_cmd - s[V]) / TAU * DT
        s[W] += (w_cmd - s[W]) / TAU * DT + rng.normal(0, 0.002)
        s[PX] += s[V] * math.cos(s[TH]) * DT
        s[PY] += s[V] * math.sin(s[TH]) * DT
        s[TH] += s[W] * DT
        row = {'t': t}
        # --- EKF exactly as in phase3_node.tick()
        ekf.predict(((ctrl.v_cmd - ekf.x[V]) / TAU, (w_cmd - ekf.x[W]) / TAU), DT)
        if k % 2 == 0:                                              # /odom at 10 Hz
            v_o = s[V] * (1 + slip) + rng.normal(0, 0.005)
            w_o = s[W] + rng.normal(0, 0.01)
            s_odom += v_o * 2 * DT
            ekf.update_odom(v_o, w_o)
            n_odom += 1
            row.update(odom_v=v_o, odom_w=w_o, odom_new=True)
        if t >= next_cam:                                           # camera ~12 Hz, 10 % dropouts
            next_cam = t + 1 / 12
            if rng.random() > 0.10:
                z_true = h_tag(s, tag)
                d_front = z_true[0] - FRONT + cam_bias + rng.normal(0, 0.015)
                phi = z_true[1] + rng.normal(0, 0.01)
                res = ekf.update_tag(d_front + FRONT, phi)
                row.update(tag_d=d_front, tag_phi=phi, tag_new=True, tag_used=res['accepted'], tag_nis=res['nis'])
                if res['accepted']:
                    n_tag += 1
                    last_K = res['K']
                    for i in range(5):
                        for j in range(2):
                            row[f'K{i}{j}'] = float(last_K[i, j])
                else:
                    n_rej += 1
                if ctrl.state == HOLDING:
                    hold_cam.append(d_front)
        d_est, sig = ekf.distance_to_tag(FRONT)
        v, w_cmd = ctrl.step(t, DT, d_est, h_tag(ekf.x, tag)[1], math.hypot(ekf.x[PX], ekf.x[PY]), n_tag)
        x = ekf.x
        row.update(state=ctrl.state, v_cmd=v, w_cmd=w_cmd, px=x[PX], py=x[PY], theta=x[TH], v=x[V], w=x[W],
                   d_est=d_est, sigma_d=sig, d_odom_only=START - s_odom)
        for i in range(5):
            for j in range(5):
                row[f'P{i}{j}'] = float(ekf.P[i, j])
        log.log_step(row)
        if ctrl.state == DONE:
            break

    d_est, sig = ekf.distance_to_tag(FRONT)
    d_true = h_tag(s, tag)[0] - FRONT
    d_meas = round(d_true + rng.normal(0, 0.002), 3)               # tape, read to the millimetre
    d_cam = float(np.median(hold_cam)) if hold_cam else None
    path = log.finish({
        'valid': ctrl.stop_reason.startswith('EKF'), 'stop_reason': ctrl.stop_reason,
        'd_est_final': d_est, 'sigma_d_final': sig, 'd_cam_final': d_cam, 'd_odom_only_final': START - s_odom,
        'x_final': [float(v) for v in ekf.x], 'P_final': matrix_to_list(ekf.P), 'K_last_tag': matrix_to_list(last_K),
        'n_odom_updates': n_odom, 'n_tag_updates': n_tag, 'n_tag_rejected': n_rej,
        'd_meas': d_meas, 'error': d_est - d_meas, 'error_cam': None if d_cam is None else d_cam - d_meas,
        'error_odom': START - s_odom - d_meas})
    print(f'sim trial {trial:2d}: d_est {d_est:.3f} ± {sig:.3f} m, tape {d_meas:.3f} m, '
          f'error {100 * (d_est - d_meas):+.2f} cm ({ctrl.stop_reason})')
    return path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=10)
    ap.add_argument('--seed', type=int, default=672)
    ap.add_argument('--out', default=str(ROOT / 'results' / '_demo' / 'trials'))
    args = ap.parse_args()
    out = Path(args.out)
    if out.exists():
        shutil.rmtree(out)
    rng = np.random.default_rng(args.seed)
    for trial in range(1, args.n + 1):
        simulate(trial, out, rng)


if __name__ == '__main__':
    main()
