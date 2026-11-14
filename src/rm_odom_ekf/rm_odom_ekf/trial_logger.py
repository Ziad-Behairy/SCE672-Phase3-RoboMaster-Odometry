"""Trial diary: one folder per trial with every time step (steps.csv) and the final numbers (summary.json).

    results/trials/trial_03_20261120_141502/
        steps.csv      one row per control tick (20 Hz): commands, sensors, EKF state, P, K
        summary.json   final estimated distance, its sigma, final P, last K, counts, stop reason;
                       d_meas and error stay null until scripts/add_measurement.py fills them in

Pure Python (csv + json), no ROS.
"""
import csv
import json
import os
import time

STEP_COLUMNS = (
    ['t', 'state', 'v_cmd', 'w_cmd',
     'odom_v', 'odom_w', 'odom_new',                         # /odom reading and whether it was new this tick
     'tag_d', 'tag_phi', 'tag_new', 'tag_used', 'tag_nis',    # camera reading, accepted by the gate?, NIS
     'tof',
     'px', 'py', 'theta', 'v', 'w',                           # EKF state
     'd_est', 'sigma_d', 'd_odom_only']                       # chassis front -> tag: EKF, 1-sigma, odometry only
    + [f'P{i}{j}' for i in range(5) for j in range(5)]        # full 5x5 covariance
    + [f'K{i}{j}' for i in range(5) for j in range(2)]        # last tag Kalman gain (5x2), blank if none
)


class TrialLogger:

    def __init__(self, results_dir, trial, meta):
        stamp = time.strftime('%Y%m%d_%H%M%S')
        self.dir = os.path.join(os.path.expanduser(results_dir), f'trial_{int(trial):02d}_{stamp}')
        os.makedirs(self.dir, exist_ok=True)
        self.meta = dict(meta, trial=int(trial), started=stamp)
        self._file = open(os.path.join(self.dir, 'steps.csv'), 'w', newline='')
        self._csv = csv.writer(self._file)
        self._csv.writerow(STEP_COLUMNS)
        self.closed = False

    def log_step(self, values):
        """values: dict with any of STEP_COLUMNS. Floats are rounded, missing ones written blank."""
        row = []
        for col in STEP_COLUMNS:
            v = values.get(col, '')
            if isinstance(v, float):
                v = f'{v:.6g}'
            elif isinstance(v, bool):
                v = int(v)
            row.append(v)
        self._csv.writerow(row)

    def finish(self, summary):
        """Write summary.json (meta + summary) and close the CSV. Returns the summary path."""
        if self.closed:
            return None
        self._file.close()
        self.closed = True
        out = dict(self.meta)
        out.update(summary)
        out.setdefault('d_meas', None)    # filled in later by scripts/add_measurement.py
        out.setdefault('error', None)     # d_est - d_meas
        path = os.path.join(self.dir, 'summary.json')
        with open(path, 'w') as f:
            json.dump(out, f, indent=2)
        return path

    def close(self):
        if not self.closed:
            self._file.close()
            self.closed = True


def matrix_to_list(m):
    """numpy matrix -> nested list of floats for JSON (None stays None)."""
    return None if m is None else [[float(v) for v in row] for row in m]
