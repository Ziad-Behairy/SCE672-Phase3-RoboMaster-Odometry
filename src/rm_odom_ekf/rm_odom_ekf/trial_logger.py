"""Per-trial log files.

Each run gets its own folder, e.g. results/trials/trial_03_20261120_141502/
    steps.csv      one row per control tick: commands, raw sensor values, filter state, full P, last K
    summary.json   the final numbers. d_meas and error stay null until add_measurement.py fills them in
"""
import csv
import json
import os
import time

STEP_COLUMNS = (
    ['t', 'mode', 'speed_cmd', 'turn_cmd',
     'odom_speed', 'odom_turn', 'odom_new',
     'tag_range', 'tag_bearing', 'tag_new', 'tag_used', 'tag_nis',
     'tof',
     'x', 'y', 'heading', 'speed', 'turn_rate',
     'd_est', 'sigma_d', 'd_odom_only']
    + [f'P{r}{c}' for r in range(5) for c in range(5)]     # covariance, row-major
    + [f'K{r}{c}' for r in range(5) for c in range(2)]     # gain of the last accepted camera fix
)


class TrialRecorder:

    def __init__(self, results_dir, trial_number, setup_info):
        started = time.strftime('%Y%m%d_%H%M%S')
        self.folder = os.path.join(os.path.expanduser(results_dir), f'trial_{int(trial_number):02d}_{started}')
        os.makedirs(self.folder, exist_ok=True)
        self.header_info = dict(setup_info, trial=int(trial_number), started=started)
        self._csv_file = open(os.path.join(self.folder, 'steps.csv'), 'w', newline='')
        self._writer = csv.writer(self._csv_file)
        self._writer.writerow(STEP_COLUMNS)
        self.closed = False

    def write_step(self, values):
        """values: dict keyed by STEP_COLUMNS. Anything missing is left blank."""
        row = []
        for column in STEP_COLUMNS:
            value = values.get(column, '')
            if isinstance(value, bool):
                value = int(value)
            elif isinstance(value, float):
                value = f'{value:.6g}'
            row.append(value)
        self._writer.writerow(row)

    def write_summary(self, final_values):
        """Close steps.csv and write summary.json. Returns the path of the summary."""
        if self.closed:
            return None
        self._csv_file.close()
        self.closed = True
        summary = dict(self.header_info)
        summary.update(final_values)
        summary.setdefault('d_meas', None)
        summary.setdefault('error', None)
        path = os.path.join(self.folder, 'summary.json')
        with open(path, 'w') as f:
            json.dump(summary, f, indent=2)
        return path

    def close(self):
        if not self.closed:
            self._csv_file.close()
            self.closed = True


def as_nested_list(matrix):
    """numpy array -> plain nested lists so json can store it."""
    return None if matrix is None else [[float(v) for v in row] for row in matrix]
