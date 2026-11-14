#!/usr/bin/env python3
"""Print the current line number of each formula in the code, to keep theory/FORMULA_MAP.md up to date.

    python3 scripts/formula_lines.py
"""
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PKG = 'src/rm_odom_ekf/rm_odom_ekf/'

# (what, file, text that starts the line)
LANDMARKS = [
    ('state indices', PKG + 'ekf.py', r'^IDX_X, IDX_Y'),
    ('wrap_angle', PKG + 'ekf.py', r'^def wrap_angle'),
    ('motion model f', PKG + 'ekf.py', r'^def predict_state'),
    ('F', PKG + 'ekf.py', r'^def motion_jacobian'),
    ('G', PKG + 'ekf.py', r'^def input_jacobian'),
    ('H_odom', PKG + 'ekf.py', r'^ODOM_JACOBIAN'),
    ('h_odom', PKG + 'ekf.py', r'^def expected_odom'),
    ('h_tag (range, bearing)', PKG + 'ekf.py', r'^def expected_tag_reading'),
    ('H_tag', PKG + 'ekf.py', r'^def tag_jacobian'),
    ('predict: P = F P F^T + ...', PKG + 'ekf.py', r'\s+def predict'),
    ('innovation e = z - h', PKG + 'ekf.py', r'\s+innovation = '),
    ('bearing innovation wrap', PKG + 'ekf.py', r'\s+innovation\[row\] = wrap_angle'),
    ('S = H P H^T + R', PKG + 'ekf.py', r'\s+innovation_cov = '),
    ('outlier gate (NIS)', PKG + 'ekf.py', r'\s+surprise = '),
    ('K = P H^T S^-1', PKG + 'ekf.py', r'\s+gain = self.cov'),
    ('x = x + K e', PKG + 'ekf.py', r'\s+self.state = self.state \+ gain'),
    ('P = (I - K H) P', PKG + 'ekf.py', r'\s+self.cov = \(np.eye'),
    ('odom correction', PKG + 'ekf.py', r'\s+def correct_with_odom'),
    ('tag correction', PKG + 'ekf.py', r'\s+def correct_with_tag'),
    ('distance from chassis front + sigma', PKG + 'ekf.py', r'\s+def distance_from_front'),
    ('node: read /odom', PKG + 'phase3_node.py', r'\s+def on_odom'),
    ('node: tick', PKG + 'phase3_node.py', r'\s+def tick'),
    ('node: 1) predict', PKG + 'phase3_node.py', r'\s+# 1\) predict'),
    ('node: 2) odometry', PKG + 'phase3_node.py', r'\s+# 2\) correct'),
    ('node: 3) camera', PKG + 'phase3_node.py', r'\s+# 3\) correct'),
    ('node: 4) control', PKG + 'phase3_node.py', r'\s+# 4\) control'),
    ('node: 5) log', PKG + 'phase3_node.py', r'\s+# 5\) log'),
    ('error = d_est - d_meas', 'scripts/add_measurement.py', r"\s+summary\['error'\] ="),
]

for what, rel_path, pattern in LANDMARKS:
    lines = (ROOT / rel_path).read_text(encoding='utf-8').splitlines()
    hits = [n for n, line in enumerate(lines, 1) if re.match(pattern, line)]
    where = f'{rel_path}:{hits[0]}' if hits else f'{rel_path}: NOT FOUND'
    print(f'{what:38s} {where}')
