#!/usr/bin/env python3
"""Camera range calibration: chassis-front distance = range_scale * camera forward + range_offset.

Step 1, on the robot (driver + tag_detector running), once per tape distance (e.g. 0.40 0.70 1.00 1.30 1.60 m):
    python3 scripts/calibrate_range.py --tape 0.70          # records 5 s of tag/point, appends one CSV row
Step 2, anywhere (no ROS):
    python3 scripts/calibrate_range.py --fit                # least-squares line through all rows
    -> copy range_scale and range_offset into src/rm_odom_ekf/config/robot.yaml (tag_detector section)
"""
import argparse
import csv
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CSV = ROOT / 'results' / 'calibration' / 'tag_range.csv'
HEADER = ['wall_time', 'tape_m', 'forward_mean_m', 'forward_std_m', 'n_detections', 'secs']


def record(tape, secs):
    import numpy as np
    import rclpy
    from geometry_msgs.msg import PointStamped

    rclpy.init()
    node = rclpy.create_node('calibrate_range')
    fwd = []
    node.create_subscription(PointStamped, 'tag/point', lambda m: fwd.append(m.point.x), 10)
    t0 = time.time()
    while time.time() - t0 < secs:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()
    if not fwd:
        print(f'tape {tape:.3f} m: tag NOT DETECTED in {secs:.0f} s')
        return
    fwd = np.array(fwd)
    CSV.parent.mkdir(parents=True, exist_ok=True)
    new = not CSV.exists()
    with CSV.open('a', newline='') as f:
        w = csv.writer(f)
        if new:
            w.writerow(HEADER)
        w.writerow([time.strftime('%Y-%m-%dT%H:%M:%S'), f'{tape:.3f}', f'{fwd.mean():.4f}', f'{fwd.std():.4f}',
                    len(fwd), f'{secs:.1f}'])
    print(f'tape {tape:.3f} m: camera forward {fwd.mean():.4f} m (std {1000 * fwd.std():.1f} mm), '
          f'{len(fwd) / secs:.1f} detections/s -> {CSV}')


def fit():
    rows = list(csv.DictReader(CSV.open()))
    if len(rows) < 2:
        raise SystemExit(f'Need at least 2 rows in {CSV}')
    x = [float(r['forward_mean_m']) for r in rows]
    y = [float(r['tape_m']) for r in rows]
    n, mx, my = len(x), sum(x) / len(x), sum(y) / len(y)
    sxx = sum((a - mx) ** 2 for a in x)
    scale = sum((a - mx) * (b - my) for a, b in zip(x, y)) / sxx
    offset = my - scale * mx
    print(f'{n} points.  range_scale: {scale:.4f}   range_offset: {offset:.4f}')
    for a, b in zip(x, y):
        print(f'  tape {b:.3f} m   fitted {scale * a + offset:.3f} m   residual {100 * (scale * a + offset - b):+.2f} cm')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--tape', type=float, help='tape distance chassis front -> tag face (m)')
    ap.add_argument('--secs', type=float, default=5.0)
    ap.add_argument('--fit', action='store_true')
    args = ap.parse_args()
    if args.fit:
        fit()
    elif args.tape is not None:
        record(args.tape, args.secs)
    else:
        ap.print_help()


if __name__ == '__main__':
    main()
