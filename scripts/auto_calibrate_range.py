#!/usr/bin/env python3
"""Automatic camera range calibration: the robot drives itself to each calibration distance.

Only two tape readings are needed: the start distance and the final distance. Between them, the distance
travelled comes from /odom (straight-line distance from the start pose), rescaled so the two tape readings match
exactly. This corrects any wheel-odometry scale error.

Step 1 (driver + tag_detector running, robot on the start line facing the tag, path clear):
    python3 scripts/auto_calibrate_range.py drive --start 1.20
    -> stops at 1.00, 0.80, 0.60, 0.50 m (nominal), records 5 s of tag/point at each stop and at the start,
       saves results/calibration/auto_run.json
Step 2: tape the final distance (chassis front -> tag centre), then
    python3 scripts/auto_calibrate_range.py finish --final 0.502
    -> writes results/calibration/tag_range.csv (same format as calibrate_range.py) and prints the fit
       (python3 scripts/calibrate_range.py --fit)

While driving it steers toward the tag (turn = steer_gain * bearing, like the controller), so the camera
forward distance and the diagonal tape distance stay the same thing. The last stop must still see the tag.

Safety: 0.05 m/s, cmd_vel at 10 Hz (the driver's chassis_timeout stops the robot if this script dies), ToF stop,
time limit per leg, and an abort if the camera range grows instead of shrinking (wrong drive_sign).
"""
import argparse
import csv
import json
import math
import statistics
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RUN = ROOT / 'results' / 'calibration' / 'auto_run.json'
CSV = ROOT / 'results' / 'calibration' / 'tag_range.csv'
HEADER = ['wall_time', 'tape_m', 'forward_mean_m', 'forward_std_m', 'n_detections', 'secs']


def drive(args):
    import rclpy
    from geometry_msgs.msg import PointStamped, Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Range

    rclpy.init()
    node = rclpy.create_node('auto_calibrate_range')
    state = {'pose': None, 'tof': None, 'fwd': [], 'fwd_t': [], 'brg': []}

    def on_odom(m):
        p = m.pose.pose.position
        state['pose'] = (p.x, p.y)

    def on_tag(m):
        state['fwd'].append(m.point.x)
        state['brg'].append(math.atan2(m.point.y, m.point.x))
        state['fwd_t'].append(time.time())

    def on_tof(m):
        state['tof'] = m.range

    node.create_subscription(Odometry, 'odom', on_odom, 10)
    node.create_subscription(PointStamped, 'tag/point', on_tag, 10)
    node.create_subscription(Range, 'range_0', on_tof, 10)
    pub = node.create_publisher(Twist, 'cmd_vel', 10)

    def spin(secs):
        t0 = time.time()
        while time.time() - t0 < secs:
            rclpy.spin_once(node, timeout_sec=0.02)

    def stop():
        for _ in range(5):
            pub.publish(Twist())
            spin(0.05)

    def travelled(start):
        x, y = state['pose']
        return math.hypot(x - start[0], y - start[1])

    def record(label):
        spin(1.0)                      # let the chassis settle
        n0 = len(state['fwd'])
        spin(args.secs)
        fwd = state['fwd'][n0:]
        brg = state['brg'][n0:]
        row = {'label': label, 'odom_m': travelled(start), 'n': len(fwd),
               'forward_mean_m': statistics.mean(fwd) if fwd else None,
               'forward_std_m': statistics.pstdev(fwd) if len(fwd) > 1 else None,
               'bearing_deg': math.degrees(statistics.mean(brg)) if brg else None}
        txt = (f"camera forward {row['forward_mean_m']:.4f} m (std {1000 * (row['forward_std_m'] or 0):.1f} mm), "
               f"bearing {row['bearing_deg']:+.1f} deg"
               if fwd else 'tag NOT DETECTED')
        print(f"[{label}] odom {row['odom_m']:.4f} m, {len(fwd) / args.secs:.1f} det/s, {txt}", flush=True)
        return row

    spin(2.0)
    if state['pose'] is None:
        raise SystemExit('No /odom. Is the driver running?')
    start = state['pose']
    rows = [record(f'start {args.start:.2f}')]
    if rows[0]['n'] == 0:
        raise SystemExit('Tag not seen at the start: fix the view before driving.')
    start_fwd = rows[0]['forward_mean_m']

    try:
        for target in args.targets:
            goal = args.start - target
            print(f'-> driving to nominal {target:.2f} m (odom {goal:.3f} m from start)', flush=True)
            t0 = time.time()
            cmd = Twist()
            cmd.linear.x = args.drive_sign * args.speed
            checked = target != args.targets[0]
            while travelled(start) < goal - 0.005:
                if state['tof'] is not None and 0.0 < state['tof'] < args.tof_stop:
                    raise RuntimeError(f"ToF {state['tof']:.3f} m < {args.tof_stop} m")
                if time.time() - t0 > goal / args.speed + 15.0:
                    raise RuntimeError('time limit for this leg')
                if not checked and travelled(start) > 0.04:
                    recent = [f for f, t in zip(state['fwd'], state['fwd_t']) if t > time.time() - 0.5]
                    if recent:
                        checked = True
                        if statistics.mean(recent) > start_fwd + 0.01:
                            raise RuntimeError('camera range INCREASES: robot drives away from the tag '
                                               '(flip drive_sign)')
                recent = [b for b, t in zip(state['brg'], state['fwd_t']) if t > time.time() - 0.3]
                turn = args.steer_gain * recent[-1] if recent else 0.0
                cmd.angular.z = max(-args.max_turn, min(args.max_turn, turn))
                pub.publish(cmd)
                spin(0.1)
            stop()
            rows.append(record(f'nominal {target:.2f}'))
    except (RuntimeError, KeyboardInterrupt) as e:
        stop()
        print(f'ABORTED: {e}', flush=True)
    finally:
        stop()

    RUN.parent.mkdir(parents=True, exist_ok=True)
    RUN.write_text(json.dumps({'start_tape_m': args.start, 'time': time.strftime('%Y-%m-%dT%H:%M:%S'),
                               'secs': args.secs, 'rows': rows}, indent=2))
    print(f'Saved {RUN}. Now tape the final distance and run: '
          f'python3 scripts/auto_calibrate_range.py finish --final 0.xxx', flush=True)
    node.destroy_node()
    rclpy.shutdown()


def finish(args):
    run = json.loads(RUN.read_text())
    rows = run['rows']
    start, last = run['start_tape_m'], rows[-1]
    scale = (start - args.final) / last['odom_m']
    print(f'odom scale correction: {scale:.4f} ({100 * (scale - 1):+.2f} %)')
    new = not CSV.exists()
    with CSV.open('a', newline='') as f:
        w = csv.writer(f)
        if new:
            w.writerow(HEADER)
        for r in rows:
            tape = start - scale * r['odom_m']
            if r['forward_mean_m'] is None:
                print(f"  {r['label']}: tape {tape:.4f} m, no detection (skipped)")
                continue
            w.writerow([run['time'], f'{tape:.4f}', f"{r['forward_mean_m']:.4f}",
                        f"{r['forward_std_m'] or 0:.4f}", r['n'], f"{run['secs']:.1f}"])
            print(f"  {r['label']}: tape {tape:.4f} m, camera forward {r['forward_mean_m']:.4f} m")
    print(f'Appended to {CSV}. Fit: python3 scripts/calibrate_range.py --fit')


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    d = sub.add_parser('drive')
    d.add_argument('--start', type=float, required=True, help='tape at the start (m)')
    d.add_argument('--targets', type=float, nargs='+', default=[1.00, 0.80, 0.60, 0.50])
    d.add_argument('--speed', type=float, default=0.05)
    d.add_argument('--drive-sign', type=float, default=-1.0, help='same as drive_sign in robot.yaml')
    d.add_argument('--secs', type=float, default=5.0)
    d.add_argument('--tof-stop', type=float, default=0.20)
    d.add_argument('--steer-gain', type=float, default=1.0, help='rad/s per rad of bearing')
    d.add_argument('--max-turn', type=float, default=0.30, help='rad/s')
    f = sub.add_parser('finish')
    f.add_argument('--final', type=float, required=True, help='tape at the last stop (m)')
    a = ap.parse_args()
    drive(a) if a.cmd == 'drive' else finish(a)
