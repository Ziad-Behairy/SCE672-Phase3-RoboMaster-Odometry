#!/usr/bin/env python3
"""Type in the tape-measured final distance of a trial (chassis front -> tag face, metres).

    python3 scripts/add_measurement.py 3 0.412            # newest folder of trial 3
    python3 scripts/add_measurement.py results/trials/trial_03_20261120_141502 0.412

Writes into that trial's summary.json:
    d_meas       the tape distance
    error        d_est_final - d_meas             (EKF: prediction + correction)   <- the graded number
    error_cam    d_cam_final - d_meas             (camera alone, for the discussion)
    error_odom   d_odom_only_final - d_meas       (odometry alone, for the discussion)
No ROS needed.
"""
import argparse
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def find_trial(which, trials_dir):
    p = Path(which)
    if p.is_dir():
        return p
    folders = sorted(Path(trials_dir).glob(f'trial_{int(which):02d}_*'))
    if not folders:
        raise SystemExit(f'No folder trial_{int(which):02d}_* in {trials_dir}')
    return folders[-1]                     # newest run of that trial number


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('trial', help='trial number or trial folder')
    ap.add_argument('d_meas', type=float, help='tape distance chassis front -> tag (m)')
    ap.add_argument('--trials-dir', default=str(ROOT / 'results' / 'trials'))
    args = ap.parse_args()

    folder = find_trial(args.trial, args.trials_dir)
    path = folder / 'summary.json'
    summary = json.loads(path.read_text())
    summary['d_meas'] = args.d_meas
    summary['error'] = summary['d_est_final'] - args.d_meas
    summary['error_cam'] = None if summary.get('d_cam_final') is None else summary['d_cam_final'] - args.d_meas
    summary['error_odom'] = None if summary.get('d_odom_only_final') is None else summary['d_odom_only_final'] - args.d_meas
    path.write_text(json.dumps(summary, indent=2))
    print(f'{folder.name}: d_est = {summary["d_est_final"]:.4f} m, d_meas = {args.d_meas:.4f} m, '
          f'error = {summary["error"] * 100:+.2f} cm' + ('' if summary.get('valid', True) else '   (marked INVALID)'))


if __name__ == '__main__':
    main()
