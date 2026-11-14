import json
import math
from pathlib import Path

from rm_odom_ekf.controller import DONE, DRIVING, HOLDING, WAITING, ApproachController, DriveSettings, bearing_to
from rm_odom_ekf.trial_logger import STEP_COLUMNS, TrialRecorder


def run_for(ctrl, steps, t0, dt, **inputs):
    out, t = None, t0
    for _ in range(steps):
        out = ctrl.update(t, dt, **inputs)
        t += dt
    return out, t


def test_waits_for_camera_then_drives():
    ctrl = ApproachController(DriveSettings(fixes_before_start=5))
    cmd = ctrl.update(0.0, 0.05, distance_left=1.5, bearing=0.0, travelled=0.0, good_fixes=2)
    assert cmd == (0.0, 0.0) and ctrl.mode == WAITING
    ctrl.update(0.05, 0.05, distance_left=1.5, bearing=0.0, travelled=0.0, good_fixes=5)
    assert ctrl.mode == DRIVING


def test_speed_ramps_and_caps():
    cfg = DriveSettings(fixes_before_start=0)
    ctrl = ApproachController(cfg)
    ctrl.update(0.0, 0.05, 1.5, 0.0, 0.0, 0)
    first_speed, _ = ctrl.update(0.05, 0.05, 1.5, 0.0, 0.0, 0)
    assert math.isclose(first_speed, cfg.max_accel * 0.05)
    (speed, _), _ = run_for(ctrl, 200, 0.1, 0.05, distance_left=1.5, bearing=0.0, travelled=0.1, good_fixes=0)
    assert math.isclose(speed, cfg.max_speed)


def test_stops_at_the_set_distance_and_finishes():
    ctrl = ApproachController(DriveSettings(fixes_before_start=0, settle_time=1.0))
    run_for(ctrl, 10, 0.0, 0.05, distance_left=1.0, bearing=0.0, travelled=0.1, good_fixes=0)
    cmd = ctrl.update(1.0, 0.05, distance_left=0.399, bearing=0.0, travelled=1.0, good_fixes=0)
    assert cmd == (0.0, 0.0) and ctrl.mode == HOLDING and 'EKF' in ctrl.stop_reason
    run_for(ctrl, 25, 1.05, 0.05, distance_left=0.399, bearing=0.0, travelled=1.0, good_fixes=0)
    assert ctrl.mode == DONE


def test_safety_stops():
    for extra, word in [(dict(tof_range=0.10), 'ToF'), (dict(travelled=5.0), 'travelled')]:
        ctrl = ApproachController(DriveSettings(fixes_before_start=0))
        ctrl.update(0.0, 0.05, 1.0, 0.0, 0.0, 0)
        inputs = dict(distance_left=1.0, bearing=0.0, travelled=0.1, good_fixes=0)
        inputs.update(extra)
        ctrl.update(0.05, 0.05, **inputs)
        assert ctrl.mode == HOLDING and word in ctrl.stop_reason


def test_steers_toward_the_tag():
    ctrl = ApproachController(DriveSettings(fixes_before_start=0))
    ctrl.update(0.0, 0.05, 1.0, 0.0, 0.0, 0)
    _, turn = ctrl.update(0.05, 0.05, 1.0, bearing=1.0, travelled=0.0, good_fixes=0)
    assert turn == ctrl.cfg.max_turn_rate          # tag on the left -> turn left, capped


def test_bearing_wraps():
    assert math.isclose(bearing_to(0, 0, 0, (1, 0)), 0.0)
    # tag at -174.29 deg, nose at +170 deg: -344.29 deg must come out as +15.71 deg
    assert math.isclose(bearing_to(0, 0, math.radians(170), (-1, -0.1)), math.radians(15.71), abs_tol=1e-3)


def test_recorder_files(tmp_path):
    rec = TrialRecorder(tmp_path, 3, {'start_distance': 1.5})
    rec.write_step({'t': 0.0, 'mode': 'WAITING', 'x': 0.1234567, 'tag_used': True})
    path = rec.write_summary({'d_est_final': 0.41})
    lines = (Path(path).parent / 'steps.csv').read_text().splitlines()
    assert lines[0].split(',') == STEP_COLUMNS
    assert lines[1].split(',')[STEP_COLUMNS.index('x')] == '0.123457'
    summary = json.loads(Path(path).read_text())
    assert summary['trial'] == 3 and summary['d_meas'] is None and summary['d_est_final'] == 0.41
