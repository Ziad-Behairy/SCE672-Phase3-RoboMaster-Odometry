import json
import math
from pathlib import Path

from rm_odom_ekf.controller import (DONE, DRIVING, HOLDING, WAITING, ControllerParams, StraightLineController,
                                    bearing_error)
from rm_odom_ekf.trial_logger import STEP_COLUMNS, TrialLogger


def run(ctrl, n, t0, dt, **kw):
    out, t = None, t0
    for _ in range(n):
        out = ctrl.step(t, dt, **kw)
        t += dt
    return out, t


def test_waits_for_tag_then_drives():
    c = StraightLineController(ControllerParams(tag_settle=5))
    v, w = c.step(0.0, 0.05, d_est=1.5, heading_err=0.0, travelled=0.0, n_tag=2)
    assert (v, w) == (0.0, 0.0) and c.state == WAITING
    c.step(0.05, 0.05, d_est=1.5, heading_err=0.0, travelled=0.0, n_tag=5)
    assert c.state == DRIVING


def test_ramp_limits_acceleration_and_speed():
    p = ControllerParams(tag_settle=0)
    c = StraightLineController(p)
    c.step(0.0, 0.05, 1.5, 0.0, 0.0, 0)
    v1, _ = c.step(0.05, 0.05, 1.5, 0.0, 0.0, 0)
    assert math.isclose(v1, p.a_max * 0.05)
    (v, _), _ = run(c, 200, 0.1, 0.05, d_est=1.5, heading_err=0.0, travelled=0.1, n_tag=0)
    assert math.isclose(v, p.v_max)


def test_stops_at_stop_distance_then_done_after_hold():
    p = ControllerParams(tag_settle=0, hold_time=1.0)
    c = StraightLineController(p)
    run(c, 10, 0.0, 0.05, d_est=1.0, heading_err=0.0, travelled=0.1, n_tag=0)
    v, w = c.step(1.0, 0.05, d_est=0.399, heading_err=0.0, travelled=1.0, n_tag=0)
    assert (v, w) == (0.0, 0.0) and c.state == HOLDING and 'EKF' in c.stop_reason
    run(c, 25, 1.05, 0.05, d_est=0.399, heading_err=0.0, travelled=1.0, n_tag=0)
    assert c.state == DONE


def test_safety_brakes():
    for kw, word in [(dict(tof=0.10), 'ToF'), (dict(travelled=5.0), 'travelled')]:
        c = StraightLineController(ControllerParams(tag_settle=0))
        c.step(0.0, 0.05, 1.0, 0.0, 0.0, 0)
        args = dict(d_est=1.0, heading_err=0.0, travelled=0.1, n_tag=0)
        args.update(kw)
        c.step(0.05, 0.05, **args)
        assert c.state == HOLDING and word in c.stop_reason


def test_steering_sign_and_limit():
    c = StraightLineController(ControllerParams(tag_settle=0))
    c.step(0.0, 0.05, 1.0, 0.0, 0.0, 0)
    _, w = c.step(0.05, 0.05, 1.0, heading_err=1.0, travelled=0.0, n_tag=0)
    assert w == c.p.w_max                       # tag on the left -> turn left (CCW), capped


def test_bearing_error_wraps():
    assert math.isclose(bearing_error(0, 0, 0, (1, 0)), 0.0)
    # tag at atan2(-0.1, -1) = -174.29 deg, nose at +170 deg: -344.29 deg wraps to +15.71 deg (not -344)
    assert math.isclose(bearing_error(0, 0, math.radians(170), (-1, -0.1)), math.radians(15.71), abs_tol=1e-3)


def test_logger_writes_csv_and_summary(tmp_path):
    log = TrialLogger(tmp_path, 3, {'start_distance': 1.5})
    log.log_step({'t': 0.0, 'state': 'WAITING', 'px': 0.1234567, 'tag_used': True})
    path = log.finish({'d_est_final': 0.41})
    lines = (Path(path).parent / 'steps.csv').read_text().splitlines()
    assert lines[0].split(',') == STEP_COLUMNS
    assert lines[1].split(',')[STEP_COLUMNS.index('px')] == '0.123457'
    s = json.load(open(path))
    assert s['trial'] == 3 and s['d_meas'] is None and s['d_est_final'] == 0.41
