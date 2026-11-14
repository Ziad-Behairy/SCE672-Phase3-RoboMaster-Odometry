"""Checks on the filter maths. Runs without ROS:  cd src/rm_odom_ekf && python3 -m pytest -q test"""
import math

import numpy as np
import pytest

from rm_odom_ekf.ekf import (IDX_HEADING, IDX_SPEED, IDX_X, IDX_Y, ODOM_JACOBIAN, RobotEKF, expected_odom,
                             expected_tag_reading, input_jacobian, motion_jacobian, predict_state, tag_jacobian,
                             wrap_angle)

STEP = 1e-6                 # finite-difference step, as in Lec 11A slide 18
TAG = (1.66, 0.0)           # 1.50 m start + 0.16 m centre-to-front


def finite_diff_jacobian(fn, point, h=STEP):
    """Column j = (fn(p + h e_j) - fn(p - h e_j)) / 2h."""
    point = np.asarray(point, dtype=float)
    columns = []
    for j in range(len(point)):
        nudge = np.zeros_like(point)
        nudge[j] = h
        columns.append((np.asarray(fn(point + nudge)) - np.asarray(fn(point - nudge))) / (2 * h))
    return np.column_stack(columns)


def new_filter(start=(0, 0, 0, 0, 0)):
    return RobotEKF(initial_state=start, initial_cov=np.diag([1e-4, 1e-4, 1e-3, 1e-4, 1e-4]), tag_position=TAG,
                    accel_noise_var=(0.25, 1.0), slip_noise_rate=(1e-4, 1e-4, 1e-4),
                    odom_noise_cov=np.diag([0.02**2, 0.03**2]), tag_noise_cov=np.diag([0.03**2, 0.02**2]))


@pytest.mark.parametrize('state', [(0.1, -0.2, 0.3, 0.15, 0.05), (1.0, 0.5, -2.9, -0.1, -0.4), (0, 0, math.pi, 0.2, 0)])
def test_motion_jacobian_against_finite_differences(state):
    dt, accel = 0.1, (0.2, -0.1)
    numeric = finite_diff_jacobian(lambda s: predict_state(s, accel, dt), state)
    assert np.allclose(motion_jacobian(np.array(state), dt), numeric, atol=1e-7)


def test_input_jacobian_against_finite_differences():
    state, dt = np.array([0.1, -0.2, 0.3, 0.15, 0.05]), 0.1
    numeric = finite_diff_jacobian(lambda u: predict_state(state, u, dt), [0.2, -0.1])
    assert np.allclose(input_jacobian(dt), numeric, atol=1e-7)


@pytest.mark.parametrize('state', [(0.0, 0.0, 0.0, 0, 0), (0.7, 0.1, 0.2, 0, 0), (0.3, -0.4, -0.5, 0, 0)])
def test_tag_jacobian_against_finite_differences(state):
    numeric = finite_diff_jacobian(lambda s: expected_tag_reading(s, TAG), state)
    assert np.allclose(tag_jacobian(np.array(state, dtype=float), TAG), numeric, atol=1e-6)


def test_odom_model_reads_speed_and_turn_rate():
    state = np.array([1, 2, 3, 0.4, 0.5])
    assert np.allclose(expected_odom(state), [0.4, 0.5])
    assert ODOM_JACOBIAN.shape == (2, 5)


def test_wrap_stays_in_range():
    assert wrap_angle(0.0) == 0.0
    assert math.isclose(wrap_angle(3 * math.pi / 2), -math.pi / 2)
    assert math.isclose(abs(wrap_angle(math.pi)), math.pi)
    for a in np.linspace(-20, 20, 101):
        w = wrap_angle(a)
        assert -math.pi <= w <= math.pi
        assert math.isclose(math.sin(w), math.sin(a), abs_tol=1e-12)


def test_bearing_error_across_the_pi_line():
    """Tag nearly behind: predicted +179 deg, camera -179 deg. The correction must be about 2 deg, not 358."""
    ekf = new_filter()
    ekf.tag_position = (-1.0, 0.0001)
    ekf.outlier_limit = None
    predicted = expected_tag_reading(ekf.state, ekf.tag_position)[1]
    result = ekf.correct_with_tag(1.0, wrap_angle(predicted + math.radians(2)))
    assert abs(result['innovation'][1]) < math.radians(3)
    assert -math.pi <= ekf.state[IDX_HEADING] <= math.pi


def test_range_error_is_never_wrapped():
    ekf = new_filter()
    ekf.outlier_limit = None
    result = ekf.correct_with_tag(expected_tag_reading(ekf.state, TAG)[0] + 4.0, 0.0)
    assert math.isclose(result['innovation'][0], 4.0)


def test_corrections_reduce_uncertainty():
    ekf = new_filter()
    for _ in range(20):
        ekf.predict((0, 0), 0.1)
    before = np.trace(ekf.cov)
    ekf.correct_with_odom(0.0, 0.0)
    ekf.correct_with_tag(*expected_tag_reading(ekf.state, TAG))
    assert np.trace(ekf.cov) < before
    assert np.allclose(ekf.cov, ekf.cov.T)
    assert np.all(np.linalg.eigvalsh(ekf.cov) > 0)


def test_odom_moves_position_through_cross_covariance():
    """/odom only measures speed, but the x-speed term in P (filled by F) lets it shift x as well."""
    ekf = new_filter(start=(0, 0, 0, 0.1, 0))
    for _ in range(5):
        ekf.predict((0, 0), 0.1)
    assert ekf.cov[IDX_X, IDX_SPEED] > 0
    x_before = ekf.state[IDX_X]
    ekf.correct_with_odom(0.2, 0.0)
    assert ekf.state[IDX_SPEED] > 0.1 and ekf.state[IDX_X] > x_before


def test_bearing_alone_cannot_fix_along_track_error():
    """Lec 12A slide 26: facing the tag, bearing fixes the sideways error; only range fixes the forward one."""
    start_cov = np.diag([0.05**2, 0.05**2, 1e-3, 1e-4, 1e-4])
    bearing_only, with_range = new_filter(), new_filter()
    bearing_only.cov, with_range.cov = start_cov.copy(), start_cov.copy()
    reading = expected_tag_reading(bearing_only.state, TAG)
    bearing_only.correct_with_tag(reading[0], reading[1], use_range=False)
    with_range.correct_with_tag(reading[0], reading[1], use_range=True)
    assert bearing_only.cov[IDX_X, IDX_X] > 0.95 * 0.05**2
    assert bearing_only.cov[IDX_Y, IDX_Y] < 0.7 * 0.05**2
    assert with_range.cov[IDX_X, IDX_X] < 0.5 * 0.05**2


def test_outlier_reading_is_ignored():
    ekf = new_filter()
    before = ekf.state.copy()
    result = ekf.correct_with_tag(expected_tag_reading(ekf.state, TAG)[0] - 1.0, 0.0)
    assert not result['accepted']
    assert np.allclose(ekf.state, before)


def test_camera_pulls_back_wheel_slip():
    """Feed odometry that reads 8 % fast. The camera fixes should still give the right final distance."""
    rng = np.random.default_rng(0)
    dt, true_speed = 0.1, 0.12
    ekf = new_filter()
    true_x = 0.0
    while TAG[0] - true_x - 0.16 > 0.40:
        true_x += true_speed * dt
        ekf.predict((0, 0), dt)
        ekf.correct_with_odom(1.08 * true_speed + rng.normal(0, 0.01), rng.normal(0, 0.01))
        ekf.correct_with_tag(TAG[0] - true_x + rng.normal(0, 0.02), rng.normal(0, 0.01))
    d_est, sigma = ekf.distance_from_front(front_offset=0.16)
    assert abs(d_est - (TAG[0] - true_x - 0.16)) < 0.03
    assert 0 < sigma < 0.05
