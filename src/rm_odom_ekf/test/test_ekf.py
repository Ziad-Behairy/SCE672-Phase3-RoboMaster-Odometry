"""Checks for ekf.py. No robot or ROS needed:  python -m pytest -q src/rm_odom_ekf/test"""
import math

import numpy as np
import pytest

from rm_odom_ekf.ekf import (EKF, H_ODOM, PX, PY, TH, V, h_odom, h_tag, jacobian_F, jacobian_G,
                             jacobian_H_tag, motion_model, wrap)

H_STEP = 1e-6          # central-difference step from Lecture 11A, slide 18
TAG = (1.66, 0.0)


def numeric_jacobian(fun, x, h=H_STEP):
    """d fun / d x by central differences: (f(x + h e_j) - f(x - h e_j)) / 2h, one column per j."""
    x = np.asarray(x, dtype=float)
    cols = []
    for j in range(len(x)):
        e = np.zeros_like(x)
        e[j] = h
        cols.append((np.asarray(fun(x + e)) - np.asarray(fun(x - e))) / (2 * h))
    return np.column_stack(cols)


def make_ekf(x0=(0, 0, 0, 0, 0)):
    return EKF(x0=x0, P0=np.diag([1e-4, 1e-4, 1e-3, 1e-4, 1e-4]), tag_xy=TAG,
               q_accel=(0.25, 1.0), q_pose=(1e-4, 1e-4, 1e-4),
               R_odom=np.diag([0.02**2, 0.03**2]), R_tag=np.diag([0.03**2, 0.02**2]))


@pytest.mark.parametrize('x', [(0.1, -0.2, 0.3, 0.15, 0.05), (1.0, 0.5, -2.9, -0.1, -0.4), (0, 0, math.pi, 0.2, 0)])
def test_F_matches_central_differences(x):
    dt, u = 0.1, (0.2, -0.1)
    num = numeric_jacobian(lambda s: motion_model(s, u, dt), x)
    assert np.allclose(jacobian_F(np.array(x), dt), num, atol=1e-7)


def test_G_matches_central_differences():
    x, dt = np.array([0.1, -0.2, 0.3, 0.15, 0.05]), 0.1
    num = numeric_jacobian(lambda u: motion_model(x, u, dt), [0.2, -0.1])
    assert np.allclose(jacobian_G(dt), num, atol=1e-7)


@pytest.mark.parametrize('x', [(0.0, 0.0, 0.0, 0, 0), (0.7, 0.1, 0.2, 0, 0), (0.3, -0.4, -0.5, 0, 0)])
def test_H_tag_matches_central_differences(x):
    num = numeric_jacobian(lambda s: h_tag(s, TAG), x)
    assert np.allclose(jacobian_H_tag(np.array(x, dtype=float), TAG), num, atol=1e-6)


def test_H_odom_picks_v_and_omega():
    x = np.array([1, 2, 3, 0.4, 0.5])
    assert np.allclose(h_odom(x), [0.4, 0.5])
    assert H_ODOM.shape == (2, 5)


def test_wrap_range_and_boundary():
    assert wrap(0.0) == 0.0
    assert math.isclose(wrap(3 * math.pi / 2), -math.pi / 2)
    assert math.isclose(abs(wrap(math.pi)), math.pi)
    for a in np.linspace(-20, 20, 101):
        w = wrap(a)
        assert -math.pi <= w <= math.pi
        assert math.isclose(math.sin(w), math.sin(a), abs_tol=1e-12)


def test_bearing_innovation_is_wrapped_across_pi():
    """Tag almost straight behind: predicted bearing +179 deg, camera says -179 deg. Innovation must be -2 deg."""
    f = make_ekf(x0=(0, 0, 0, 0, 0))
    f.tag_xy = (-1.0, 0.0001)
    phi_pred = h_tag(f.x, f.tag_xy)[1]
    z_phi = wrap(phi_pred + math.radians(2))
    f.gate = None
    out = f.update_tag(1.0, z_phi)
    assert abs(out['e'][1]) < math.radians(3)          # not ~ 2*pi
    assert -math.pi <= f.x[TH] <= math.pi


def test_range_innovation_is_not_wrapped():
    f = make_ekf()
    f.gate = None
    out = f.update_tag(h_tag(f.x, TAG)[0] + 4.0, 0.0)    # 4 m disagreement stays 4 m (a length, not an angle)
    assert math.isclose(out['e'][0], 4.0)


def test_updates_shrink_uncertainty():
    f = make_ekf()
    for _ in range(20):
        f.predict((0, 0), 0.1)
    before = np.trace(f.P)
    f.update_odom(0.0, 0.0)
    f.update_tag(*h_tag(f.x, TAG))
    assert np.trace(f.P) < before
    assert np.allclose(f.P, f.P.T)
    assert np.all(np.linalg.eigvalsh(f.P) > 0)


def test_odom_corrects_position_through_cross_covariance():
    """Odometry only sees v, but P_xv (filled in by F) lets it move px too (Lecture 11B note)."""
    f = make_ekf(x0=(0, 0, 0, 0.1, 0))
    for _ in range(5):
        f.predict((0, 0), 0.1)
    assert f.P[PX, V] > 0
    px_before = f.x[PX]
    f.update_odom(0.2, 0.0)                              # robot is faster than we thought
    assert f.x[V] > 0.1 and f.x[PX] > px_before


def test_bearing_only_barely_fixes_along_track():
    """Lecture 12A slide 26: driving at the tag, bearing fixes sideways error, range fixes along-track error."""
    f = make_ekf()
    f.P = np.diag([0.05**2, 0.05**2, 1e-3, 1e-4, 1e-4])
    g = make_ekf()
    g.P = f.P.copy()
    z = h_tag(f.x, TAG)
    f.update_tag(z[0], z[1], use_range=False)
    g.update_tag(z[0], z[1], use_range=True)
    assert f.P[PX, PX] > 0.95 * 0.05**2                  # bearing only: along-track variance almost unchanged
    assert f.P[PY, PY] < 0.7 * 0.05**2                   # but sideways variance falls (shared with heading)
    assert g.P[PX, PX] < 0.5 * 0.05**2                   # with range: along-track variance falls too


def test_gate_rejects_wild_tag_reading():
    f = make_ekf()
    x_before = f.x.copy()
    out = f.update_tag(h_tag(f.x, TAG)[0] - 1.0, 0.0)    # 1 m off with 3 cm noise: rejected
    assert not out['accepted']
    assert np.allclose(f.x, x_before)


def test_simulated_run_keeps_final_distance_error_small():
    """Odometry over-reads speed by 8 % (wheel slip). Tag updates must pull the final distance back."""
    rng = np.random.default_rng(0)
    dt, v_true = 0.1, 0.12
    f = make_ekf()
    p_true = 0.0
    while TAG[0] - p_true - 0.16 > 0.40:
        p_true += v_true * dt
        f.predict((0, 0), dt)
        f.update_odom(1.08 * v_true + rng.normal(0, 0.01), rng.normal(0, 0.01))
        d_true = TAG[0] - p_true
        f.update_tag(d_true + rng.normal(0, 0.02), rng.normal(0, 0.01))
    d_est, sigma = f.distance_to_tag(front_offset=0.16)
    d_meas = TAG[0] - p_true - 0.16
    assert abs(d_est - d_meas) < 0.03
    assert 0 < sigma < 0.05
