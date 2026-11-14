"""5-state Extended Kalman Filter for the RoboMaster EP Core (SCE-672 Project Phase 3).

Pure maths: no ROS, so it runs (and is unit tested) on Windows too.

    state        x = [px, py, theta, v, omega]   room frame, metres and radians
    input        u = [a, alpha]                   linear and angular acceleration
    odometry     z = [v, omega]                   from /odom            (Lecture 11B)
    camera tag   z = [d, phi]                     range and bearing     (Lecture 12A)

Every formula carries a tag like [EQ ...]. The number is the section of theory/theory_equations.md,
and theory/FORMULA_MAP.md lists where each tag sits in this file (python scripts/formula_lines.py).
"""
import numpy as np

PX, PY, TH, V, W = range(5)   # index of each state inside x
N_STATES = 5


def wrap(angle):
    """[EQ 4.WRAP] Bring any angle into (-pi, pi]: wrap(a) = atan2(sin a, cos a)."""
    return float(np.arctan2(np.sin(angle), np.cos(angle)))


# ---------------------------------------------------------------- prediction (Lecture 11A)

def motion_model(x, u, dt):
    """[EQ 2.f] Discrete unicycle with velocity states: x_{k+1} = f(x_k, u_k)."""
    px, py, th, v, w = x
    a, alpha = u
    return np.array([
        px + v * np.cos(th) * dt,     # move forward along the heading (x part)
        py + v * np.sin(th) * dt,     # move forward along the heading (y part)
        th + w * dt,                  # turn
        v + a * dt,                   # speed up / slow down
        w + alpha * dt,               # turn faster / slower
    ])


def jacobian_F(x, dt):
    """[EQ 2.F] State Jacobian df/dx, evaluated at x_k (before the step)."""
    th, v = x[TH], x[V]
    return np.array([
        [1.0, 0.0, -v * np.sin(th) * dt, np.cos(th) * dt, 0.0],
        [0.0, 1.0,  v * np.cos(th) * dt, np.sin(th) * dt, 0.0],
        [0.0, 0.0, 1.0, 0.0, dt],
        [0.0, 0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 1.0],
    ])


def jacobian_G(dt):
    """[EQ 2.G] Input Jacobian df/du: an acceleration only changes v and omega directly."""
    return np.array([
        [0.0, 0.0],
        [0.0, 0.0],
        [0.0, 0.0],
        [dt, 0.0],
        [0.0, dt],
    ])


# ---------------------------------------------------------------- measurements

# [EQ 3A.H_ODOM] /odom measures v and omega directly: pick rows 4 and 5 of the state.
H_ODOM = np.array([
    [0.0, 0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 1.0],
])


def h_odom(x):
    """[EQ 3A.h_odom] Predicted odometry reading: h_odom(x) = [v, omega]."""
    return H_ODOM @ x


def h_tag(x, tag_xy):
    """[EQ 3B.h] Predicted camera reading of the tag at (mx, my): range d and bearing phi."""
    dx = tag_xy[0] - x[PX]
    dy = tag_xy[1] - x[PY]
    d = np.hypot(dx, dy)                          # d = sqrt(dx^2 + dy^2)
    phi = wrap(np.arctan2(dy, dx) - x[TH])        # angle to the tag seen from the robot's nose
    return np.array([d, phi])


def jacobian_H_tag(x, tag_xy):
    """[EQ 3B.H_TAG] Measurement Jacobian dh/dx of the range-bearing model."""
    dx = tag_xy[0] - x[PX]
    dy = tag_xy[1] - x[PY]
    q = dx * dx + dy * dy
    d = np.sqrt(q)
    return np.array([
        [-dx / d, -dy / d, 0.0, 0.0, 0.0],        # range row: fixes the along-track distance
        [dy / q, -dx / q, -1.0, 0.0, 0.0],        # bearing row: fixes sideways error and heading
    ])


# ---------------------------------------------------------------- the filter

class EKF:
    """Predict with the motion model, correct with /odom and with the camera tag.

    tag_xy  : known room position (mx, my) of the tag; a constant, not a state.
    q_accel : variances (sigma_a^2, sigma_alpha^2) of the unknown accelerations, fed through G.
    q_pose  : extra variance per second on (px, py, theta) for wheel slip the model cannot see.
    gate    : chi-square limit on the tag innovation (9.21 = 99 % for 2 values); None = accept all.
    """

    def __init__(self, x0, P0, tag_xy, q_accel, q_pose, R_odom, R_tag, gate=9.21):
        self.x = np.array(x0, dtype=float)
        self.P = np.array(P0, dtype=float)
        self.tag_xy = (float(tag_xy[0]), float(tag_xy[1]))
        self.Q_u = np.diag(q_accel)
        self.q_pose = np.diag(list(q_pose) + [0.0, 0.0])
        self.R_odom = np.array(R_odom, dtype=float)
        self.R_tag = np.array(R_tag, dtype=float)
        self.gate = gate

    def predict(self, u, dt):
        """[EQ 2.PREDICT] x- = f(x, u);  P- = F P F^T + G Q_u G^T + Q_pose dt."""
        F = jacobian_F(self.x, dt)
        G = jacobian_G(dt)
        self.x = motion_model(self.x, u, dt)
        self.x[TH] = wrap(self.x[TH])
        self.P = F @ self.P @ F.T + G @ self.Q_u @ G.T + self.q_pose * dt

    def _correct(self, z, z_pred, H, R, angle_rows=(), gate=None):
        """[EQ 4] The five Kalman correction steps, shared by both sensors."""
        e = np.asarray(z, dtype=float) - z_pred           # 1. innovation (disagreement)
        for i in angle_rows:
            e[i] = wrap(e[i])                              #    angles: wrap, distances: never
        S = H @ self.P @ H.T + R                           # 2. innovation covariance
        S_inv = np.linalg.inv(S)
        nis = float(e @ S_inv @ e)                         #    how surprising the reading is
        if gate is not None and nis > gate:
            return {'accepted': False, 'e': e, 'S': S, 'K': None, 'nis': nis}
        K = self.P @ H.T @ S_inv                           # 3. Kalman gain
        self.x = self.x + K @ e                            # 4. state update
        self.x[TH] = wrap(self.x[TH])                      #    keep the heading in (-pi, pi]
        self.P = (np.eye(N_STATES) - K @ H) @ self.P       # 5. covariance update
        self.P = 0.5 * (self.P + self.P.T)                 #    remove rounding asymmetry
        return {'accepted': True, 'e': e, 'S': S, 'K': K, 'nis': nis}

    def update_odom(self, v, w):
        """[EQ 3A] Correct with /odom (v, omega). /odom already contains the IMU: never add /imu too."""
        return self._correct([v, w], h_odom(self.x), H_ODOM, self.R_odom)

    def update_tag(self, d, phi, use_range=True):
        """[EQ 3B] Correct with the camera: range d and bearing phi (or bearing only)."""
        z_pred = h_tag(self.x, self.tag_xy)
        H = jacobian_H_tag(self.x, self.tag_xy)
        if use_range:
            return self._correct([d, phi], z_pred, H, self.R_tag, angle_rows=(1,), gate=self.gate)
        gate = None if self.gate is None else 6.63          # 99 % for one value
        return self._correct([phi], z_pred[1:], H[1:], self.R_tag[1:, 1:], angle_rows=(0,), gate=gate)

    def distance_to_tag(self, front_offset=0.0):
        """Estimated distance from the chassis front to the tag, and its 1-sigma from P.

        d_front = |m - p| - front_offset.  Its gradient w.r.t. (px, py) is J = -[dx/d, dy/d],
        so var(d_front) = J P_xy J^T (first-order propagation of the covariance).
        """
        dx = self.tag_xy[0] - self.x[PX]
        dy = self.tag_xy[1] - self.x[PY]
        d = float(np.hypot(dx, dy))
        J = np.array([-dx / d, -dy / d])
        var = float(J @ self.P[:2, :2] @ J)
        return d - front_offset, float(np.sqrt(max(var, 0.0)))
