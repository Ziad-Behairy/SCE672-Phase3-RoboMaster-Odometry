"""Extended Kalman filter that tracks the RoboMaster while it drives toward the tag.

What the filter keeps track of (5 numbers, room frame):
    x, y        position of the robot centre in metres
    heading     where the nose points, radians, measured CCW from the room x axis
    speed       forward speed in m/s
    turn_rate   yaw rate in rad/s

It predicts with a simple unicycle model (Lec 11A) and corrects with two sensors:
    /odom   gives speed and turn rate                       (Lec 11B)
    camera  gives range and bearing to the tag              (Lec 12A)
Angles are folded back into (-pi, pi] wherever two angles get subtracted (Lec 12B).

No ROS imports here on purpose, so the maths can be tested on any laptop.
"""
import numpy as np

STATE_SIZE = 5
IDX_X, IDX_Y, IDX_HEADING, IDX_SPEED, IDX_TURN_RATE = range(STATE_SIZE)


def wrap_angle(angle):
    """Fold an angle into (-pi, pi]. atan2 of sin/cos does it without any if-branches (Lec 12B)."""
    return float(np.arctan2(np.sin(angle), np.cos(angle)))


# --------------------------------------------------------------------------- motion model (Lec 11A)

def predict_state(state, accel_input, dt):
    """Move the state forward by dt seconds: x_next = f(x, u), with u = (linear accel, angular accel)."""
    x, y, heading, speed, turn_rate = state
    linear_accel, angular_accel = accel_input
    return np.array([
        x + speed * np.cos(heading) * dt,
        y + speed * np.sin(heading) * dt,
        heading + turn_rate * dt,
        speed + linear_accel * dt,
        turn_rate + angular_accel * dt,
    ])


def motion_jacobian(state, dt):
    """F = df/dx, taken at the state *before* the step.

    The two entries that matter: a speed error becomes a position error (cos/sin column),
    and a heading error pushes the robot sideways (the speed*dt column).
    """
    heading, speed = state[IDX_HEADING], state[IDX_SPEED]
    cos_h, sin_h = np.cos(heading), np.sin(heading)
    return np.array([
        [1.0, 0.0, -speed * sin_h * dt, cos_h * dt, 0.0],
        [0.0, 1.0,  speed * cos_h * dt, sin_h * dt, 0.0],
        [0.0, 0.0, 1.0,                 0.0,        dt],
        [0.0, 0.0, 0.0,                 1.0,        0.0],
        [0.0, 0.0, 0.0,                 0.0,        1.0],
    ])


def input_jacobian(dt):
    """G = df/du. The accelerations only touch speed and turn rate directly."""
    jac = np.zeros((STATE_SIZE, 2))
    jac[IDX_SPEED, 0] = dt
    jac[IDX_TURN_RATE, 1] = dt
    return jac


# --------------------------------------------------------------------------- what the sensors should read

# /odom measures speed and turn rate straight off the state, so H_odom just picks those two rows (Lec 11B).
ODOM_JACOBIAN = np.array([
    [0.0, 0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 1.0],
])


def expected_odom(state):
    """h_odom(x) = [speed, turn_rate]."""
    return ODOM_JACOBIAN @ state


def expected_tag_reading(state, tag_position):
    """h(x) for the camera: range to the tag and bearing relative to the nose (Lec 12A)."""
    dx = tag_position[0] - state[IDX_X]
    dy = tag_position[1] - state[IDX_Y]
    tag_range = np.hypot(dx, dy)
    bearing = wrap_angle(np.arctan2(dy, dx) - state[IDX_HEADING])   # + means tag is to the left
    return np.array([tag_range, bearing])


def tag_jacobian(state, tag_position):
    """H_tag = dh/dx.

    Row 1 (range) depends on x, y only -> this is what fixes along-track drift.
    Row 2 (bearing) depends on x, y and heading -> fixes sideways error and heading.
    """
    dx = tag_position[0] - state[IDX_X]
    dy = tag_position[1] - state[IDX_Y]
    dist_sq = dx * dx + dy * dy                 # q in the lecture
    dist = np.sqrt(dist_sq)                     # d in the lecture
    return np.array([
        [-dx / dist,    -dy / dist,     0.0, 0.0, 0.0],
        [dy / dist_sq,  -dx / dist_sq, -1.0, 0.0, 0.0],
    ])


# --------------------------------------------------------------------------- the filter

class RobotEKF:
    """Holds the estimate (state + covariance) and runs predict / correct.

    tag_position     known (x, y) of the tag in the room. Fixed, so it is not part of the state.
    accel_noise_var  variance of the unknown accelerations, pushed into the state through G.
    slip_noise_rate  extra variance per second on x, y, heading for slip the model can't see.
    odom_noise_cov   R for /odom.      tag_noise_cov   R for the camera.
    outlier_limit    chi-square cut-off on a camera reading (9.21 = 99 % for 2 values). None turns it off.
    """

    def __init__(self, initial_state, initial_cov, tag_position, accel_noise_var, slip_noise_rate,
                 odom_noise_cov, tag_noise_cov, outlier_limit=9.21):
        self.state = np.array(initial_state, dtype=float)
        self.cov = np.array(initial_cov, dtype=float)
        self.tag_position = (float(tag_position[0]), float(tag_position[1]))
        self.accel_noise = np.diag(accel_noise_var)
        self.slip_noise = np.diag(list(slip_noise_rate) + [0.0, 0.0])
        self.odom_noise = np.array(odom_noise_cov, dtype=float)
        self.tag_noise = np.array(tag_noise_cov, dtype=float)
        self.outlier_limit = outlier_limit

    def predict(self, accel_input, dt):
        """Prediction step: push the state through the model and grow the covariance.

        P = F P F^T + G Q G^T + slip * dt
        """
        F = motion_jacobian(self.state, dt)      # must use the state before it moves
        G = input_jacobian(dt)
        self.state = predict_state(self.state, accel_input, dt)
        self.state[IDX_HEADING] = wrap_angle(self.state[IDX_HEADING])
        self.cov = F @ self.cov @ F.T + G @ self.accel_noise @ G.T + self.slip_noise * dt

    def _correct(self, measured, expected, H, noise, angle_rows=(), outlier_limit=None):
        """Standard EKF correction, shared by both sensors. Returns a small report for logging."""
        innovation = np.asarray(measured, dtype=float) - expected
        for row in angle_rows:
            innovation[row] = wrap_angle(innovation[row])     # only bearings; a distance is never wrapped

        innovation_cov = H @ self.cov @ H.T + noise           # S = H P H^T + R
        innovation_cov_inv = np.linalg.inv(innovation_cov)
        surprise = float(innovation @ innovation_cov_inv @ innovation)   # NIS, ~chi-square if all is well
        if outlier_limit is not None and surprise > outlier_limit:
            return {'accepted': False, 'innovation': innovation, 'gain': None, 'nis': surprise}

        gain = self.cov @ H.T @ innovation_cov_inv            # K = P H^T S^-1
        self.state = self.state + gain @ innovation
        self.state[IDX_HEADING] = wrap_angle(self.state[IDX_HEADING])
        self.cov = (np.eye(STATE_SIZE) - gain @ H) @ self.cov  # P = (I - K H) P
        self.cov = 0.5 * (self.cov + self.cov.T)                # float rounding makes P drift off symmetric
        return {'accepted': True, 'innovation': innovation, 'gain': gain, 'nis': surprise}

    def correct_with_odom(self, speed, turn_rate):
        """Use /odom speed and turn rate. /odom already has the IMU mixed in, so /imu is never added on top."""
        return self._correct([speed, turn_rate], expected_odom(self.state), ODOM_JACOBIAN, self.odom_noise)

    def correct_with_tag(self, tag_range, bearing, use_range=True):
        """Use the camera reading. With use_range=False only the bearing row is used (for comparison runs)."""
        expected = expected_tag_reading(self.state, self.tag_position)
        H = tag_jacobian(self.state, self.tag_position)
        if use_range:
            return self._correct([tag_range, bearing], expected, H, self.tag_noise,
                                 angle_rows=(1,), outlier_limit=self.outlier_limit)
        limit_1dof = None if self.outlier_limit is None else 6.63      # 99 % for a single value
        return self._correct([bearing], expected[1:], H[1:], self.tag_noise[1:, 1:],
                             angle_rows=(0,), outlier_limit=limit_1dof)

    def distance_from_front(self, front_offset=0.0):
        """Distance from the chassis front to the tag, plus its 1-sigma.

        The filter tracks the robot centre, but the tape is laid from the chassis front,
        so subtract the centre-to-front length. For the sigma, the distance changes with x, y
        by grad = -(dx, dy) / dist, so var = grad * P_xy * grad^T.
        """
        dx = self.tag_position[0] - self.state[IDX_X]
        dy = self.tag_position[1] - self.state[IDX_Y]
        centre_dist = float(np.hypot(dx, dy))
        grad = np.array([-dx / centre_dist, -dy / centre_dist])
        variance = float(grad @ self.cov[:2, :2] @ grad)
        return centre_dist - front_offset, float(np.sqrt(max(variance, 0.0)))
