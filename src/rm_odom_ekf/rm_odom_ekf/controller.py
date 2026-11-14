"""Straight-line drive toward the tag with a clean stop. Pure Python (no ROS), unit tested.

States:
  WAITING  robot still. Waits until the tag has been seen tag_settle times (the EKF locks on to the start
           position meanwhile). Gives up after wait_timeout.
  DRIVING  v ramps up to v_max, then slows down in proportion to the distance left. A small steering term
           keeps the nose pointed at the tag (the bearing comes from the EKF estimate, not the raw camera,
           so it still works when the camera loses the tag).
  HOLDING  v = 0, omega = 0. The robot settles; the EKF keeps running so the final estimate is taken at rest.
  DONE     trial finished; the node writes the summary and exits.

Brakes (DRIVING -> HOLDING), first one wins:
  EKF distance to the tag <= stop_distance   (the normal stop: odometry prediction + camera correction)
  front ToF <= tof_brake                      (safety only)
  distance travelled >= max_travel            (safety: odometry says we went too far)
  drive_timeout                               (safety)
"""
import math
from dataclasses import dataclass

WAITING, DRIVING, HOLDING, DONE = 'WAITING', 'DRIVING', 'HOLDING', 'DONE'


@dataclass
class ControllerParams:
    stop_distance: float = 0.40   # m, chassis front -> tag, the same for every trial
    v_max: float = 0.15           # m/s
    v_min: float = 0.03           # m/s, below this the chassis barely moves
    k_v: float = 0.6              # 1/s, slow-down gain: v = k_v * distance left
    a_max: float = 0.25           # m/s^2, ramp limit (no wheel slip at the start)
    k_heading: float = 1.0        # rad/s per rad of bearing error
    w_max: float = 0.30           # rad/s
    tag_settle: int = 10          # tag detections needed before driving
    wait_timeout: float = 15.0    # s
    hold_time: float = 2.0        # s at rest before the final estimate is read
    max_travel: float = 1.30      # m, safety
    tof_brake: float = 0.15       # m, safety
    tof_min_valid: float = 0.02   # m, smaller readings are noise
    drive_timeout: float = 30.0   # s, safety


def clamp(x, lim):
    return max(-lim, min(lim, x))


class StraightLineController:

    def __init__(self, params=None):
        self.p = params or ControllerParams()
        self.state = WAITING
        self.t_state = None        # time the current state started
        self.v_cmd = 0.0
        self.stop_reason = ''

    def _enter(self, state, now, reason=''):
        self.state, self.t_state = state, now
        if reason:
            self.stop_reason = reason

    def step(self, now, dt, d_est, heading_err, travelled, n_tag, tof=None):
        """One control tick. Returns (v, omega) in the mission frame (+v = toward the tag).

        d_est       EKF distance chassis front -> tag (m)
        heading_err EKF bearing to the tag (rad, + = tag on the left)
        travelled   EKF distance from the start position (m)
        n_tag       number of accepted tag updates so far
        tof         latest front ToF range (m) or None
        """
        p = self.p
        if self.t_state is None:
            self.t_state = now

        if self.state == WAITING:
            if n_tag >= p.tag_settle:
                self._enter(DRIVING, now)
            elif now - self.t_state > p.wait_timeout:
                self._enter(DONE, now, f'tag not seen {p.tag_settle} times in {p.wait_timeout:.0f} s')
            return self._stop()

        if self.state == DRIVING:
            reason = self._brake_reason(now, d_est, travelled, tof)
            if reason:
                self._enter(HOLDING, now, reason)
                return self._stop()
            left = d_est - p.stop_distance
            v_target = min(p.v_max, max(p.v_min, p.k_v * left))
            self.v_cmd += clamp(v_target - self.v_cmd, p.a_max * dt)     # ramp
            w = clamp(p.k_heading * heading_err, p.w_max)
            return self.v_cmd, w

        if self.state == HOLDING and now - self.t_state >= p.hold_time:
            self._enter(DONE, now)
        return self._stop()

    def abort(self, now, reason):
        """Brake right now (used by the node, e.g. when the camera arm moves). Ends in HOLDING -> DONE."""
        if self.state in (WAITING, DRIVING):
            self._enter(HOLDING, now, reason)
        return self._stop()

    def _stop(self):
        self.v_cmd = 0.0
        return 0.0, 0.0

    def _brake_reason(self, now, d_est, travelled, tof):
        p = self.p
        if d_est <= p.stop_distance:
            return f'EKF distance {d_est:.3f} m'
        if tof is not None and p.tof_min_valid < tof <= p.tof_brake:
            return f'ToF {tof:.3f} m (safety)'
        if travelled >= p.max_travel:
            return f'travelled {travelled:.3f} m (safety)'
        if now - self.t_state >= p.drive_timeout:
            return f'drive timeout {p.drive_timeout:.0f} s (safety)'
        return ''


def bearing_error(px, py, theta, tag_xy):
    """Angle from the robot's nose to the tag, wrapped to (-pi, pi]."""
    a = math.atan2(tag_xy[1] - py, tag_xy[0] - px) - theta
    return math.atan2(math.sin(a), math.cos(a))
