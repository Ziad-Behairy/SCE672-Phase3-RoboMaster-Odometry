"""Drive straight at the tag and stop at a fixed distance. No ROS here, so it can be unit tested.

Modes:
    WAITING   stand still until the camera has given enough good fixes (the filter settles meanwhile)
    DRIVING   ramp up, cruise, slow down as the tag gets close, keep the nose on the tag
    HOLDING   wheels stopped, wait a moment so the robot stops rolling before we read the final estimate
    DONE      trial over

The normal stop comes from the filtered distance (prediction + correction), not the raw camera.
The ToF sensor, distance travelled and a timer are only there as safety stops.
"""
import math
from dataclasses import dataclass

WAITING, DRIVING, HOLDING, DONE = 'WAITING', 'DRIVING', 'HOLDING', 'DONE'


@dataclass
class DriveSettings:
    stop_distance: float = 0.50       # m, chassis front to tag. Same value for every trial
    max_speed: float = 0.15           # m/s
    min_speed: float = 0.03           # m/s, the chassis hardly moves below this
    slowdown_gain: float = 0.6        # 1/s, target speed = gain * distance still to go
    max_accel: float = 0.25           # m/s^2, gentle ramp so the wheels don't slip at the start
    steer_gain: float = 1.0           # rad/s per rad of bearing error
    max_turn_rate: float = 0.30       # rad/s
    fixes_before_start: int = 10      # accepted camera fixes needed before moving
    wait_limit: float = 15.0          # s, give up if the tag never shows up
    settle_time: float = 2.0          # s standing still before the final reading
    max_travel: float = 1.30          # m, safety
    tof_stop: float = 0.15            # m, safety
    tof_ignore_below: float = 0.02    # m, the ToF reports ~0 when it has no echo
    drive_limit: float = 30.0         # s, safety


def clamp(value, limit):
    return max(-limit, min(limit, value))


class ApproachController:

    def __init__(self, settings=None):
        self.cfg = settings or DriveSettings()
        self.mode = WAITING
        self.mode_started = None
        self.speed_cmd = 0.0
        self.stop_reason = ''

    def _switch(self, mode, now, reason=''):
        self.mode, self.mode_started = mode, now
        if reason:
            self.stop_reason = reason

    def update(self, now, dt, distance_left, bearing, travelled, good_fixes, tof_range=None):
        """One control step. Returns (speed, turn_rate); positive speed = toward the tag.

        distance_left  filtered distance chassis front -> tag (m)
        bearing        filtered angle from the nose to the tag (rad, + = left)
        travelled      filtered distance from the start point (m)
        good_fixes     camera readings the filter has accepted so far
        tof_range      latest front ToF reading (m), or None
        """
        cfg = self.cfg
        if self.mode_started is None:
            self.mode_started = now

        if self.mode == WAITING:
            if good_fixes >= cfg.fixes_before_start:
                self._switch(DRIVING, now)
            elif now - self.mode_started > cfg.wait_limit:
                self._switch(DONE, now, f'tag not seen {cfg.fixes_before_start} times in {cfg.wait_limit:.0f} s')
            return self._halt()

        if self.mode == DRIVING:
            reason = self._why_stop(now, distance_left, travelled, tof_range)
            if reason:
                self._switch(HOLDING, now, reason)
                return self._halt()
            to_go = distance_left - cfg.stop_distance
            target = min(cfg.max_speed, max(cfg.min_speed, cfg.slowdown_gain * to_go))
            self.speed_cmd += clamp(target - self.speed_cmd, cfg.max_accel * dt)   # ramp toward target
            turn = clamp(cfg.steer_gain * bearing, cfg.max_turn_rate)
            return self.speed_cmd, turn

        if self.mode == HOLDING and now - self.mode_started >= cfg.settle_time:
            self._switch(DONE, now)
        return self._halt()

    def emergency_stop(self, now, reason):
        """Brake straight away (the node calls this if the camera arm moves). Still goes HOLDING -> DONE."""
        if self.mode in (WAITING, DRIVING):
            self._switch(HOLDING, now, reason)
        return self._halt()

    def _halt(self):
        self.speed_cmd = 0.0
        return 0.0, 0.0

    def _why_stop(self, now, distance_left, travelled, tof_range):
        cfg = self.cfg
        if distance_left <= cfg.stop_distance:
            return f'EKF distance {distance_left:.3f} m'
        if tof_range is not None and cfg.tof_ignore_below < tof_range <= cfg.tof_stop:
            return f'ToF {tof_range:.3f} m (safety)'
        if travelled >= cfg.max_travel:
            return f'travelled {travelled:.3f} m (safety)'
        if now - self.mode_started >= cfg.drive_limit:
            return f'drive timeout {cfg.drive_limit:.0f} s (safety)'
        return ''


def bearing_to(x, y, heading, tag_position):
    """Angle from the nose to the tag, folded into (-pi, pi]."""
    angle = math.atan2(tag_position[1] - y, tag_position[0] - x) - heading
    return math.atan2(math.sin(angle), math.cos(angle))
