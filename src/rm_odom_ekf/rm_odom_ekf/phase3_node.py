"""Phase 3 ROS 2 node: odometry prediction + camera correction, straight drive to the tag, trial logging.

Room frame used by the filter:
    origin  = robot centre at the start position
    +x      = straight at the tag, +y to the left, heading 0 at the start
    tag     = (start_distance + front_offset, 0)
start_distance is taped from the chassis FRONT, front_offset is centre -> front, so every distance this
node reports is front -> tag, which is the same point the tape uses at the end of the trial.

Each tick (20 Hz): predict, correct with /odom if a new message came in, correct with the camera if a
fresh tag reading came in, run the controller, write a log row.

Subscribes: odom (nav_msgs/Odometry), tag/range_bearing (from tag_detector), range_0 (front ToF),
            arm_position (camera arm). /imu is left out on purpose: /odom already uses it.
Publishes:  cmd_vel, ekf/state = [x, y, heading, speed, turn_rate, d_est, sigma_d]
"""
import math
import statistics

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Range
from std_msgs.msg import Float32MultiArray

from rm_odom_ekf.controller import DONE, DRIVING, HOLDING, ApproachController, DriveSettings
from rm_odom_ekf.ekf import (IDX_HEADING, IDX_SPEED, IDX_TURN_RATE, IDX_X, IDX_Y, RobotEKF,
                             expected_tag_reading, wrap_angle)
from rm_odom_ekf.trial_logger import TrialRecorder, as_nested_list


def yaw_from_quaternion(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class ApproachTagNode(Node):

    def __init__(self):
        super().__init__('phase3_ekf')

        def param(name, default):
            return self.declare_parameter(name, default).value

        # --- experiment setup (must not change between trials)
        self.trial_number = param('trial', 0)
        results_dir = param('results_dir', '~/SCE672-Phase3-RoboMaster-Odometry/results/trials')
        self.start_distance = param('start_distance', 1.20)
        self.front_offset = param('front_offset', 0.16)
        rate_hz = param('control_rate', 20.0)

        # --- sign conventions of this particular robot (found in Phase 2 with the wheels in the air)
        self.drive_sign = param('drive_sign', -1.0)        # driver's +x points away from the camera
        self.odom_speed_sign = param('odom_v_sign', -1.0)
        self.odom_turn_sign = param('odom_w_sign', 1.0)
        self.odom_source = param('odom_source', 'twist')   # 'pose' if the driver leaves twist empty
        self.odom_max_speed = param('odom_max_speed', 0.40)  # m/s, pose mode: faster = driver glitch, skipped
        self.odom_max_turn = param('odom_max_turn', 1.50)    # rad/s, same

        # --- camera handling
        self.use_tag_range = param('use_tag_range', True)
        self.tag_max_age = param('tag_max_age', 0.30)       # s, older readings are skipped
        self.min_tag_quality = param('min_decision_margin', 20.0)

        # --- noise levels, all as standard deviations
        accel_std, ang_accel_std = param('sigma_accel', 0.5), param('sigma_alpha', 1.0)
        slip_xy, slip_heading = param('q_pos', 1e-4), param('q_theta', 1e-4)
        odom_speed_std, odom_turn_std = param('r_odom_v', 0.02), param('r_odom_w', 0.03)
        tag_range_std, tag_bearing_std = param('r_tag_d', 0.03), param('r_tag_phi', 0.02)
        start_pos_std = param('p0_pos', 0.02)
        start_heading_std = param('p0_theta', 0.035)
        outlier_limit = param('tag_gate', 9.21)
        self.speed_lag = param('input_tau', 0.3)            # s, how slowly the chassis follows cmd_vel

        # --- controller
        drive = DriveSettings(
            stop_distance=param('stop_distance', 0.50), max_speed=param('max_speed', 0.15),
            min_speed=param('min_speed', 0.03), slowdown_gain=param('slowdown_gain', 0.6),
            max_accel=param('max_accel', 0.25), steer_gain=param('steer_gain', 1.0),
            max_turn_rate=param('max_turn_rate', 0.30), fixes_before_start=param('fixes_before_start', 10),
            wait_limit=param('wait_limit', 15.0), settle_time=param('settle_time', 2.0),
            tof_stop=param('tof_stop', 0.15), drive_limit=param('drive_limit', 30.0))
        drive.max_travel = self.start_distance - drive.stop_distance + 0.15   # a bit past the planned stop

        # --- camera arm: the range calibration only holds at the pose it was done at
        self.check_arm = param('arm_check', True)
        self.saved_arm_pose = (param('arm_x_saved', 0.123), param('arm_z_saved', 0.067))
        self.arm_tolerance = param('arm_tolerance', 0.006)

        self.tag_position = (self.start_distance + self.front_offset, 0.0)
        start_std = [start_pos_std, start_pos_std, start_heading_std, 0.01, 0.01]
        self.ekf = RobotEKF(
            initial_state=np.zeros(5), initial_cov=np.diag(np.square(start_std)),
            tag_position=self.tag_position,
            accel_noise_var=(accel_std ** 2, ang_accel_std ** 2),
            slip_noise_rate=(slip_xy, slip_xy, slip_heading),
            odom_noise_cov=np.diag([odom_speed_std ** 2, odom_turn_std ** 2]),
            tag_noise_cov=np.diag([tag_range_std ** 2, tag_bearing_std ** 2]),
            outlier_limit=outlier_limit)
        self.controller = ApproachController(drive)

        settings_used = dict(
            start_distance=self.start_distance, front_offset=self.front_offset, tag_position=list(self.tag_position),
            control_rate=rate_hz, drive_sign=self.drive_sign, odom_v_sign=self.odom_speed_sign,
            odom_w_sign=self.odom_turn_sign, odom_source=self.odom_source, use_tag_range=self.use_tag_range,
            sigma_accel=accel_std, sigma_alpha=ang_accel_std, q_pos=slip_xy, q_theta=slip_heading,
            r_odom_v=odom_speed_std, r_odom_w=odom_turn_std, r_tag_d=tag_range_std, r_tag_phi=tag_bearing_std,
            start_std=start_std, tag_gate=outlier_limit, input_tau=self.speed_lag, drive=vars(drive))
        self.recorder = TrialRecorder(results_dir, self.trial_number, {
            'start_distance': self.start_distance, 'stop_distance': drive.stop_distance,
            'front_offset': self.front_offset, 'params': settings_used})

        # latest sensor data
        self.odom_reading = None            # (speed, turn_rate) in the trial frame
        self.odom_is_new = False
        self.prev_odom = None               # (stamp, x, y, yaw)
        self.odom_only_travel = 0.0         # summed odometry distance, never corrected by the camera
        self.tag_reading = None             # (front distance, bearing, time received)
        self.tag_is_new = False
        self.tof_range = None
        self.arm_pose = None
        self.arm_ready = not self.check_arm

        # counters for the summary
        self.odom_updates = 0
        self.tag_updates = 0
        self.tag_rejected = 0
        self.odom_glitches = 0
        self.last_gain = None
        self.camera_at_rest = []            # camera distances while stopped, for the camera-only baseline
        self.drive_started_at = None
        self.turn_cmd = 0.0
        self.finished = False

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.state_pub = self.create_publisher(Float32MultiArray, 'ekf/state', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(Float32MultiArray, 'tag/range_bearing', self.on_tag, 10)
        self.create_subscription(Range, 'range_0', self.on_tof, qos_profile_sensor_data)
        self.create_subscription(PointStamped, 'arm_position', self.on_arm, 10)

        self.t_start = self.t_prev = self.now()
        self.next_print = 0.0
        self.create_timer(1.0 / rate_hz, self.tick)
        self.get_logger().info(
            f'Trial {self.trial_number}: start {self.start_distance:.3f} m, stop {drive.stop_distance:.3f} m, '
            f'tag at x = {self.tag_position[0]:.3f} m. Logging to {self.recorder.folder}')

    # ------------------------------------------------------------------ callbacks
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg):
        stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        if self.odom_source == 'pose':
            # Work speed out from two poses in a row. Needed only if the driver doesn't fill in twist.
            pos, yaw = msg.pose.pose.position, yaw_from_quaternion(msg.pose.pose.orientation)
            prev, self.prev_odom = self.prev_odom, (stamp, pos.x, pos.y, yaw)
            if prev is None or stamp - prev[0] <= 1e-3:
                return
            dt = stamp - prev[0]
            forward = (pos.x - prev[1]) * math.cos(prev[3]) + (pos.y - prev[2]) * math.sin(prev[3])
            raw_speed, raw_turn = forward / dt, wrap_angle(yaw - prev[3]) / dt
            if abs(raw_speed) > self.odom_max_speed or abs(raw_turn) > self.odom_max_turn or dt > 0.5:
                # Impossible for this robot (it is commanded at most max_speed): a pose glitch from the driver
                # (trial 0, 2026-11-15: -0.72 m/s while standing still) or a Wi-Fi reconnect that restarted
                # /odom from zero. Skip this pair; the next message continues from the new pose.
                self.odom_glitches += 1
                return
        else:
            raw_speed, raw_turn = msg.twist.twist.linear.x, msg.twist.twist.angular.z
            prev, self.prev_odom = self.prev_odom, (stamp, 0.0, 0.0, 0.0)
            dt = 0.0 if prev is None else max(0.0, min(stamp - prev[0], 0.5))

        speed = self.odom_speed_sign * raw_speed      # positive = toward the tag
        turn_rate = self.odom_turn_sign * raw_turn
        self.odom_only_travel += speed * dt           # what wheels alone would say, for comparison
        self.odom_reading, self.odom_is_new = (speed, turn_rate), True

    def on_tag(self, msg):
        front_dist, bearing, quality = float(msg.data[0]), float(msg.data[1]), float(msg.data[2])
        if quality >= self.min_tag_quality:            # weak detections are usually blur or partial tags
            self.tag_reading, self.tag_is_new = (front_dist, bearing, self.now()), True

    def on_tof(self, msg):
        self.tof_range = float(msg.range)

    def on_arm(self, msg):
        self.arm_pose = (msg.point.x, msg.point.z)

    def arm_offset(self):
        return None if self.arm_pose is None else math.dist(self.arm_pose, self.saved_arm_pose)

    # ------------------------------------------------------------------ main loop
    def tick(self):
        if self.finished:
            return
        now = self.now()
        dt = min(max(now - self.t_prev, 1e-3), 0.2)
        self.t_prev = now
        ekf = self.ekf
        log = {'t': now - self.t_start}

        # 1) predict. The chassis speed loop lags, so model it as a(t) = (commanded - current) / lag
        if self.speed_lag > 0:
            accel_input = ((self.controller.speed_cmd - ekf.state[IDX_SPEED]) / self.speed_lag,
                           (self.turn_cmd - ekf.state[IDX_TURN_RATE]) / self.speed_lag)
        else:
            accel_input = (0.0, 0.0)
        ekf.predict(accel_input, dt)

        # 2) correct with odometry, only when a new /odom message arrived since last tick
        if self.odom_is_new:
            ekf.correct_with_odom(*self.odom_reading)
            self.odom_updates += 1
        if self.odom_reading is not None:
            log.update(odom_speed=self.odom_reading[0], odom_turn=self.odom_reading[1], odom_new=self.odom_is_new)
        self.odom_is_new = False

        # 3) correct with the camera. The detector reports front -> tag, the model wants centre -> tag
        if self.tag_reading is not None:
            front_dist, bearing, received_at = self.tag_reading
            log.update(tag_range=front_dist, tag_bearing=bearing, tag_new=self.tag_is_new)
            if self.tag_is_new and now - received_at <= self.tag_max_age:
                result = ekf.correct_with_tag(front_dist + self.front_offset, bearing, self.use_tag_range)
                log.update(tag_used=result['accepted'], tag_nis=result['nis'])
                if result['accepted']:
                    self.tag_updates += 1
                    self.last_gain = result['gain']
                    for r in range(5):
                        for c in range(result['gain'].shape[1]):
                            log[f'K{r}{c}'] = float(result['gain'][r, c])
                else:
                    self.tag_rejected += 1
                if self.controller.mode == HOLDING:
                    self.camera_at_rest.append(front_dist)
        self.tag_is_new = False

        # 4) control, using the filtered estimate
        d_est, sigma_d = ekf.distance_from_front(self.front_offset)
        bearing_est = expected_tag_reading(ekf.state, self.tag_position)[1]
        travelled = math.hypot(ekf.state[IDX_X], ekf.state[IDX_Y])
        arm_off = self.arm_offset()
        if not self.arm_ready:
            if arm_off is not None and arm_off <= self.arm_tolerance:
                self.arm_ready = True
                self.get_logger().info(f'Camera arm in place ({arm_off * 1000:.1f} mm off). Starting.')
            else:
                status = 'no arm_position yet' if arm_off is None else f'{arm_off * 1000:.0f} mm off'
                self.get_logger().warn(f'Waiting for the saved camera arm pose ({status})',
                                       throttle_duration_sec=2.0)
            speed, turn = 0.0, 0.0
        else:
            mode_before = self.controller.mode
            if self.check_arm and arm_off is not None and arm_off > self.arm_tolerance:
                speed, turn = self.controller.emergency_stop(now, f'camera arm moved {arm_off * 1000:.0f} mm (safety)')
            else:
                speed, turn = self.controller.update(now, dt, d_est, bearing_est, travelled,
                                                     self.tag_updates, self.tof_range)
            if self.controller.mode != mode_before:
                if self.controller.mode == DRIVING:
                    self.drive_started_at = now
                why = f' ({self.controller.stop_reason})' if self.controller.mode in (HOLDING, DONE) else ''
                self.get_logger().info(f'{mode_before} -> {self.controller.mode}{why}')
        self.turn_cmd = turn
        self.send(speed, turn)

        # 5) log this tick
        d_odom_only = self.start_distance - self.odom_only_travel
        s, P = ekf.state, ekf.cov
        log.update(mode=self.controller.mode, speed_cmd=speed, turn_cmd=turn,
                   tof='' if self.tof_range is None else self.tof_range,
                   x=s[IDX_X], y=s[IDX_Y], heading=s[IDX_HEADING], speed=s[IDX_SPEED], turn_rate=s[IDX_TURN_RATE],
                   d_est=d_est, sigma_d=sigma_d, d_odom_only=d_odom_only)
        for r in range(5):
            for c in range(5):
                log[f'P{r}{c}'] = float(P[r, c])
        self.recorder.write_step(log)
        self.state_pub.publish(Float32MultiArray(data=[float(v) for v in (*s, d_est, sigma_d)]))

        if log['t'] >= self.next_print:
            cam = '  -  ' if self.tag_reading is None else f'{self.tag_reading[0]:.3f}'
            print(f't={log["t"]:5.1f}s {self.controller.mode:8s} d_est={d_est:.3f}±{sigma_d:.3f} m  '
                  f'camera={cam} m  odom-only={d_odom_only:.3f} m  v={s[IDX_SPEED]:+.3f} m/s', flush=True)
            self.next_print += 0.5

        if self.controller.mode == DONE:
            # only a stop on the filtered distance counts as a proper trial
            self.finish(valid=self.controller.stop_reason.startswith('EKF'))

    def send(self, speed, turn_rate):
        cmd = Twist()
        cmd.linear.x = float(self.drive_sign * speed)
        cmd.angular.z = float(turn_rate)
        self.cmd_pub.publish(cmd)

    # ------------------------------------------------------------------ end of trial
    def finish(self, valid, reason=None):
        if self.finished:
            return
        self.finished = True
        for _ in range(5):                      # repeat in case one message gets lost on Wi-Fi
            self.send(0.0, 0.0)
        d_est, sigma_d = self.ekf.distance_from_front(self.front_offset)
        final = {
            'valid': bool(valid),
            'stop_reason': reason or self.controller.stop_reason,
            'd_est_final': d_est,
            'sigma_d_final': sigma_d,
            'd_cam_final': statistics.median(self.camera_at_rest) if self.camera_at_rest else None,
            'd_odom_only_final': self.start_distance - self.odom_only_travel,
            'x_final': [float(v) for v in self.ekf.state],
            'P_final': as_nested_list(self.ekf.cov),
            'K_last_tag': as_nested_list(self.last_gain),
            'n_odom_updates': self.odom_updates,
            'n_tag_updates': self.tag_updates,
            'n_tag_rejected': self.tag_rejected,
            'n_odom_glitches': self.odom_glitches,
            't_start_unix': self.t_start,      # wall clock of t = 0 in steps.csv (video sync)
            'drive_time_s': None if self.drive_started_at is None else self.now() - self.drive_started_at,
        }
        path = self.recorder.write_summary(final)
        self.get_logger().info(
            f'Trial {self.trial_number} done ({final["stop_reason"]}): d_est = {d_est:.3f} ± {sigma_d:.3f} m. '
            f'Tape the front -> tag distance, then run scripts/add_measurement.py. Summary: {path}')

    def shutdown(self):
        for _ in range(5):
            self.send(0.0, 0.0)
        if not self.finished:
            self.finish(valid=False, reason='interrupted (Ctrl+C)')


def main():
    # keep rclpy alive after Ctrl+C so the stop commands below still go out
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = ApproachTagNode()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()                         # robot gets a zero command whatever happened
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
