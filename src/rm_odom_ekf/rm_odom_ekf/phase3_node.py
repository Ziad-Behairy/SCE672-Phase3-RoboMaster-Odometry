"""Phase 3 node: odometry prediction + camera correction (EKF), straight-line drive, trial logging.

Room frame: origin at the robot CENTRE at the start, +x straight toward the tag, +y left, theta = 0 at start.
The tag is a known constant at (start_distance + front_offset, 0): start_distance is taped from the CHASSIS
FRONT to the tag, front_offset is robot centre -> chassis front. All reported distances are chassis front ->
tag, the same reference point the tape uses (spec, required work 2).

Every tick (control_rate, 20 Hz):
  1. predict      x- = f(x, u), P- = F P F^T + G Q G^T          u = (a, alpha) from the commanded speed
  2. odometry     if a new /odom arrived: correct v, omega      (Lecture 11B, H_odom)
  3. camera       if a new tag reading arrived: correct d, phi  (Lecture 12A, H_tag, wrapped bearing)
  4. control      drive straight, brake when the EKF distance reaches stop_distance
  5. log          one row of steps.csv; summary.json when the trial ends

Topics in:  odom (nav_msgs/Odometry, ~10 Hz, already fuses wheels + IMU), tag/range_bearing (tag_detector),
            range_0 (front ToF, safety brake only), arm_position (camera arm, optional guard).
Topics out: cmd_vel (geometry_msgs/Twist), ekf/state [px, py, theta, v, omega, d_est, sigma_d].
/imu is deliberately NOT used: /odom already contains it, and using both counts the IMU twice.
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

from rm_odom_ekf.controller import DONE, DRIVING, HOLDING, WAITING, ControllerParams, StraightLineController
from rm_odom_ekf.ekf import EKF, PX, PY, TH, V, W, h_tag, wrap
from rm_odom_ekf.trial_logger import TrialLogger, matrix_to_list


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Phase3Node(Node):

    def __init__(self):
        super().__init__('phase3_ekf')
        P = self.declare_parameter
        # experiment
        self.trial = P('trial', 0).value
        results_dir = P('results_dir', '~/SCE672-Phase3-RoboMaster-Odometry/results/trials').value
        self.start_distance = P('start_distance', 1.50).value   # m, tape: chassis front -> tag at start
        self.front_offset = P('front_offset', 0.16).value       # m, robot centre -> chassis front
        self.rate = P('control_rate', 20.0).value
        # robot conventions (Phase 2: the driver's +x points AWAY from the camera side)
        self.drive_sign = P('drive_sign', -1.0).value           # cmd_vel.linear.x = drive_sign * v
        self.odom_v_sign = P('odom_v_sign', -1.0).value         # v = odom_v_sign * odom forward speed
        self.odom_w_sign = P('odom_w_sign', 1.0).value
        self.odom_source = P('odom_source', 'twist').value      # 'twist' or 'pose' (differentiate pose)
        # camera measurement
        self.use_range = P('use_tag_range', True).value         # False = bearing only (for the report)
        self.tag_max_age = P('tag_max_age', 0.30).value         # s
        self.min_margin = P('min_decision_margin', 20.0).value
        # EKF noise (standard deviations)
        sa, sal = P('sigma_accel', 0.5).value, P('sigma_alpha', 1.0).value
        qp, qt = P('q_pos', 1e-4).value, P('q_theta', 1e-4).value
        rv, rw = P('r_odom_v', 0.02).value, P('r_odom_w', 0.03).value
        rd, rp = P('r_tag_d', 0.03).value, P('r_tag_phi', 0.02).value
        p0_pos = P('p0_pos', 0.02).value                        # start placement accuracy (m)
        p0 = [p0_pos, p0_pos, P('p0_theta', 0.035).value, 0.01, 0.01]
        gate = P('tag_gate', 9.21).value
        self.input_tau = P('input_tau', 0.3).value              # s, inner speed-loop lag; 0 = u = 0
        # controller
        cp = ControllerParams(
            stop_distance=P('stop_distance', 0.40).value, v_max=P('v_max', 0.15).value,
            v_min=P('v_min', 0.03).value, k_v=P('k_v', 0.6).value, a_max=P('a_max', 0.25).value,
            k_heading=P('k_heading', 1.0).value, w_max=P('w_max', 0.30).value,
            tag_settle=P('tag_settle', 10).value, wait_timeout=P('wait_timeout', 15.0).value,
            hold_time=P('hold_time', 2.0).value, tof_brake=P('tof_brake', 0.15).value,
            drive_timeout=P('drive_timeout', 30.0).value)
        cp.max_travel = self.start_distance - cp.stop_distance + 0.15
        # camera arm guard: the camera calibration is only valid at the saved arm pose
        self.arm_check = P('arm_check', True).value
        self.arm_saved = (P('arm_x_saved', 0.123).value, P('arm_z_saved', 0.067).value)
        self.arm_tol = P('arm_tolerance', 0.006).value

        self.tag_xy = (self.start_distance + self.front_offset, 0.0)
        self.ekf = EKF(x0=np.zeros(5), P0=np.diag(np.square(p0)), tag_xy=self.tag_xy,
                       q_accel=(sa ** 2, sal ** 2), q_pose=(qp, qp, qt),
                       R_odom=np.diag([rv ** 2, rw ** 2]), R_tag=np.diag([rd ** 2, rp ** 2]), gate=gate)
        self.ctrl = StraightLineController(cp)

        self.params = dict(
            start_distance=self.start_distance, front_offset=self.front_offset, tag_xy=list(self.tag_xy),
            control_rate=self.rate, drive_sign=self.drive_sign, odom_v_sign=self.odom_v_sign,
            odom_w_sign=self.odom_w_sign, odom_source=self.odom_source, use_tag_range=self.use_range,
            sigma_accel=sa, sigma_alpha=sal, q_pos=qp, q_theta=qt, r_odom_v=rv, r_odom_w=rw, r_tag_d=rd,
            r_tag_phi=rp, p0=p0, tag_gate=gate, input_tau=self.input_tau, controller=vars(cp))
        self.log = TrialLogger(results_dir, self.trial, {
            'start_distance': self.start_distance, 'stop_distance': cp.stop_distance,
            'front_offset': self.front_offset, 'params': self.params})

        # latest sensor values
        self.odom = None                 # (v, w) in the mission frame
        self.odom_new = False
        self.odom_prev = None            # (stamp, x, y, yaw) for odom_source = 'pose'
        self.s_odom = 0.0                # odometry-only distance travelled (prediction without correction)
        self.tag = None                  # (d_front, phi, receive_time)
        self.tag_new = False
        self.tof = None
        self.arm_pos = None
        self.arm_ok = not self.arm_check
        # counters and results
        self.n_odom = self.n_tag = self.n_tag_rej = 0
        self.last_K = None
        self.hold_cam = []               # camera distances while HOLDING (robot at rest)
        self.t_drive = None
        self.w_cmd = 0.0                 # last commanded turn rate
        self.done = False

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.state_pub = self.create_publisher(Float32MultiArray, 'ekf/state', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(Float32MultiArray, 'tag/range_bearing', self.on_tag, 10)
        self.create_subscription(Range, 'range_0', self.on_tof, qos_profile_sensor_data)
        self.create_subscription(PointStamped, 'arm_position', self.on_arm, 10)

        self.t0 = self.t_last = self.now()
        self.next_print = 0.0
        self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(f'Trial {self.trial}: start {self.start_distance:.3f} m, stop '
                               f'{cp.stop_distance:.3f} m, tag at x={self.tag_xy[0]:.3f} m. Folder {self.log.dir}')

    # ------------------------------------------------------------ inputs
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg):
        stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        if self.odom_source == 'pose':
            p, yaw = msg.pose.pose.position, yaw_of(msg.pose.pose.orientation)
            prev, self.odom_prev = self.odom_prev, (stamp, p.x, p.y, yaw)
            if prev is None or stamp - prev[0] <= 1e-3:
                return
            dt = stamp - prev[0]
            fwd = ((p.x - prev[1]) * math.cos(prev[3]) + (p.y - prev[2]) * math.sin(prev[3])) / dt
            v_raw, w_raw = fwd, wrap(yaw - prev[3]) / dt
        else:
            v_raw, w_raw = msg.twist.twist.linear.x, msg.twist.twist.angular.z
            prev = self.odom_prev
            self.odom_prev = (stamp, 0.0, 0.0, 0.0)
            dt = 0.0 if prev is None else max(0.0, min(stamp - prev[0], 0.5))
        v, w = self.odom_v_sign * v_raw, self.odom_w_sign * w_raw
        self.s_odom += v * dt                        # dead reckoning: what odometry alone would say
        self.odom, self.odom_new = (v, w), True

    def on_tag(self, msg):
        d, phi, margin = float(msg.data[0]), float(msg.data[1]), float(msg.data[2])
        if margin >= self.min_margin:
            self.tag, self.tag_new = (d, phi, self.now()), True

    def on_tof(self, msg):
        self.tof = float(msg.range)

    def on_arm(self, msg):
        self.arm_pos = (msg.point.x, msg.point.z)

    def arm_error(self):
        if self.arm_pos is None:
            return None
        return math.dist(self.arm_pos, self.arm_saved)

    # ------------------------------------------------------------ main loop
    def tick(self):
        if self.done:
            return
        now = self.now()
        dt = min(max(now - self.t_last, 1e-3), 0.2)
        self.t_last = now
        f = self.ekf
        row = {'t': now - self.t0}

        # 1. predict. u = (a, alpha): the chassis speed loop pulls v toward the command with lag input_tau
        if self.input_tau > 0:
            u = ((self.ctrl.v_cmd - f.x[V]) / self.input_tau, (self.w_cmd - f.x[W]) / self.input_tau)
        else:
            u = (0.0, 0.0)
        f.predict(u, dt)

        # 2. odometry correction
        if self.odom_new:
            f.update_odom(*self.odom)
            self.n_odom += 1
        if self.odom is not None:
            row.update(odom_v=self.odom[0], odom_w=self.odom[1], odom_new=self.odom_new)
        self.odom_new = False

        # 3. camera correction (only fresh readings)
        if self.tag is not None:
            d_front, phi, t_rx = self.tag
            row.update(tag_d=d_front, tag_phi=phi, tag_new=self.tag_new)
            if self.tag_new and now - t_rx <= self.tag_max_age:
                res = f.update_tag(d_front + self.front_offset, phi, use_range=self.use_range)
                row.update(tag_used=res['accepted'], tag_nis=res['nis'])
                if res['accepted']:
                    self.n_tag += 1
                    self.last_K = res['K']
                    for i in range(5):
                        for j in range(res['K'].shape[1]):
                            row[f'K{i}{j}'] = float(res['K'][i, j])
                else:
                    self.n_tag_rej += 1
                if self.ctrl.state == HOLDING:
                    self.hold_cam.append(d_front)
        self.tag_new = False

        # 4. control
        d_est, sigma_d = f.distance_to_tag(self.front_offset)
        heading_err = h_tag(f.x, self.tag_xy)[1]
        travelled = math.hypot(f.x[PX], f.x[PY])
        err = self.arm_error()
        if not self.arm_ok:
            if err is not None and err <= self.arm_tol:
                self.arm_ok = True
                self.get_logger().info(f'Arm pose OK ({err * 1000:.1f} mm from saved). Trial starts.')
            else:
                self.get_logger().warn('Waiting for the saved camera arm pose '
                                       f'({"no arm_position yet" if err is None else f"{err * 1000:.0f} mm off"})',
                                       throttle_duration_sec=2.0)
            v, w = 0.0, 0.0
        else:
            prev = self.ctrl.state
            if self.arm_check and err is not None and err > self.arm_tol:
                v, w = self.ctrl.abort(now, f'camera arm moved {err * 1000:.0f} mm (safety)')
            else:
                v, w = self.ctrl.step(now, dt, d_est, heading_err, travelled, self.n_tag, self.tof)
            if self.ctrl.state != prev:
                if self.ctrl.state == DRIVING:
                    self.t_drive = now
                extra = f' ({self.ctrl.stop_reason})' if self.ctrl.state in (HOLDING, DONE) else ''
                self.get_logger().info(f'{prev} -> {self.ctrl.state}{extra}')
        self.w_cmd = w
        self.send(v, w)

        # 5. log
        d_odom_only = self.start_distance - self.s_odom
        x, Pm = f.x, f.P
        row.update(state=self.ctrl.state, v_cmd=v, w_cmd=w, tof=self.tof if self.tof is not None else '',
                   px=x[PX], py=x[PY], theta=x[TH], v=x[V], w=x[W], d_est=d_est, sigma_d=sigma_d,
                   d_odom_only=d_odom_only)
        for i in range(5):
            for j in range(5):
                row[f'P{i}{j}'] = float(Pm[i, j])
        self.log.log_step(row)
        self.state_pub.publish(Float32MultiArray(data=[float(v_) for v_ in (*x, d_est, sigma_d)]))
        if row['t'] >= self.next_print:
            cam = f'{self.tag[0]:.3f}' if self.tag is not None else '  -  '
            print(f't={row["t"]:5.1f}s {self.ctrl.state:8s} d_est={d_est:.3f}±{sigma_d:.3f} m  camera={cam} m  '
                  f'odom-only={d_odom_only:.3f} m  v={x[V]:+.3f} m/s', flush=True)
            self.next_print += 0.5

        if self.ctrl.state == DONE:
            self.finish(valid=self.ctrl.stop_reason.startswith('EKF'))   # safety stops are not valid trials

    def send(self, v, w):
        cmd = Twist()
        cmd.linear.x = float(self.drive_sign * v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)

    # ------------------------------------------------------------ end of trial
    def finish(self, valid, reason=None):
        if self.done:
            return
        self.done = True
        for _ in range(5):
            self.send(0.0, 0.0)
        d_est, sigma_d = self.ekf.distance_to_tag(self.front_offset)
        summary = {
            'valid': bool(valid),
            'stop_reason': reason or self.ctrl.stop_reason,
            'd_est_final': d_est,                              # EKF: prediction + correction
            'sigma_d_final': sigma_d,                          # 1-sigma from P
            'd_cam_final': statistics.median(self.hold_cam) if self.hold_cam else None,   # camera alone, at rest
            'd_odom_only_final': self.start_distance - self.s_odom,                       # odometry alone
            'x_final': [float(v) for v in self.ekf.x],
            'P_final': matrix_to_list(self.ekf.P),
            'K_last_tag': matrix_to_list(self.last_K),
            'n_odom_updates': self.n_odom,
            'n_tag_updates': self.n_tag,
            'n_tag_rejected': self.n_tag_rej,
            'drive_time_s': None if self.t_drive is None else self.now() - self.t_drive,
        }
        path = self.log.finish(summary)
        self.get_logger().info(f'Trial {self.trial} done ({summary["stop_reason"]}): d_est = {d_est:.3f} '
                               f'± {sigma_d:.3f} m. Now tape the chassis front -> tag distance and run '
                               f'scripts/add_measurement.py. Summary: {path}')

    def stop(self):
        for _ in range(5):
            self.send(0.0, 0.0)
        if not self.done:
            self.finish(valid=False, reason='interrupted (Ctrl+C)')


def main():
    # keep the ROS context alive on Ctrl+C so the zero-velocity commands can still be published
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Phase3Node()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()                  # zero velocity on every exit path
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
