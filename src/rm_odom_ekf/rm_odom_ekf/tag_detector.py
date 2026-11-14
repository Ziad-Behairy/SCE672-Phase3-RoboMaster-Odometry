"""Camera side of the project. Started from my Phase 2 detector and cut down to what Phase 3 needs.
The tag lies flat on the floor, as in Phase 2.

pupil-apriltags finds tag36h11 ID 3 in camera/image_color and, given the printed tag size and the
camera intrinsics, returns where the tag sits relative to the camera. From that we publish the two
numbers the filter uses, both measured from the chassis front:
    range    = range_scale * forward + range_offset   (scale/offset come from the tape calibration)
    bearing  = atan2(left, forward)                   (+ = tag on the left, ROS convention)
on tag/range_bearing as [range, bearing, decision_margin, image_stamp].

Only the newest frame is processed, so a slow detection never leaves us working on old images.
If the images stop for 2 s (Wi-Fi reconnect), the subscription is recreated.
"""
import os

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from pupil_apriltags import Detector
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32MultiArray

from rm_odom_ekf.geometry import camera_from_floor_tag, camera_to_level


class TagDetector(Node):

    def __init__(self):
        super().__init__('tag_detector')
        self.target_id = self.declare_parameter('target_id', 3).value
        self.tag_size = self.declare_parameter('tag_size', 0.155).value         # measured black edge (m)
        self.rate = self.declare_parameter('rate', 15.0).value
        self.decimate = self.declare_parameter('quad_decimate', 1.0).value
        self.cam_pitch = np.radians(self.declare_parameter('camera_pitch_deg', 5.3).value)
        self.range_scale = self.declare_parameter('range_scale', 1.0).value
        self.range_offset = self.declare_parameter('range_offset', -0.162).value  # camera -> chassis front
        self.save_dir = os.path.expanduser(self.declare_parameter('save_frames_dir', '').value)
        if self.save_dir:
            os.makedirs(self.save_dir, exist_ok=True)

        self.detector = Detector(families='tag36h11', quad_decimate=self.decimate, refine_edges=True)
        self.bridge = CvBridge()
        self.camera_params = None        # (fx, fy, cx, cy)
        self.latest_image = None
        self.n_frames = self.n_hits = 0
        self.floor_est = None            # (height, pitch, roll) of the camera, worked out from the floor tag

        self.image_sub = self.info_sub = None
        self.subscribe_camera()
        self.last_image_time = self.get_clock().now()
        self.rb_pub = self.create_publisher(Float32MultiArray, 'tag/range_bearing', 10)
        self.point_pub = self.create_publisher(PointStamped, 'tag/point', 10)
        self.debug_pub = self.create_publisher(Image, 'tag/debug_image', 1)
        self.create_timer(1.0 / self.rate, self.process)
        self.create_timer(1.0, self.image_watchdog)
        self.create_timer(5.0, self.report)

    def subscribe_camera(self):
        if self.image_sub is not None:
            self.destroy_subscription(self.image_sub)
            self.destroy_subscription(self.info_sub)
        self.info_sub = self.create_subscription(CameraInfo, 'camera/camera_info', self.on_info, 1)
        self.image_sub = self.create_subscription(Image, 'camera/image_color', self.on_image,
                                                  qos_profile_sensor_data)

    def image_watchdog(self):
        silent = (self.get_clock().now() - self.last_image_time).nanoseconds * 1e-9
        if silent > 2.0:
            self.get_logger().warn(f'No camera image for {silent:.1f} s: re-subscribing')
            self.subscribe_camera()
            self.last_image_time = self.get_clock().now()

    def on_info(self, msg):
        if self.camera_params is None:
            k = msg.k
            self.camera_params = (k[0], k[4], k[2], k[5])
            self.get_logger().info(f'Camera {msg.width}x{msg.height}: fx={k[0]:.1f} fy={k[4]:.1f} '
                                   f'cx={k[2]:.1f} cy={k[5]:.1f}')

    def on_image(self, msg):
        self.latest_image = msg
        self.last_image_time = self.get_clock().now()

    def process(self):
        msg, self.latest_image = self.latest_image, None
        if msg is None or self.camera_params is None:
            return
        self.n_frames += 1
        gray = self.bridge.imgmsg_to_cv2(msg, 'mono8')
        detections = self.detector.detect(gray, estimate_tag_pose=True,
                                          camera_params=self.camera_params, tag_size=self.tag_size)
        target = next((d for d in detections if d.tag_id == self.target_id), None)

        if self.debug_pub.get_subscription_count() > 0 or self.save_dir:
            self.publish_debug(msg, detections)
        if target is None:
            return
        self.n_hits += 1
        # The tag lies flat, so its normal is the floor normal: that gives the camera height and pitch
        # directly. Only printed in report(), to check camera_pitch_deg at the saved arm pose.
        self.floor_est = camera_from_floor_tag(target.pose_R, target.pose_t)

        # pose_t is in the optical frame (x right, y down, z out of the lens). The camera looks slightly
        # down, so rotate by the pitch first to get forward/left on the floor plane.
        forward, left, up = camera_to_level(target.pose_t, self.cam_pitch)
        front_range = self.range_scale * forward + self.range_offset
        bearing = float(np.arctan2(left, forward))

        pt = PointStamped()
        pt.header.stamp = msg.header.stamp
        pt.header.frame_id = 'camera_level'
        pt.point.x, pt.point.y, pt.point.z = float(forward), float(left), float(up)
        self.point_pub.publish(pt)
        stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        self.rb_pub.publish(Float32MultiArray(data=[float(front_range), bearing, float(target.decision_margin), stamp]))

    def publish_debug(self, msg, detections):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8').copy()
        for det in detections:
            color = (0, 200, 0) if det.tag_id == self.target_id else (0, 160, 255)
            cv2.polylines(img, [det.corners.astype(np.int32)], True, color, 2)
            cx, cy = det.center.astype(int)
            cv2.putText(img, f'T{det.tag_id}', (cx - 15, cy + 8), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        h, w = img.shape[:2]
        cv2.line(img, (w // 2, 0), (w // 2, h), (255, 255, 255), 1)
        if self.debug_pub.get_subscription_count() > 0:
            out = self.bridge.cv2_to_imgmsg(img, 'bgr8')
            out.header = msg.header
            self.debug_pub.publish(out)
        if self.save_dir:
            stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            small = cv2.resize(img, (640, 360), interpolation=cv2.INTER_AREA)
            cv2.imwrite(os.path.join(self.save_dir, f'{stamp_ns}.jpg'), small, [cv2.IMWRITE_JPEG_QUALITY, 85])

    def report(self):
        line = f'{self.n_frames / 5.0:.1f} frames/s processed, T{self.target_id} seen in {self.n_hits}'
        if self.floor_est is not None:
            height, pitch, roll = self.floor_est
            line += (f' | camera from floor tag: height {height:.3f} m, pitch {np.degrees(pitch):+.1f} deg down, '
                     f'roll {np.degrees(roll):+.1f} deg')
        self.get_logger().info(line)
        self.floor_est = None
        self.n_frames = self.n_hits = 0


def main():
    rclpy.init()
    node = TagDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
