"""Tag detector node: the robot's "eyes". Trimmed from the Phase 2 detector (rm_flatness_nav).

- Reads camera/image_color and camera/camera_info from the robomaster_ros driver.
- Finds the target AprilTag (family tag36h11) with pupil-apriltags, which also returns the tag position
  in the camera frame from the known printed tag size and the camera intrinsics.
- Turns that into the two numbers the EKF needs, measured from the CHASSIS FRONT:
    d    = range_scale * forward + range_offset     (horizontal distance, calibrated with a tape)
    phi  = atan2(left, forward)                      (bearing, + = tag on the left, REP-103)
- Publishes tag/range_bearing = [d, phi, decision_margin, image_stamp_s] and tag/point (forward, left, up).

Only the newest frame is processed (fixed-rate timer, queue depth 1), so slow detection never builds a
backlog of old frames. A watchdog re-subscribes if the camera goes silent after a Wi-Fi reconnect.
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

from rm_odom_ekf.geometry import camera_to_level


class TagDetector(Node):

    def __init__(self):
        super().__init__('tag_detector')
        self.target_id = self.declare_parameter('target_id', 3).value
        self.tag_size = self.declare_parameter('tag_size', 0.25).value          # measured black edge (m)
        self.rate = self.declare_parameter('rate', 15.0).value
        self.decimate = self.declare_parameter('quad_decimate', 1.0).value
        self.cam_pitch = np.radians(self.declare_parameter('camera_pitch_deg', 5.3).value)
        self.range_scale = self.declare_parameter('range_scale', 1.0).value
        self.range_offset = self.declare_parameter('range_offset', -0.149).value  # camera -> chassis front
        self.save_dir = os.path.expanduser(self.declare_parameter('save_frames_dir', '').value)
        if self.save_dir:
            os.makedirs(self.save_dir, exist_ok=True)

        self.detector = Detector(families='tag36h11', quad_decimate=self.decimate, refine_edges=True)
        self.bridge = CvBridge()
        self.camera_params = None        # (fx, fy, cx, cy)
        self.latest_image = None
        self.n_frames = self.n_hits = 0

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

        forward, left, up = camera_to_level(target.pose_t, self.cam_pitch)
        d = self.range_scale * forward + self.range_offset     # chassis front -> tag (m)
        phi = float(np.arctan2(left, forward))                 # + = tag on the left

        pt = PointStamped()
        pt.header.stamp = msg.header.stamp
        pt.header.frame_id = 'camera_level'
        pt.point.x, pt.point.y, pt.point.z = float(forward), float(left), float(up)
        self.point_pub.publish(pt)
        stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        self.rb_pub.publish(Float32MultiArray(data=[float(d), phi, float(target.decision_margin), stamp]))

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
        self.get_logger().info(f'{self.n_frames / 5.0:.1f} frames/s processed, '
                               f'T{self.target_id} seen in {self.n_hits}')
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
