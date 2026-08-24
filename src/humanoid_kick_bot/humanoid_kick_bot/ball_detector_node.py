#!/usr/bin/env python3
"""
ball_detector_node.py
----------------------
PART 3 - Computer Vision: Ball Detection.

Reads an MJPEG-over-HTTP stream from an ESP32-CAM (see
firmware/esp32cam_streamer/esp32cam_streamer.ino) and:

  1. Detects an orange/football-colored ball via HSV color threshold +
     contour circularity check (robust and cheap enough for a companion
     computer / Raspberry Pi, no GPU needed).
  2. Estimates the ball's real-world position relative to the camera using
     the pinhole camera model and a known ball radius:
         Z (distance) = (focal_px * real_radius_m) / pixel_radius_px
         X (lateral)  = (u - cx) * Z / focal_px
         Y (vertical) = (v - cy) * Z / focal_px
  3. Publishes geometry_msgs/PointStamped on /ball_position in the
     "camera_link" frame, and a Bool on /ball_detected.

The planner node (ik_planner_node) transforms this into the kicking leg's
local plane via tf2 and feeds it to the IK solver.

WiFi camera notes:
    A WiFi MJPEG stream is far less reliable than a wired USB webcam - the
    connection can drop, stall, or return garbage frames if the ESP32-CAM
    reboots, loses WiFi, or the network is congested. This node therefore
    auto-reconnects after a run of failed reads instead of just silently
    reporting "no ball" forever.
"""

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
import cv2
import numpy as np


class BallDetectorNode(Node):
    def __init__(self):
        super().__init__('ball_detector_node')

        # Preferred: set esp32_cam_ip and the URL is built automatically.
        # Fallback: set camera_source directly (full URL, or a webcam index
        # like '0' for local testing/debugging without the ESP32-CAM).
        self.declare_parameter('esp32_cam_ip', '')              # e.g. '192.168.1.50'
        self.declare_parameter('camera_source', '0')            # used only if esp32_cam_ip is empty
        self.declare_parameter('camera_frame_id', 'camera_link')
        self.declare_parameter('focal_length_px', 600.0)        # calibrate for YOUR ESP32-CAM lens, see README
        self.declare_parameter('ball_radius_m', 0.035)          # mini/foam football radius
        self.declare_parameter('publish_rate_hz', 15.0)
        self.declare_parameter('hsv_lower', [5, 120, 100])      # orange football default
        self.declare_parameter('hsv_upper', [20, 255, 255])
        self.declare_parameter('reconnect_after_failures', 15)  # ~1s of dropped frames at 15Hz
        self.declare_parameter('reconnect_backoff_sec', 2.0)

        esp32_cam_ip = self.get_parameter('esp32_cam_ip').value
        if esp32_cam_ip:
            self.camera_source = f'http://{esp32_cam_ip}:81/stream'
        else:
            raw = self.get_parameter('camera_source').value
            self.camera_source = int(raw) if str(raw).isdigit() else raw

        self.frame_id = self.get_parameter('camera_frame_id').value
        self.focal_px = float(self.get_parameter('focal_length_px').value)
        self.ball_radius_m = float(self.get_parameter('ball_radius_m').value)
        self.hsv_lower = np.array(self.get_parameter('hsv_lower').value, dtype=np.uint8)
        self.hsv_upper = np.array(self.get_parameter('hsv_upper').value, dtype=np.uint8)
        self.reconnect_after_failures = int(self.get_parameter('reconnect_after_failures').value)
        self.reconnect_backoff_sec = float(self.get_parameter('reconnect_backoff_sec').value)

        self._consecutive_failures = 0
        self._last_reconnect_attempt = 0.0
        self.cap = None
        self._open_capture()

        self.pos_pub = self.create_publisher(PointStamped, '/ball_position', 10)
        self.detected_pub = self.create_publisher(Bool, '/ball_detected', 10)

        period = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(period, self.process_frame)

        self.get_logger().info(
            f'Ball detector started on source={self.camera_source}, frame={self.frame_id}')

    def _open_capture(self):
        if self.cap is not None:
            self.cap.release()
        self.cap = cv2.VideoCapture(self.camera_source)
        if not self.cap.isOpened():
            self.get_logger().error(f'Could not open camera source: {self.camera_source}')
        else:
            self._consecutive_failures = 0

    def _reconnect_if_needed(self):
        """After several consecutive bad reads, drop and reopen the stream -
        common recovery pattern for a flaky WiFi MJPEG source. Backs off
        between attempts so it doesn't hammer a genuinely offline camera."""
        if self._consecutive_failures < self.reconnect_after_failures:
            return
        now = time.monotonic()
        if now - self._last_reconnect_attempt < self.reconnect_backoff_sec:
            return
        self._last_reconnect_attempt = now
        self.get_logger().warning(
            f'{self._consecutive_failures} consecutive bad frames - '
            f'reconnecting to {self.camera_source}')
        self._open_capture()

    def process_frame(self):
        if self.cap is None or not self.cap.isOpened():
            self._consecutive_failures += 1
            self._reconnect_if_needed()
            self.detected_pub.publish(Bool(data=False))
            return

        ok, frame = self.cap.read()
        if not ok or frame is None:
            self._consecutive_failures += 1
            self._reconnect_if_needed()
            self.detected_pub.publish(Bool(data=False))
            return

        self._consecutive_failures = 0

        h, w = frame.shape[:2]
        cx, cy = w / 2.0, h / 2.0

        blurred = cv2.GaussianBlur(frame, (9, 9), 2)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected = False
        if contours:
            largest = max(contours, key=cv2.contourArea)
            (u, v), pixel_radius = cv2.minEnclosingCircle(largest)
            area = cv2.contourArea(largest)
            circle_area = np.pi * pixel_radius * pixel_radius

            # Reject noise: must be reasonably circular and not tiny
            if pixel_radius > 4 and circle_area > 0 and (area / circle_area) > 0.6:
                detected = True
                Z = (self.focal_px * self.ball_radius_m) / pixel_radius
                X = (u - cx) * Z / self.focal_px
                Y = (v - cy) * Z / self.focal_px

                msg = PointStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id
                # Camera optical convention -> robot convention:
                # forward = Z (distance to ball), lateral = X, vertical = -Y
                msg.point.x = float(Z)
                msg.point.y = float(-X)
                msg.point.z = float(-Y)
                self.pos_pub.publish(msg)

        self.detected_pub.publish(Bool(data=detected))

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BallDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

