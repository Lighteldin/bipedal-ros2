#!/usr/bin/env python3
"""
ball_detector_node.py
----------------------
PART 3 - Computer Vision: Ball Detection.

Subscribes to nothing; owns a camera capture (USB webcam or an ESP32-CAM
MJPEG stream, both are just a cv2.VideoCapture source) and:

  1. Detects an orange/football-colored ball via HSV color threshold +
     Hough circle refinement (robust and cheap enough for a companion
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
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
import cv2
import numpy as np


class BallDetectorNode(Node):
    def __init__(self):
        super().__init__('ball_detector_node')

        self.declare_parameter('camera_source', '0')          # '0' -> /dev/video0, or an http:// MJPEG URL for ESP32-CAM
        self.declare_parameter('camera_frame_id', 'camera_link')
        self.declare_parameter('focal_length_px', 600.0)       # calibrate with checkerboard for accuracy
        self.declare_parameter('ball_radius_m', 0.035)         # mini/foam football radius
        self.declare_parameter('publish_rate_hz', 15.0)
        self.declare_parameter('hsv_lower', [5, 120, 100])     # orange football default
        self.declare_parameter('hsv_upper', [20, 255, 255])

        source = self.get_parameter('camera_source').value
        self.frame_id = self.get_parameter('camera_frame_id').value
        self.focal_px = float(self.get_parameter('focal_length_px').value)
        self.ball_radius_m = float(self.get_parameter('ball_radius_m').value)
        self.hsv_lower = np.array(self.get_parameter('hsv_lower').value, dtype=np.uint8)
        self.hsv_upper = np.array(self.get_parameter('hsv_upper').value, dtype=np.uint8)

        cam_index_or_url = int(source) if str(source).isdigit() else source
        self.cap = cv2.VideoCapture(cam_index_or_url)
        if not self.cap.isOpened():
            self.get_logger().error(f'Could not open camera source: {source}')

        self.pos_pub = self.create_publisher(PointStamped, '/ball_position', 10)
        self.detected_pub = self.create_publisher(Bool, '/ball_detected', 10)

        period = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(period, self.process_frame)

        self.get_logger().info(f'Ball detector started on source={source}, frame={self.frame_id}')

    def process_frame(self):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.detected_pub.publish(Bool(data=False))
            return

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
