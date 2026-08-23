#!/usr/bin/env python3
"""
ik_planner_node.py
-------------------
PART 2 - Robot Kinematics + PART 4 - Autonomous Behavior.

Subscribes:  /ball_position   (geometry_msgs/PointStamped, camera_link frame)
             /ball_detected   (std_msgs/Bool)
Publishes:   /joint_commands  (sensor_msgs/JointState) - target angles [rad]
                               for ALL 9 servos (camera_pan + 8 leg joints),
                               consumed by servo_controller_node.
             /leg_target      (geometry_msgs/PointStamped) - the (x,y) target
                               handed to the IK solver, for debugging/RViz.

Uses tf2 to transform the ball position from the camera frame into the
kicking leg's hip frame (<side>_thigh_link), runs the geometric IK from
kinematics.py, verifies it with FK, and drives a simple autonomous state
machine:

    SEARCH -> ALIGN -> APPROACH_READY -> WINDUP -> STRIKE -> FOLLOW_THROUGH -> RETRACT -> SEARCH

The state machine is what makes the system "fully autonomous": no operator
input is required once the node is launched with a ball in view.
"""

import math
import yaml
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import tf2_ros
from tf2_geometry_msgs import do_transform_point

from humanoid_kick_bot.kinematics import (
    LegGeometry, inverse_kinematics, verify_ik, KinematicsError, clamp_angle
)

JOINT_ORDER = [
    'camera_pan', 'left_waist', 'right_waist', 'left_thigh', 'right_thigh',
    'left_shin', 'right_shin', 'left_foot', 'right_foot'
]


class IKPlannerNode(Node):
    STATE_SEARCH = 'SEARCH'
    STATE_ALIGN = 'ALIGN'
    STATE_READY = 'APPROACH_READY'
    STATE_WINDUP = 'WINDUP'
    STATE_STRIKE = 'STRIKE'
    STATE_FOLLOW = 'FOLLOW_THROUGH'
    STATE_RETRACT = 'RETRACT'

    def __init__(self):
        super().__init__('ik_planner_node')

        self.declare_parameter('config_path', '')
        self.declare_parameter('kick_trigger_distance_m', 0.15)
        self.declare_parameter('align_tolerance_m', 0.02)
        self.declare_parameter('control_rate_hz', 20.0)

        cfg_path = self.get_parameter('config_path').value
        self.cfg = self._load_config(cfg_path)

        geo = self.cfg['leg_geometry']
        self.geom = LegGeometry(L1=geo['L1_thigh_m'], L2=geo['L2_shin_m'])
        self.theta1_offset = math.radians(geo['theta1_servo_offset_deg'])
        self.theta2_offset = math.radians(geo['theta2_servo_offset_deg'])
        self.kick_leg = self.cfg['kick_leg']  # 'left' or 'right'
        self.balance_pose = self.cfg['balance_pose_deg']
        self.limits = self.cfg['joint_limits_deg']

        self.kick_trigger_d = float(self.get_parameter('kick_trigger_distance_m').value)
        self.align_tol = float(self.get_parameter('align_tolerance_m').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.ball_point = None       # latest PointStamped in camera frame
        self.ball_seen_recently = False
        self.state = self.STATE_SEARCH
        self.state_enter_time = self.get_clock().now()

        self.create_subscription(PointStamped, '/ball_position', self._ball_cb, 10)
        self.create_subscription(Bool, '/ball_detected', self._detected_cb, 10)

        self.cmd_pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.target_pub = self.create_publisher(PointStamped, '/leg_target', 10)

        period = 1.0 / float(self.get_parameter('control_rate_hz').value)
        self.timer = self.create_timer(period, self.control_loop)

        self.get_logger().info('IK planner started; kicking leg = ' + self.kick_leg)

    # ---------- config ----------
    def _load_config(self, path):
        if not path:
            self.get_logger().warn('No config_path given, using built-in defaults.')
            return {
                'leg_geometry': {'L1_thigh_m': 0.09, 'L2_shin_m': 0.085,
                                  'theta1_servo_offset_deg': 90, 'theta2_servo_offset_deg': 90},
                'kick_leg': 'right',
                'balance_pose_deg': {'left_waist': 90, 'right_waist': 90, 'left_thigh': 90,
                                      'left_shin': 90, 'left_foot': 90, 'right_foot': 90},
                'joint_limits_deg': {},
            }
        with open(path, 'r') as f:
            return yaml.safe_load(f)

    # ---------- callbacks ----------
    def _ball_cb(self, msg: PointStamped):
        self.ball_point = msg

    def _detected_cb(self, msg: Bool):
        self.ball_seen_recently = msg.data

    # ---------- helpers ----------
    def _deg(self, name):
        return math.radians(self.balance_pose.get(name, 90))

    def _clip_to_limits(self, name, angle_rad):
        lim = self.limits.get(name)
        if not lim:
            return angle_rad
        return clamp_angle(angle_rad, math.radians(lim['min']), math.radians(lim['max']))

    def _publish_joint_cmd(self, positions: dict, camera_pan_rad=None):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_ORDER
        vals = []
        for j in JOINT_ORDER:
            if j == 'camera_pan':
                v = camera_pan_rad if camera_pan_rad is not None else math.radians(90)
            else:
                v = positions.get(j, self._deg(j))
            vals.append(self._clip_to_limits(j, v))
        msg.position = vals
        self.cmd_pub.publish(msg)

    def _transform_ball_to_hip_frame(self):
        """Transform the latest ball point into the kicking-leg hip frame
        (<side>_thigh_link) using the TF tree published from the URDF."""
        if self.ball_point is None:
            return None
        target_frame = f'{self.kick_leg}_thigh_link'
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame, self.ball_point.header.frame_id,
                rclpy.time.Time(), timeout=Duration(seconds=0.2))
            p = do_transform_point(self.ball_point, tf)
            return p
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'TF transform failed: {e}', throttle_duration_sec=2.0)
            return None

    # ---------- state machine ----------
    def _set_state(self, new_state):
        if new_state != self.state:
            self.get_logger().info(f'[state] {self.state} -> {new_state}')
            self.state = new_state
            self.state_enter_time = self.get_clock().now()

    def _time_in_state(self):
        return (self.get_clock().now() - self.state_enter_time).nanoseconds / 1e9

    def control_loop(self):
        p = self._transform_ball_to_hip_frame()

        if self.state == self.STATE_SEARCH:
            self._do_search()
            if self.ball_seen_recently and p is not None:
                self._set_state(self.STATE_ALIGN)

        elif self.state == self.STATE_ALIGN:
            if not self.ball_seen_recently or p is None:
                self._set_state(self.STATE_SEARCH)
                return
            self._do_align(p)

        elif self.state == self.STATE_READY:
            if not self.ball_seen_recently or p is None:
                self._set_state(self.STATE_SEARCH)
                return
            self._do_track_and_check_kick(p)

        elif self.state == self.STATE_WINDUP:
            self._do_windup()
        elif self.state == self.STATE_STRIKE:
            self._do_strike()
        elif self.state == self.STATE_FOLLOW:
            self._do_follow_through()
        elif self.state == self.STATE_RETRACT:
            self._do_retract()

    def _do_search(self):
        """No ball in view: sweep the camera pan servo to search."""
        t = self._time_in_state()
        sweep = math.radians(90) + math.radians(45) * math.sin(t)
        self._publish_joint_cmd(self.balance_pose, camera_pan_rad=sweep)

    def _do_align(self, ball_in_hip_frame: PointStamped):
        """Center the camera pan on the ball (waist/camera yaw alignment)."""
        y = ball_in_hip_frame.point.y  # lateral offset
        pan_correction = clamp_angle(-y * 2.0, math.radians(-45), math.radians(45))
        pan = math.radians(90) + pan_correction
        self._publish_joint_cmd(self.balance_pose, camera_pan_rad=pan)

        if abs(y) < self.align_tol:
            self._set_state(self.STATE_READY)

    def _do_track_and_check_kick(self, ball_in_hip_frame: PointStamped):
        x, y = ball_in_hip_frame.point.x, ball_in_hip_frame.point.y
        distance = math.hypot(x, y)

        target_pt = PointStamped()
        target_pt.header.stamp = self.get_clock().now().to_msg()
        target_pt.header.frame_id = f'{self.kick_leg}_thigh_link'
        target_pt.point.x, target_pt.point.y = x, y
        self.target_pub.publish(target_pt)

        self._publish_joint_cmd(self.balance_pose)

        if distance <= self.kick_trigger_d:
            self._kick_x, self._kick_y = x, y
            self._set_state(self.STATE_WINDUP)

    def _solve_and_command_leg(self, x, y, knee_forward=True, extra=None):
        """Run IK for (x,y), verify with FK, and publish the resulting servo
        commands for the kicking leg while holding the stance pose."""
        try:
            theta1, theta2 = inverse_kinematics(x, y, self.geom, knee_forward=knee_forward)
        except KinematicsError as e:
            self.get_logger().warn(f'IK unreachable: {e}')
            return False

        if not verify_ik(theta1, theta2, x, y, self.geom):
            self.get_logger().error('FK verification of IK solution failed - aborting command')
            return False

        pose = dict(self.balance_pose)
        thigh_joint = f'{self.kick_leg}_thigh'
        shin_joint = f'{self.kick_leg}_shin'
        pose[thigh_joint] = self.theta1_offset + theta1
        pose[shin_joint] = self.theta2_offset + theta2
        if extra:
            pose.update(extra)
        self._publish_joint_cmd(pose)
        return True

    def _do_windup(self):
        """Swing the kicking leg backward to load the strike."""
        windup_x = self.geom.L1 * 0.6
        windup_y = -(self.geom.L1 + self.geom.L2) * 0.5
        self._solve_and_command_leg(windup_x, windup_y, knee_forward=False)
        if self._time_in_state() > 0.6:
            self._set_state(self.STATE_STRIKE)

    def _do_strike(self):
        """Drive the foot through the ball's position at (near) full extension."""
        ok = self._solve_and_command_leg(self._kick_x, self._kick_y, knee_forward=True)
        if not ok:
            self._set_state(self.STATE_RETRACT)
            return
        if self._time_in_state() > 0.3:
            self._set_state(self.STATE_FOLLOW)

    def _do_follow_through(self):
        follow_x = self.geom.L1 * 0.9
        follow_y = (self.geom.L1 + self.geom.L2) * 0.3
        self._solve_and_command_leg(follow_x, follow_y, knee_forward=True)
        if self._time_in_state() > 0.4:
            self._set_state(self.STATE_RETRACT)

    def _do_retract(self):
        """Return the kicking leg to the neutral standing pose."""
        self._publish_joint_cmd(self.balance_pose)
        if self._time_in_state() > 0.5:
            self._set_state(self.STATE_SEARCH)


def main(args=None):
    rclpy.init(args=args)
    node = IKPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
