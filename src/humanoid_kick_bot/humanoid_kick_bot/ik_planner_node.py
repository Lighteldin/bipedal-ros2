#!/usr/bin/env python3
"""
ik_planner_node.py
-------------------
PART 2 - Robot Kinematics + PART 4 - Autonomous Behavior.

Subscribes:  /ball_position   (geometry_msgs/PointStamped, camera_link frame)
             /ball_detected   (std_msgs/Bool)
             /kick_command    (std_msgs/String, data: "left" or "right")
                               -> manually trigger a kick with that leg,
                               independent of the camera/ball.
             /walk_command    (std_msgs/String, data: "start" or "stop")
                               -> toggle an open-loop marching gait.
Publishes:   /joint_commands  (sensor_msgs/JointState) - target angles [rad]
                               for ALL 9 servos, consumed by
                               servo_controller_node.
             /leg_target      (geometry_msgs/PointStamped) - the (x,y) target
                               handed to the IK solver, for debugging/RViz.

Uses tf2 to transform the ball position from the camera frame into the
kicking leg's hip frame (<side>_thigh_link), runs the geometric IK from
kinematics.py, verifies it with FK, and drives a simple autonomous state
machine:

    SEARCH -> ALIGN -> APPROACH_READY -> WINDUP -> STRIKE -> FOLLOW_THROUGH -> RETRACT -> SEARCH

That state machine is what makes the ball-kicking behavior fully autonomous.
On top of it, two manual override commands are available for bench testing
(see the docstrings on _kick_cmd_cb / _walk_cmd_cb below) without needing a
ball in view at all.

IMPORTANT - about /walk_command:
    The mechanical design in urdf/humanoid_leg.urdf.xacro attaches the torso
    to base_footprint with a FIXED joint, matching the assignment's "stand
    stably on a flat surface using a fixed base" requirement. That means
    /walk_command cycles the leg joints through a walking-like motion
    pattern (an "attempt" at a gait), but it will NOT actually translate the
    robot across a floor - there's no mechanism in this design for the base
    itself to move. Real locomotion would need a different mechanical base
    (feet that can bear the whole robot's weight one at a time, freed from
    any fixed mount) and closed-loop balance, which is out of scope for what
    was built here.
"""

import math
import yaml
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
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
    IDLE_STATES = (STATE_SEARCH, STATE_ALIGN, STATE_READY)

    def __init__(self):
        super().__init__('ik_planner_node')

        self.declare_parameter('config_path', '')
        self.declare_parameter('kick_trigger_distance_m', 0.15)
        self.declare_parameter('align_tolerance_m', 0.02)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('walk_phase_duration_sec', 0.5)
        self.declare_parameter('kick_confirm_frames', 8)  # debounce: consecutive in-range frames needed before committing to a kick
        self.declare_parameter('autonomous_kick_enabled', False)

        cfg_path = self.get_parameter('config_path').value
        self.cfg = self._load_config(cfg_path)

        geo = self.cfg['leg_geometry']
        self.geom = LegGeometry(L1=geo['L1_thigh_m'], L2=geo['L2_shin_m'])
        self.theta1_offset = math.radians(geo['theta1_servo_offset_deg'])
        self.theta2_offset = math.radians(geo['theta2_servo_offset_deg'])
        self.kick_leg = self.cfg['kick_leg']         # 'left' or 'right' - default/autonomous kicking leg
        self.active_kick_leg = self.kick_leg          # can be swapped per-kick by /kick_command
        self.balance_pose = self.cfg['balance_pose_deg']
        self.limits = self.cfg['joint_limits_deg']


        self.kick_trigger_d = float(self.get_parameter('kick_trigger_distance_m').value)
        self.align_tol = float(self.get_parameter('align_tolerance_m').value)
        self.walk_phase_duration = float(self.get_parameter('walk_phase_duration_sec').value)
        self.kick_confirm_frames = int(self.get_parameter('kick_confirm_frames').value)
        self._in_range_count = 0  # debounce counter, see _do_track_and_check_kick
        self.autonomous_kick_enabled = bool(self.get_parameter('autonomous_kick_enabled').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.ball_point = None       # latest PointStamped in camera frame
        self.ball_seen_recently = False
        self.state = self.STATE_SEARCH
        self.state_enter_time = self.get_clock().now()

        self.walking = False
        self.walk_start_time = None

        self.create_subscription(PointStamped, '/ball_position', self._ball_cb, 10)
        self.create_subscription(Bool, '/ball_detected', self._detected_cb, 10)
        self.create_subscription(String, '/kick_command', self._kick_cmd_cb, 10)
        self.create_subscription(String, '/walk_command', self._walk_cmd_cb, 10)
        self.create_subscription(Bool, '/autonomy_enable', self._autonomy_enable_cb, 10)

        self.cmd_pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.target_pub = self.create_publisher(PointStamped, '/leg_target', 10)

        period = 1.0 / float(self.get_parameter('control_rate_hz').value)
        self.timer = self.create_timer(period, self.control_loop)

        self.get_logger().info('IK planner started; default kicking leg = ' + self.kick_leg)

    # ---------- config ----------
    def _load_config(self, path):
        if not path:
            self.get_logger().warning('No config_path given, using built-in defaults.')
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

    def _kick_cmd_cb(self, msg: String):
        """Manually trigger a kick with a chosen leg, independent of vision.

        Example:
            ros2 topic pub --once /kick_command std_msgs/msg/String "{data: 'right'}"
            ros2 topic pub --once /kick_command std_msgs/msg/String "{data: 'left'}"

        Uses a fixed, generic "straight ahead" strike target rather than a
        vision-derived one, since there may be no ball/camera involved at
        all when testing this way.
        """
        leg = (msg.data or '').strip().lower()
        if leg not in ('left', 'right'):
            self.get_logger().warning(f"Invalid /kick_command '{msg.data}' - use 'left' or 'right'")
            return
        if self.walking:
            self.get_logger().warning('Ignoring /kick_command - currently walking, send walk_command=stop first')
            return
        if self.state not in self.IDLE_STATES:
            self.get_logger().warning(f'Ignoring /kick_command - already mid-kick (state={self.state})')
            return

        self.active_kick_leg = leg
        self._kick_x = self.geom.L1 * 0.85
        self._kick_y = 0.0
        self.get_logger().info(f'Manual kick command received: {leg} leg')
        self._set_state(self.STATE_WINDUP)

    def _walk_cmd_cb(self, msg: String):
        """Start/stop the open-loop marching gait.

        Example:
            ros2 topic pub --once /walk_command std_msgs/msg/String "{data: 'start'}"
            ros2 topic pub --once /walk_command std_msgs/msg/String "{data: 'stop'}"
        """
        cmd = (msg.data or '').strip().lower()
        if cmd == 'start':
            if self.state not in self.IDLE_STATES:
                self.get_logger().warning(f'Ignoring /walk_command start - mid-kick (state={self.state})')
                return
            self.walking = True
            self.walk_start_time = self.get_clock().now()
            self.get_logger().info('Walk gait: START (open-loop leg cycling - see module docstring re: fixed base)')
        elif cmd == 'stop':
            self.walking = False
            self._publish_joint_cmd(self.balance_pose)  # return to neutral stance
            self.get_logger().info('Walk gait: STOP')
        else:
            self.get_logger().warning(f"Invalid /walk_command '{msg.data}' - use 'start' or 'stop'")

    def _autonomy_enable_cb(self, msg: Bool):
        """Toggle autonomous ball-triggered kicking at runtime, e.g. while
        tuning vision - the robot keeps searching/tracking/standing but
        will never commit to a kick on its own while this is False.
            ros2 topic pub --once /autonomy_enable std_msgs/msg/Bool "{data: false}"
            ros2 topic pub --once /autonomy_enable std_msgs/msg/Bool "{data: true}"
        Manual /kick_command and /walk_command still work regardless."""
        self.autonomous_kick_enabled = bool(msg.data)
        self.get_logger().info(f'Autonomous kicking {"ENABLED" if msg.data else "DISABLED"}')

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
        """Transform the latest ball point into the (autonomous) kicking
        leg's hip frame (<side>_thigh_link) using the TF tree from the URDF.
        Manual /kick_command kicks don't use this - they use a fixed target."""
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
            self.get_logger().warning(f'TF transform failed: {e}', throttle_duration_sec=2.0)
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
        if self.walking:
            self._do_walk_step()
            return

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
                self._in_range_count = 0
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

        # Debounce: require several CONSECUTIVE in-range frames before
        # actually committing to a kick, so one noisy/false-positive frame
        # (background color matching the HSV filter, a lighting flare, etc.)
        # can't fire the whole kick sequence by itself.
        if distance <= self.kick_trigger_d:
            self._in_range_count += 1
        else:
            self._in_range_count = 0

        if self._in_range_count >= self.kick_confirm_frames:
            self._in_range_count = 0
            if not self.autonomous_kick_enabled:
                self.get_logger().info(
                    'Ball in kicking range, but autonomous_kick_enabled is False - standing down. '
                    'Enable with: ros2 topic pub --once /autonomy_enable std_msgs/msg/Bool "{data: true}"',
                    throttle_duration_sec=3.0)
                return
            self.active_kick_leg = self.kick_leg  # autonomous kicks always use the configured leg
            self._kick_x, self._kick_y = x, y
            self._set_state(self.STATE_WINDUP)

    def _solve_and_command_leg(self, x, y, knee_forward=True, extra=None):
        """Run IK for (x,y), verify with FK, and publish the resulting servo
        commands for self.active_kick_leg while holding the stance pose."""
        try:
            theta1, theta2 = inverse_kinematics(x, y, self.geom, knee_forward=knee_forward)
        except KinematicsError as e:
            self.get_logger().warning(f'IK unreachable: {e}')
            return False

        if not verify_ik(theta1, theta2, x, y, self.geom):
            self.get_logger().error('FK verification of IK solution failed - aborting command')
            return False

        pose = dict(self.balance_pose)
        thigh_joint = f'{self.active_kick_leg}_thigh'
        shin_joint = f'{self.active_kick_leg}_shin'
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

    # ---------- walking (open-loop, see module docstring caveat) ----------
    def _command_leg_pair(self, left_xy=None, right_xy=None, knee_forward=True):
        """Command BOTH legs' IK at once (unlike _solve_and_command_leg,
        which only drives self.active_kick_leg). Used by the walk gait,
        where both legs move every phase."""
        pose = dict(self.balance_pose)

        for side, xy in (('left', left_xy), ('right', right_xy)):
            if xy is None:
                continue
            try:
                theta1, theta2 = inverse_kinematics(xy[0], xy[1], self.geom, knee_forward=knee_forward)
            except KinematicsError as e:
                self.get_logger().warning(f'Walk IK ({side}) unreachable: {e}')
                continue
            if not verify_ik(theta1, theta2, xy[0], xy[1], self.geom):
                self.get_logger().error(f'Walk FK verification failed ({side}) - skipping this leg')
                continue
            pose[f'{side}_thigh'] = self.theta1_offset + theta1
            pose[f'{side}_shin'] = self.theta2_offset + theta2

        self._publish_joint_cmd(pose)

    def _do_walk_step(self):
        """4-phase alternating march: lift+swing one leg forward, plant it,
        then repeat with the other leg. Open-loop (no balance feedback) -
        see the module docstring for why this won't actually translate a
        robot whose base is fixedly mounted."""
        elapsed = (self.get_clock().now() - self.walk_start_time).nanoseconds / 1e9
        phase = int(elapsed / self.walk_phase_duration) % 4

        reach = self.geom.L1 + self.geom.L2
        stance = (0.0, -0.90 * reach)    # under the hip, standing tall
        lift = (0.30 * reach, -0.50 * reach)   # raised, swung slightly forward
        plant = (0.50 * reach, -0.80 * reach)  # forward and back down

        if phase == 0:
            self._command_leg_pair(left_xy=stance, right_xy=lift)
        elif phase == 1:
            self._command_leg_pair(left_xy=stance, right_xy=plant)
        elif phase == 2:
            self._command_leg_pair(left_xy=lift, right_xy=stance)
        else:
            self._command_leg_pair(left_xy=plant, right_xy=stance)


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
