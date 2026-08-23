#!/usr/bin/env python3
"""
servo_controller_node.py
-------------------------
Actuator control node (final stage of the perception -> planning -> control
pipeline).

Subscribes:  /joint_commands (sensor_msgs/JointState), angles in RADIANS,
             names matching the 9 servo joints in servo_config.yaml.

Does:        - Converts each radian angle -> servo degrees [0,180] ->
               PCA9685 PWM tick using the pwm_min/pwm_max calibration from
               servo_config.yaml (linear map, matches the "0-180 deg into
               120-520 PWM" spec).
             - Clamps to the per-joint safety limits.
             - Streams one line per changed channel to the ESP32 over serial
               in the simple text protocol:  "<channel>,<pwm_ticks>\n"
               e.g. "10,340\n" -> channel 10 (left_thigh) to 340 ticks.
               The matching Arduino sketch is in
               firmware/esp32_pca9685_bridge/esp32_pca9685_bridge.ino
             - Republishes the commanded pose on /joint_states so
               robot_state_publisher can drive TF/RViz even without hardware
               attached (useful for simulation/demo).

Hardware notes:
    PCA9685 is wired to the ESP32 on GPIO21 (SDA) / GPIO22 (SCL) and runs at
    50 Hz. The ESP32, not this ROS node, talks I2C to the PCA9685; this node
    only talks serial (USB) to the ESP32. If ROS 2 is instead running
    directly on a Linux SBC with I2C pins wired to the PCA9685, swap the
    SerialBridge class below for a smbus2/Adafruit_PCA9685 direct driver -
    the rest of the node is unchanged.
"""

import math
import time
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    import serial
except ImportError:
    serial = None


class SerialBridge:
    """Thin wrapper around pyserial; degrades to a dry-run logger if no
    serial port / pyserial is available, so the graph still runs without
    hardware attached (e.g. for RViz-only demos)."""

    def __init__(self, port, baud, logger):
        self.logger = logger
        self.ser = None
        if serial is None:
            self.logger.warn('pyserial not installed - running in DRY-RUN mode (no hardware).')
            return
        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
            time.sleep(2.0)  # allow ESP32 to reset after port open
            self.logger.info(f'Connected to ESP32 on {port} @ {baud} baud')
        except Exception as e:
            self.logger.warn(f'Could not open serial port {port}: {e}. DRY-RUN mode.')
            self.ser = None

    def send(self, channel: int, pwm_ticks: int):
        line = f'{channel},{pwm_ticks}\n'
        if self.ser is not None:
            try:
                self.ser.write(line.encode('ascii'))
            except Exception as e:
                self.logger.error(f'Serial write failed: {e}')
        else:
            self.logger.debug(f'[DRY-RUN] -> {line.strip()}')


class ServoControllerNode(Node):
    def __init__(self):
        super().__init__('servo_controller_node')

        self.declare_parameter('config_path', '')
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('serial_baud', 115200)

        cfg_path = self.get_parameter('config_path').value
        self.cfg = self._load_config(cfg_path)

        pca = self.cfg['pca9685']
        self.pwm_min = pca['pwm_min']
        self.pwm_max = pca['pwm_max']
        self.angle_min = pca['angle_min_deg']
        self.angle_max = pca['angle_max_deg']
        self.channels = self.cfg['channels']
        self.limits = self.cfg.get('joint_limits_deg', {})

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('serial_baud').value
        self.bridge = SerialBridge(port, baud, self.get_logger())

        self._last_pwm = {}

        self.create_subscription(JointState, '/joint_commands', self._cmd_cb, 10)
        self.state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Home all servos to neutral (90 deg) on startup, as specified.
        self._home_all()

        self.get_logger().info('Servo controller ready.')

    def _load_config(self, path):
        if not path:
            raise RuntimeError('servo_controller_node requires config_path to servo_config.yaml')
        with open(path, 'r') as f:
            return yaml.safe_load(f)

    def _deg_to_pwm(self, angle_deg: float) -> int:
        angle_deg = max(self.angle_min, min(self.angle_max, angle_deg))
        span_deg = self.angle_max - self.angle_min
        span_pwm = self.pwm_max - self.pwm_min
        return int(round(self.pwm_min + (angle_deg - self.angle_min) * span_pwm / span_deg))

    def _clip_limits(self, name, angle_deg):
        lim = self.limits.get(name)
        if not lim:
            return angle_deg
        return max(lim['min'], min(lim['max'], angle_deg))

    def _home_all(self):
        names, positions = [], []
        for name in self.channels.keys():
            self._send_angle(name, 90.0)
            names.append(name)
            positions.append(math.radians(90.0))
        init_state = JointState()
        init_state.header.stamp = self.get_clock().now().to_msg()
        init_state.name = names
        init_state.position = positions
        self.state_pub.publish(init_state)

    def _send_angle(self, name: str, angle_deg: float):
        if name not in self.channels:
            self.get_logger().warn(f'Unknown joint "{name}", no channel mapping - skipped.')
            return
        angle_deg = self._clip_limits(name, angle_deg)
        ch = self.channels[name]
        pwm = self._deg_to_pwm(angle_deg)
        if self._last_pwm.get(ch) == pwm:
            return  # avoid spamming identical commands
        self._last_pwm[ch] = pwm
        self.bridge.send(ch, pwm)

    def _cmd_cb(self, msg: JointState):
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = list(msg.name)
        out.position = list(msg.position)

        for name, pos_rad in zip(msg.name, msg.position):
            angle_deg = math.degrees(pos_rad)
            self._send_angle(name, angle_deg)

        # Mirror the commanded pose to /joint_states for RViz/TF, regardless
        # of whether real hardware is attached.
        self.state_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ServoControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
