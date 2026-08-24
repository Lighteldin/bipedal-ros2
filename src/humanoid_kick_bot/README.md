# humanoid_kick_bot

A ROS 2 system for a **fixed-base humanoid lower body** that detects a
football with a camera, computes leg joint angles with forward/inverse
kinematics, and kicks it — built as a modular perception → planning →
control pipeline.

> **Scope note:** the robot stands in one spot and kicks; it does not
> translate across a floor. See "About the walk command" below for why.

---

## 1. The big picture

```
Camera → Perception Node → Planning Node → Control Node → ESP32 + PCA9685 → Servos → Kick
         (finds ball)      (kinematics +    (angles→PWM)
                            decision logic)
```

Three ROS 2 nodes, each doing one job, talking only through topics.

---

## 2. Two separate ESP32 boards

This project uses **two physically separate ESP32s** — don't try to combine
them:

| Board | Runs | Job |
|---|---|---|
| ESP32-CAM | `firmware/esp32cam_streamer/esp32cam_streamer.ino` | Streams video over WiFi to `ball_detector_node` |
| Plain ESP32 | `firmware/esp32_pca9685_bridge/esp32_pca9685_bridge.ino` | Drives the PCA9685 over I²C (GPIO21/22) from `servo_controller_node`'s serial commands |

Their GPIO21/22 usages are unrelated: on the ESP32-CAM those pins are
hardwired to the camera sensor itself; they're not available for I²C on
that board.

### Setting up the ESP32-CAM
1. Open `esp32cam_streamer.ino`, set `WIFI_SSID`/`WIFI_PASSWORD` near the top.
2. Board: **AI Thinker ESP32-CAM**, Partition Scheme: **Huge APP**.
3. Upload, then open Serial Monitor @ 115200 baud — it prints its IP once
   connected, e.g. `Camera ready! Stream at: http://192.168.1.50:81/stream`.
4. Point `ball_detector_node` at it with `esp32_cam_ip` (preferred — builds
   the URL automatically) or `camera_source` (a full URL, or a webcam
   index like `0` for local testing without the ESP32-CAM):
   ```bash
   ros2 launch humanoid_kick_bot bringup.launch.py esp32_cam_ip:=192.168.1.50
   ```
5. **Calibrate `focal_length_px`** for your specific camera: hold the ball
   at a known distance, note the pixel radius it reports, and solve
   `focal_length_px = pixel_radius * distance_m / ball_radius_m`.

WiFi MJPEG is less reliable than a wired webcam — `ball_detector_node` now
auto-reconnects after ~1s of failed frame reads (tunable via
`reconnect_after_failures` / `reconnect_backoff_sec` params), so a dropped
stream recovers on its own instead of leaving the node stuck.

---

## 3. Topics, at a glance

| Topic | Type | Published by | Used by |
|---|---|---|---|
| `/ball_position` | `geometry_msgs/PointStamped` | `ball_detector_node` | `ik_planner_node` |
| `/ball_detected` | `std_msgs/Bool` | `ball_detector_node` | `ik_planner_node` |
| `/kick_command` | `std_msgs/String` (`"left"`/`"right"`) | you, manually | `ik_planner_node` |
| `/walk_command` | `std_msgs/String` (`"start"`/`"stop"`) | you, manually | `ik_planner_node` |
| `/joint_commands` | `sensor_msgs/JointState` | `ik_planner_node` | `servo_controller_node` |
| `/joint_states` | `sensor_msgs/JointState` | `servo_controller_node` | `robot_state_publisher` |

---

## 4. Manual commands

These bypass the camera entirely — useful for bench-testing the leg motion
without needing a ball in view. Both require `ik_planner_node` to be
running (they're ignored while it's mid-kick or mid-walk).

### Kick with a specific leg
```bash
ros2 topic pub --once /kick_command std_msgs/msg/String "{data: 'right'}"
ros2 topic pub --once /kick_command std_msgs/msg/String "{data: 'left'}"
```
Runs the same WINDUP → STRIKE → FOLLOW_THROUGH → RETRACT sequence as an
autonomous ball-triggered kick, but on whichever leg you name, aimed at a
fixed "straight ahead" target instead of a vision-derived one.

### Attempt to walk
```bash
ros2 topic pub --once /walk_command std_msgs/msg/String "{data: 'start'}"
ros2 topic pub --once /walk_command std_msgs/msg/String "{data: 'stop'}"
```
Cycles both legs through a 4-phase alternating march (lift/swing one leg
forward, plant it, repeat with the other), computed via the same IK solver.

**About the walk command:** the URDF attaches the torso to `base_footprint`
with a **fixed** joint, matching the assignment's "stand stably on a flat
surface using a fixed base" requirement. So this command makes the *joints*
cycle through a walking motion, but it will **not** actually move the robot
across a floor — there's no mechanism in this mechanical design for the
base itself to translate, and no closed-loop balance control. Treat it as
a gait demo / stretch-goal starting point, not a working locomotion system.
Real walking would need a different mechanical base (feet that can bear
the robot's full weight one at a time, not fixed-mounted) plus balance
feedback — a substantially bigger project than what's built here.

---

## 5. What's in the package

```
humanoid_kick_bot/
├── humanoid_kick_bot/
│   ├── kinematics.py            # FK/IK, pure Python, unit-tested
│   ├── ball_detector_node.py    # perception (ESP32-CAM MJPEG + auto-reconnect)
│   ├── ik_planner_node.py       # planning: autonomy state machine + kick/walk commands
│   └── servo_controller_node.py # control: rad -> PWM -> serial to ESP32
├── config/servo_config.yaml     # channel map, PWM calib, link lengths, limits
├── urdf/humanoid_leg.urdf.xacro # mechanical model, drives TF/RViz
├── launch/
│   ├── bringup.launch.py            # full system: camera + planner + control + RViz
│   └── visualize_and_move.launch.py # dev/testing: RViz + control only, no camera/planner
├── rviz/humanoid.rviz
├── firmware/
│   ├── esp32cam_streamer/esp32cam_streamer.ino     # ESP32-CAM: video streaming
│   └── esp32_pca9685_bridge/esp32_pca9685_bridge.ino # ESP32: servo control
├── package.xml / setup.py / setup.cfg
└── README.md
```

### `kinematics.py`
Pure math, no ROS dependency. `forward_kinematics()`, `inverse_kinematics()`
(law of cosines, raises `KinematicsError` if unreachable), and `verify_ik()`
— a runtime FK-recheck of every IK solution before it's ever sent to a servo.

### `ball_detector_node.py`
Reads the ESP32-CAM's MJPEG stream, isolates the ball by HSV color +
contour circularity, estimates distance via the pinhole camera model, and
publishes `/ball_position` + `/ball_detected`. Auto-reconnects on stream
failure.

### `ik_planner_node.py`
The "brain": autonomous ball-kicking state machine, plus the two manual
override commands described above (`/kick_command`, `/walk_command`).
Publishes final joint targets on `/joint_commands`.

### `servo_controller_node.py`
Converts `/joint_commands` (radians) → PWM ticks → serial to the ESP32.
Continuously republishes the full 9-joint pose on `/joint_states` so RViz
always has complete, current data. Falls back to a logged dry-run if no
serial hardware is attached.

### `servo_config.yaml`
Single source of truth: channel mapping, PWM calibration, per-joint safety
limits, leg link lengths (`L1`/`L2`), and the balance/stance pose.

---

## 6. Quick reference: running it

```bash
# Full system, real ESP32-CAM:
ros2 launch humanoid_kick_bot bringup.launch.py \
    esp32_cam_ip:=192.168.1.50 serial_port:=/dev/ttyUSB0

# Visualization + manual servo testing only, no camera/planner:
ros2 launch humanoid_kick_bot visualize_and_move.launch.py \
    serial_port:=/dev/ttyUSB0

# Manually command one joint (radians), e.g. right_thigh to 150°:
ros2 topic pub --once /joint_commands sensor_msgs/msg/JointState \
    "{name: ['right_thigh'], position: [2.618]}"

# Trigger a kick without a ball/camera:
ros2 topic pub --once /kick_command std_msgs/msg/String "{data: 'right'}"

# Try the walk gait:
ros2 topic pub --once /walk_command std_msgs/msg/String "{data: 'start'}"
ros2 topic pub --once /walk_command std_msgs/msg/String "{data: 'stop'}"
```
