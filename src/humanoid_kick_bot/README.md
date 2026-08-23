# humanoid_kick_bot

Autonomous ball-kicking humanoid lower body — ROS 2 implementation for the
AI for Robotics final term project. Modular perception → planning → control
pipeline, driving a PCA9685 (via ESP32) with your existing 9-servo channel
map.

## Your hardware, exactly as specified

| Joint          | PCA9685 CH | Notes                              |
|----------------|-----------:|-------------------------------------|
| Camera pan     | 7          | search sweep / ball-tracking yaw    |
| Left Waist     | 8          | hip yaw, stance/posture only        |
| Right Waist    | 9          | hip yaw, stance/posture only        |
| Left Thigh     | 10         | θ1 (hip pitch) — IK joint if kicking|
| Right Thigh    | 11         | θ1 (hip pitch) — IK joint if kicking|
| Left Shin      | 12         | θ2 (knee) — IK joint if kicking     |
| Right Shin     | 13         | θ2 (knee) — IK joint if kicking     |
| Left Foot      | 14         | ankle, stance/posture only          |
| Right Foot     | 15         | ankle, stance/posture only          |

- All servos: 0–180°, mapped linearly to PWM 120–520, PCA9685 @ 50 Hz.
- PCA9685 I²C wired to **ESP32 GPIO21 (SDA) / GPIO22 (SCL)**.
- `kick_leg: right` in `config/servo_config.yaml` selects which leg is the
  2-DOF IK leg from the assignment; the other leg + both waists/feet hold a
  fixed stance/balance pose. Flip it to `left` if that's your kicking leg —
  nothing else needs to change.

## Architecture (modular, matches the assignment's Parts 1–4)

```
[ESP32-CAM / USB webcam]
        │  image frames
        ▼
 ball_detector_node  (perception)
   HSV + Hough circle detection → pinhole distance estimate
        │  /ball_position (PointStamped, camera_link)
        │  /ball_detected (Bool)
        ▼
 ik_planner_node     (planning)
   tf2: camera_link → right_thigh_link
   geometric IK (law of cosines) + FK verification
   autonomous state machine: SEARCH→ALIGN→READY→WINDUP→STRIKE→FOLLOW→RETRACT
        │  /joint_commands (JointState, radians, all 9 joints)
        ▼
 servo_controller_node (control)
   rad → deg → PWM(120-520) → serial line "channel,ticks\n" → ESP32
        │  /joint_states (mirrors commanded pose for RViz/TF)
        ▼
 robot_state_publisher (URDF) → TF tree → RViz2
```

Each stage is an independent ROS 2 node communicating only over topics, so
you can develop/test perception, planning and control separately (e.g. run
`ik_planner_node` + `servo_controller_node` with a hand-published
`/ball_position` to test the kick, with no camera attached).

## Part 2 — Kinematics, mapped to code

`humanoid_kick_bot/kinematics.py` implements exactly the FK/IK from the
brief:

- FK: `x = L1*cos(θ1) + L2*cos(θ1+θ2)`, `y = L1*sin(θ1) + L2*sin(θ1+θ2)`
- IK: law-of-cosines geometric solve for `θ2`, then `θ1` from
  `atan2(y,x) - atan2(L2 sinθ2, L1+L2 cosθ2)`, with an unreachable-target
  guard and a knee-direction choice (`knee_forward`).
- `verify_ik()` re-runs FK on every IK solution before it's sent to
  `/joint_commands` — this is the runtime FK-validates-IK safety check the
  brief calls for. If verification fails, the planner aborts that command
  and logs an error rather than moving the servo.

`L1`/`L2` and the servo-neutral offset (kinematic θ=0 ⇒ servo 90°) live in
`config/servo_config.yaml` — measure your actual thigh/shin link lengths
and put them there.

## Part 3 — Vision

`ball_detector_node` works with either a USB webcam (`camera_source:=0`) or
an ESP32-CAM MJPEG stream (`camera_source:="http://<esp32-cam-ip>/stream"`).
Tune `hsv_lower`/`hsv_upper` to your ball's color and `focal_length_px` /
`ball_radius_m` for accurate distance estimation (a quick checkerboard
calibration will get you a good `focal_length_px`).

## Part 4 — Autonomous behavior

`ik_planner_node` is a self-driving state machine — no operator input is
needed after launch:

`SEARCH` (sweep camera pan) → `ALIGN` (center on ball) → `APPROACH_READY`
(track ball, wait until in kicking range) → `WINDUP` → `STRIKE` → auto
→ `FOLLOW_THROUGH` → `RETRACT` → back to `SEARCH`.

TF is published by `robot_state_publisher` from the URDF
(`urdf/humanoid_leg.urdf.xacro`), giving you `base_footprint → base_link →
{left,right}_waist_link → {left,right}_thigh_link → {left,right}_shin_link
→ {left,right}_foot_link` plus `camera_link`. The planner uses this tree
(via `tf2_ros`) to transform the ball from `camera_link` into
`right_thigh_link` (the IK frame origin) before solving.

## Build & run

```bash
# 1. Arduino side: flash firmware/esp32_pca9685_bridge/esp32_pca9685_bridge.ino
#    to the ESP32 (Adafruit PWM Servo Driver Library required).

# 2. ROS 2 side (on your companion PC / Raspberry Pi):
cd ~/ros2_ws
cp -r humanoid_kick_bot src/
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select humanoid_kick_bot
source install/setup.bash

ros2 launch humanoid_kick_bot bringup.launch.py \
    camera_source:=0 \
    serial_port:=/dev/ttyUSB0
```

No hardware attached yet? `servo_controller_node` auto-falls-back to
DRY-RUN mode (logs the PWM it would send) if the serial port can't be
opened or `pyserial` isn't installed, so the full graph — including RViz
visualization — still runs for a software-only demo.

## Files

```
humanoid_kick_bot/
├── humanoid_kick_bot/
│   ├── kinematics.py            # FK/IK (Part 2, pure math, unit-testable)
│   ├── ball_detector_node.py    # perception (Part 3)
│   ├── ik_planner_node.py       # planning + autonomy state machine (Parts 2 & 4)
│   └── servo_controller_node.py # control: PWM + serial to ESP32
├── config/servo_config.yaml     # channel map, PWM calib, link lengths, limits
├── urdf/humanoid_leg.urdf.xacro # Part 1 mechanical model, drives TF/RViz
├── launch/bringup.launch.py
├── rviz/humanoid.rviz
├── firmware/esp32_pca9685_bridge/esp32_pca9685_bridge.ino
├── package.xml / setup.py / setup.cfg
└── README.md
```
