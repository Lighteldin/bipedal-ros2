# humanoid_kick_bot

A ROS 2 system for a **fixed-base humanoid lower body** that detects a
football with a camera, computes leg joint angles with forward/inverse
kinematics, and kicks it — built as a modular perception → planning →
control pipeline.

> **Scope note:** the robot stands in one spot and kicks. It does not walk
> toward the ball. That matches the assignment's "stand stably on a flat
> surface using a fixed base" requirement.

---

## 1. The big picture

```
Camera → Perception Node → Planning Node → Control Node → ESP32 + PCA9685 → Servos → Kick
         (finds ball)      (kinematics +    (angles→PWM)
                            decision logic)
```

Three ROS 2 nodes, each doing one job, talking only through topics:

| Stage | Node | Job |
|---|---|---|
| Perception | `ball_detector_node` | Find the ball in the camera image, estimate its 3D position |
| Planning | `ik_planner_node` | Decide what the robot should do, and where the foot should go, using FK/IK |
| Control | `servo_controller_node` | Turn joint angles into real PWM signals and send them to hardware |

Below that sits `robot_state_publisher`, which reads the URDF (the robot's
"skeleton" description) and publishes the TF tree, letting RViz draw the
robot and letting the planner reason about coordinate frames.

---

## 2. This is a **package**, not multiple packages

Everything lives in **one ROS 2 package**, `humanoid_kick_bot`, built with
`ament_python`. "Modular" here means modular *nodes* inside one package
(separate perception/planning/control processes talking over topics), not
separate ROS packages — that's a normal and expected way to satisfy a
"modular architecture" requirement without the overhead of managing
several interdependent packages.

---

## 3. What's in the package, and what each part does

```
humanoid_kick_bot/
├── humanoid_kick_bot/            ← Python source (the actual node code)
│   ├── kinematics.py
│   ├── ball_detector_node.py
│   ├── ik_planner_node.py
│   └── servo_controller_node.py
├── config/
│   └── servo_config.yaml
├── urdf/
│   └── humanoid_leg.urdf.xacro
├── launch/
│   ├── bringup.launch.py
│   └── visualize_and_move.launch.py
├── rviz/
│   └── humanoid.rviz
├── firmware/
│   └── esp32_pca9685_bridge/esp32_pca9685_bridge.ino
├── package.xml
├── setup.py / setup.cfg
└── README.md
```

### `kinematics.py` — the math, no ROS involved
Pure Python functions, no dependency on ROS at all, so they can be tested
in isolation (and were — round-trip tested over a full angle grid).

- `forward_kinematics(theta1, theta2, geom)` — given hip angle θ1 and knee
  angle θ2, returns where the foot ends up `(x, y)`.
- `inverse_kinematics(x, y, geom, knee_forward)` — given a target foot
  position, returns the θ1/θ2 needed to reach it, using the law of
  cosines. Raises `KinematicsError` if the target is out of reach.
- `verify_ik(...)` — re-runs FK on an IK result and checks it actually
  reproduces the target. This is the runtime safety check: if IK produced
  a bad answer, this catches it *before* a servo command is sent.

### `ball_detector_node.py` — perception
Opens a camera (USB webcam or an ESP32-CAM MJPEG stream), and every frame:
1. Thresholds the image by color (HSV) to isolate the ball.
2. Finds the largest ball-shaped contour.
3. Uses the pinhole camera model (`known ball size + pixel size → distance`)
   to estimate how far away the ball is, and how far left/right/up/down.
4. Publishes that as `/ball_position` (a 3D point) and `/ball_detected`
   (a yes/no flag).

*(Not wired into your testing yet — you've been feeding fake positions by
hand instead, which is exactly what this node's output would normally be.)*

### `ik_planner_node.py` — planning + the autonomy logic
This is the "brain." It:
1. Listens for `/ball_position`.
2. Uses `tf2` to convert that position from the camera's frame into the
   kicking leg's own frame (i.e. "how far is the ball from *this hip
   joint*").
3. Runs `inverse_kinematics()` to get the θ1/θ2 needed to reach it, and
   `verify_ik()` to double check the answer before trusting it.
4. Runs a state machine that makes the whole thing autonomous — no person
   needs to press a button mid-sequence:

   ```
   SEARCH → ALIGN → APPROACH_READY → WINDUP → STRIKE → FOLLOW_THROUGH → RETRACT → (back to SEARCH)
   ```

   - **SEARCH** — no ball seen; sweep the camera pan servo looking for one.
   - **ALIGN** — ball seen; rotate to center it.
   - **APPROACH_READY** — ball centered; wait until it's within kicking range.
   - **WINDUP → STRIKE → FOLLOW_THROUGH** — the kick itself, as three IK
     targets in sequence.
   - **RETRACT** — leg returns to neutral standing pose, cycle repeats.

5. Publishes the resulting joint angles for **all 9 servos** (not just the
   two leg joints) on `/joint_commands`, since the rest of the body needs
   to hold a steady stance pose while the kick happens.

### `servo_controller_node.py` — control (the only node that talks to hardware)
1. Subscribes to `/joint_commands` (angles in radians).
2. Converts radians → degrees → PWM ticks, using your `120–520` /
   `0–180°` calibration from `servo_config.yaml`.
3. Clamps each joint to its configured safe range of motion.
4. Sends `"channel,ticks\n"` lines over USB serial to the ESP32.
5. Keeps a running record of every joint's current angle and republishes
   the **full 9-joint state** on `/joint_states` five times a second — this
   is what lets RViz and `robot_state_publisher` always have a complete,
   current picture of the robot, regardless of whether real hardware is
   attached or when RViz happened to start.

If no ESP32 is plugged in (or `pyserial` isn't installed), this node
**doesn't crash** — it logs what it *would* have sent and keeps running, so
you can still test the rest of the system without hardware.

### `esp32_pca9685_bridge.ino` — the only code that isn't ROS
Arduino firmware for the ESP32. It does nothing clever: reads
`"channel,ticks"` lines off serial and calls `pwm.setPWM()`. All the
"thinking" happens upstream in ROS; this is just the last hop from serial
bytes to an I²C command on the PCA9685.

### `humanoid_leg.urdf.xacro` — the robot's skeleton
Describes every link and joint (fixed base, waist → thigh → shin → foot per
leg, camera pan) with the same names used everywhere else in the code.
`robot_state_publisher` reads this and, combined with live `/joint_states`
data, computes the full TF tree — the thing that lets RViz draw the robot
moving and lets the planner reason in 3D coordinates.

### `servo_config.yaml` — the single source of truth
Every other file reads from this instead of hardcoding numbers:
channel-to-servo mapping, PWM calibration, per-joint safety limits, leg
link lengths (`L1`/`L2`), and the "balance pose" the stance leg holds
during a kick. **Update this file, not the code**, when you measure your
real hardware.

### `launch/bringup.launch.py` — full system
Starts everything: perception + planning + control + `robot_state_publisher`
+ RViz. This is the "real" autonomous launch, once the camera is wired in.

### `launch/visualize_and_move.launch.py` — dev/testing launch
Starts only `robot_state_publisher` + RViz + `servo_controller_node` — no
camera, no planner. This is what you've actually been using: you manually
publish to `/joint_commands` and watch both the real servo and the RViz
model respond.

### `rviz/humanoid.rviz` — saved RViz layout
So you don't have to manually add the RobotModel/TF displays every time you
open RViz.

---

## 4. Topics, at a glance

| Topic | Type | Published by | Used by |
|---|---|---|---|
| `/ball_position` | `geometry_msgs/PointStamped` | `ball_detector_node` | `ik_planner_node` |
| `/ball_detected` | `std_msgs/Bool` | `ball_detector_node` | `ik_planner_node` |
| `/joint_commands` | `sensor_msgs/JointState` | `ik_planner_node` (or you, manually) | `servo_controller_node` |
| `/joint_states` | `sensor_msgs/JointState` | `servo_controller_node` | `robot_state_publisher` |

---

## 5. What's actually verified working right now

- ✅ `servo_controller_node` — confirmed moving real servos over serial via
  manual `ros2 topic pub` commands, on every channel.
- ✅ `kinematics.py` — FK/IK round-trip tested in isolation, correct.
- ✅ RViz visualization — `robot_state_publisher` + RViz showing the full
  TF tree and robot model, driven by `/joint_states`.
- ⏳ `ball_detector_node` — built, not yet connected (no camera wired in).
- ⏳ `ik_planner_node` — built, not yet run end-to-end (you've been
  substituting manual `/joint_commands` publishes for its output).

---

## 6. Quick reference: running it

```bash
# Everything, real system (once camera is wired in):
ros2 launch humanoid_kick_bot bringup.launch.py \
    camera_source:=0 serial_port:=/dev/ttyUSB0

# Visualization + manual servo testing only, no camera/planner:
ros2 launch humanoid_kick_bot visualize_and_move.launch.py \
    serial_port:=/dev/ttyUSB0

# Manually command one joint (radians), e.g. right_thigh to 150°:
ros2 topic pub --once /joint_commands sensor_msgs/msg/JointState \
    "{name: ['right_thigh'], position: [2.618]}"
```
