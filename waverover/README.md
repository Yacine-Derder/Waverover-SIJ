# WaveRover onboard stack

All deployment defaults live in `config/robot_defaults.yaml`. The unified
launch reads this file and selects either the SLAM pipeline or the external
motion-capture pose path:

```bash
ros2 launch waverover robot.launch.py
```

The waypoint terminal remains a separate operator-laptop application and is
not started by this command.

## Central configuration

SLAM deployment:

```yaml
robot_name: "29"
control_mode: fixed_wing
pose_source: SLAM
```

MCS deployment:

```yaml
robot_name: "29"
control_mode: fixed_wing
pose_source: MCS
```

The same file defines the MCS topic pattern, frame, and timeout:

```yaml
mcs:
  pose_topic_pattern: /macortex_bridge/{robot_namespace}/pose
  frame: robotics_lab
  pose_timeout_sec: 0.50
```

Changing only `robot_name` derives the namespace, robot topics, TF frame IDs,
and MCS topic. Robot 30 therefore uses `/robot_30/...`, namespaced SLAM frames,
and `/macortex_bridge/robot_30/pose`. `/tf`, `/tf_static`, and the external
`/macortex_bridge/...` path remain global topics.

SLAM mode starts LiDAR, the base-to-laser static transform, RF2O, SLAM
Toolbox, the rover bridge, the waypoint controller, and optionally Foxglove.
MCS mode starts only the rover bridge and waypoint controller. The external
MCS bridge runs on the operator laptop and is intentionally never launched
onboard.

Launch arguments provide temporary overrides without editing the central
file:

```bash
ros2 launch waverover robot.launch.py \
  robot_name:=30 control_mode:=twist pose_source:=MCS
```

Use `ros2 launch waverover robot.launch.py --show-args` for UART, sensor,
Foxglove, controller-tuning, and MCS overrides. When `control_mode=manual_lr`,
the unified launch omits the incompatible autonomous waypoint controller and
keeps the bridge/manual command path available.

Rebuild after changing source configuration:

```bash
cd ~/ros2_ws
colcon build --packages-select waverover ros2waverover \
  waverover_controller waverover_waypoint_ui --symlink-install
source install/setup.bash
```

## Frames and Foxglove

In SLAM mode, robot 29 publishes `/robot_29/map` and `/robot_29/scan`; choose
`robot_29/map` as the Foxglove 3D fixed/display frame. MCS mode does not start
map or scan producers, so those topics are intentionally absent. MCS waypoint
coordinates use `robotics_lab`; the controller does not create a synthetic TF
tree from the incoming pose.

Foxglove is global. In a multi-robot SLAM composition, start one bridge and use
`start_foxglove:=false` for additional onboard launches.

## Keyboard and manual control

The wrappers read their default robot from the central configuration:

```bash
ros2 run waverover waverover_teleop
ros2 run waverover waverover_manual_lr
```

Use `--robot-name 30` for a temporary robot override. The manual L/R wrapper
is intended for a bridge launched with `control_mode:=manual_lr`.
