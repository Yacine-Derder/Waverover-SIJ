# WaveRover onboard stack

Shared deployment settings live in `config/robot_defaults.yaml`. Machine
identity is deliberately separate: copy `config/robot_identity.example.yaml`
to the ignored `config/robot_identity.yaml` and set only `robot_name`. The
onboard stack fails clearly if this file is absent or invalid. It can also be
selected explicitly with `WAVEROVER_IDENTITY_FILE`.

```bash
ros2 launch waverover robot.launch.py
```

The default shared mode is `control_mode=fixed_wing` and `pose_source=MCS`.
MCS starts the UART bridge and waypoint controller and consumes
`/macortex_bridge/waverover_<ID>/pose` in the global `robotics_lab` frame.
SLAM starts LiDAR, static TF, RF2O, SLAM Toolbox, the bridge, controller, and
optional Foxglove. Its frames include `waverover_<ID>/map`,
`waverover_<ID>/odom`, and `waverover_<ID>/base_footprint`. `/tf` and
`/tf_static` remain global topics while frame IDs prevent collisions.

Launch arguments may temporarily override identity and shared modes:

```bash
ros2 launch waverover robot.launch.py \
  robot_name:=30 control_mode:=twist pose_source:=SLAM
```

When `control_mode=manual_lr`, the unified launch omits the autonomous
controller. Robot-local keyboard wrappers use the machine identity by default
and accept `--robot-name 30` overrides. The separate operator waypoint UI does
not use a rover identity file and requires an explicit robot target.

The installed package contains `robot_defaults.yaml` and the identity example,
never the ignored real identity. With `--symlink-install`, onboard identity
discovery resolves the source-tree path; the systemd integration also exports
the absolute path.

Rebuild after source changes:

```bash
cd /home/waverover/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```
