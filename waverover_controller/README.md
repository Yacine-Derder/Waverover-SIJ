# WaveRover waypoint controller

MCS supplies approximately 60 Hz poses and the configurable onboard loop
defaults to 30 Hz. `goal_tolerance_m` remains 0.05 m. Progress is tracked per
stamp token; stalled navigation uses bounded straight escapes, then publishes
`waypoint_failed` with the original frame, token, and coordinates if recovery
is exhausted. Recovery never fabricates a reached acknowledgement.

The controller consumes a reliable FIFO stream of
`geometry_msgs/msg/PointStamped` waypoints and publishes namespaced Twist
commands. Defaults come from `waverover/config/robot_defaults.yaml`; both the
bridge and controller use the shared `control_mode` name.

The controller supports two pose providers without duplicating navigation
logic:

- `pose_source=SLAM` looks up
  `waverover_<ID>/map -> waverover_<ID>/base_footprint` on TF and accepts
  waypoints in `waverover_<ID>/map`.
- `pose_source=MCS` consumes the latest valid PoseStamped on the derived
  `/macortex_bridge/waverover_<ID>/pose` topic and accepts waypoints in the
  configured MCS frame, normally `robotics_lab`. It does not create or require
  a SLAM TF listener.

MCS poses with a wrong frame, non-finite fields, or a zero quaternion are
rejected. The rover uses the existing explicit safe-stop command until the
first valid pose, stops again when pose receipt exceeds
`mcs.pose_timeout_sec`, and automatically resumes when valid updates return.
The freshness timeout is based on local monotonic receipt time, not on the
message timestamp, so unsynchronized laptop/rover wall clocks are safe.

The waypoint `header.stamp` is the logical command token. When a waypoint is
removed at `goal_tolerance_m`, the controller publishes a reliable
`PointStamped` acknowledgement on `/waverover_<ID>/waypoint_reached`, echoing
the original frame, stamp, and coordinates. Recently reached tokens are kept
in a bounded cache, so delayed PC refreshes cannot re-enter the FIFO; the same
coordinate with a genuinely new stamp remains eligible after reach.

The preferred onboard command starts the bridge and appropriate pose stack as
well:

```bash
ros2 launch waverover robot.launch.py
```

The controller can also be launched alone:

```bash
ros2 launch waverover_controller waypoint_controller.launch.py \
  robot_name:=29 control_mode:=fixed_wing pose_source:=MCS
```

For MCS robot 29, a waypoint can be queued with:

```bash
ros2 topic pub --once /waverover_29/waypoints \
  geometry_msgs/msg/PointStamped \
  "{header: {frame_id: robotics_lab}, point: {x: 1.0, y: 2.0, z: 0.0}}"
```

For SLAM, use `frame_id: waverover_29/map` instead. Waypoints append to an
in-memory FIFO. Before the first waypoint both modes stop. After the queue
completes, twist mode stops and fixed-wing mode loiters; a new waypoint exits
loiter. TF or MCS pose failure leaves the queue unchanged.

`config/waypoints.yaml` is an optional controller-tuning-only override. Supply
it with `params_file:=...`; robot identity, modes, topics, frames, and MCS
settings remain central or explicit launch arguments.
