# WaveRover terminal waypoint sender

Run this separately on the operator laptop:

```bash
ros2 launch waverover_waypoint_ui waypoint_ui.launch.py
```

It reads `robot_name` and `pose_source` from the central configuration. A
temporary override must match the onboard stack, for example:

```bash
ros2 launch waverover_waypoint_ui waypoint_ui.launch.py \
  robot_name:=29 pose_source:=MCS
```

SLAM waypoints use `robot_<ID>/map`; MCS waypoints use the configured MCS
frame, normally `robotics_lab`. The prompt shows the selected source, frame,
and destination topic.

Commands:

```text
1.0 2.0          # send to the currently selected robot
30 1.0 2.0       # select robot 30 and send
robot 29         # change robot without sending
status           # show destination and recent sends
help
quit
```

For robot 30 the cached reliable publisher writes
`geometry_msgs/msg/PointStamped` to `/robot_30/waypoints`. Message timestamps
come from the local ROS clock. The application uses the controlling terminal,
so it works in a local shell, VS Code terminal, or interactive SSH session
without an X display.
