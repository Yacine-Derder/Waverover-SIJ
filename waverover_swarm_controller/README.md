# WaveRover swarm controller (operator PC only)

`waverover_swarm_controller` computes coordinated setpoints from external MCS
poses and sends them through the existing per-rover waypoint controllers. The
package is present and buildable in the shared repository on every machine,
but it is an **operator-PC application**. No rover launch file or service
starts it, and it must never be added to `robot.launch.py`, `slam.launch.py`,
or a rover systemd unit.

This is experimental research software. It is not ready for unattended
operation.

## Architecture

The coordinator loads robot IDs from an explicit PC experiment file; it never
uses a rover's local identity. It subscribes directly to each configured
`/macortex_bridge/waverover_<ID>/pose`, validates and synchronizes the
`robotics_lab` poses, creates one immutable swarm snapshot, and passes that
snapshot to a pure controller. Pure controllers return setpoints, predicted
paths, selected communication edges, solver state, timing, and diagnostics;
they do not import or publish ROS messages.

After conservative safety validation, an event-triggered dispatcher maintains
at most one active rover waypoint and one replaceable pending setpoint in PC
memory. It publishes only to `/waverover_<ID>/waypoints`; the existing onboard
waypoint controller remains the only autonomous `cmd_vel` publisher.

The PC exposes `/waverover_swarm/arm` as `std_srvs/srv/SetBool`. Diagnostics,
markers, and predicted paths are under `/waverover_swarm`.

## Algorithms

Five controllers were ported from `Yacine-Derder/drone-simulator-master`:

- `heuristic`: main-target relay chain, deterministic secondary clustering,
  required-relay calculation, and Hungarian robot/position assignment.
- `heuristic_decentralized`: PC-hosted target-aware agents with a distinct
  per-agent computation boundary and local relay-chain decisions. It does not
  add ROS traffic between agents yet.
- `convex`: centralized target assignment and position optimization with a
  tested connectivity-safe output projection equivalent to the simulator's
  `ConnectedDrone` carrot restriction.
- `mpc_centralized`: the centralized convex problem over a configurable
  horizon; only the first future carrot is returned to the dispatcher.
- `mpc_distributed`: local per-agent solvers, Fiedler-vector edge selection,
  predicted neighbor trajectories, and first-future-position output.

Distributed MPC defaults to immutable Jacobi updates: every agent sees the
same previous-cycle predictions. `distributed_update_semantics: gauss_seidel`
reproduces the simulator's sequential update choice where practical.

The simulator is the executable baseline. Deliberate hardware-safety fixes are
documented and tested: coincident/short-distance relay calculations cannot
divide by zero; zero-adjacency Fiedler selection returns no edges rather than
NaNs; missing neighbors and removed string robot IDs are handled explicitly;
solver failures never fall back to another algorithm; and unused `cvxpygen`
code generation was removed. Deterministic string IDs replace simulator class
counters and all iteration order is canonical. The decentralized implementation
retains local chain behavior but removes simulator display/color/battery state.

## Measured rover surrogate

The default model uses measurements from 2 m straight travel in 6 s and a
0.30 m diameter circle completed in 2.5 s:

| Parameter | Default |
| --- | ---: |
| Straight speed | `0.333333 m/s` |
| Turn radius | `0.15 m` |
| Bank yaw rate | `2.513274 rad/s` |
| Turning-path speed | `0.376991 m/s` |
| Outer control/MPC period | `1.0 s` |
| MPC maximum step | `0.333333 m` |
| Minimum MPC lookahead | `0.30 m` |
| MPC horizon | `5` |

Turning-path speed differs slightly from straight speed. This is a calibrated
hardware surrogate for the rover's fixed-wing-like straight/bank controller,
not an exact aircraft model. The onboard `0.15 m` goal tolerance is unchanged.

## Configuration and targets

Copy `config/experiment.example.yaml` to a PC experiment-specific file. It
defines the `robotics_lab` frame, explicit rover IDs, pose freshness/skew,
station, controller, vehicle, dispatcher, communication, dry-run, separation,
and geofence settings. `targets_file` is resolved relative to the experiment
file unless it is absolute.

`config/targets.yaml` is an installed editable example:

```yaml
frame_id: robotics_lab
main_target_id: target_main
targets:
  - id: target_main
    position: [2.5, 0.0]
    weight: 10.0
```

IDs are stable strings and need not be contiguous integers. The loader
requires unique nonempty IDs, exactly one main target, finite positions and
weights, nonnegative weights, and positions inside the experiment geofence.
The supplied target coordinates, `1.5/2.0 m` communication ranges, and
`[-4, 4] m` geofence are illustrative. Verify them against the actual
`robotics_lab` origin, floor area, MCS calibration, radio behavior, and rover
footprints before arming.

## Operator-PC dependencies

The package deliberately uses lazy imports for PC-only optimization modules.
It remains discoverable and buildable on a rover without them; selecting an
unavailable controller produces a clear arming/runtime error and publishes no
waypoint. `cvxpygen` is neither used nor required.

On an Ubuntu 24.04 ROS 2 Jazzy operator PC:

```bash
sudo apt update
sudo apt install \
  python3-numpy python3-scipy python3-sklearn python3-networkx \
  python3-cvxpy python3-yaml \
  ros-jazzy-diagnostic-msgs ros-jazzy-geometry-msgs ros-jazzy-nav-msgs \
  ros-jazzy-std-msgs ros-jazzy-std-srvs ros-jazzy-visualization-msgs

cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select \
  waverover waverover_swarm_controller
source install/setup.bash
```

## Dry-run, arming, and topics

Always begin in dry-run:

```bash
ros2 launch waverover_swarm_controller swarm_controller.launch.py \
  config_file:=<absolute path to experiment.example.yaml> \
  algorithm:=heuristic \
  dry_run:=true
```

Dry-run computes controllers, validates safety, and publishes diagnostics and
visualization, but never publishes a waypoint or startup `end_trial`. It cannot
be armed. Algorithm changes require stopping/disarming and relaunching.

| Purpose | Topic/service |
| --- | --- |
| MCS input | `/macortex_bridge/waverover_<ID>/pose` |
| FIFO waypoint output | `/waverover_<ID>/waypoints` |
| Reliable stop | `/waverover_<ID>/end_trial` |
| Arm/disarm | `/waverover_swarm/arm` (`SetBool`) |
| Diagnostics | `/waverover_swarm/diagnostics` |
| Markers | `/waverover_swarm/markers` |
| Predicted path | `/waverover_swarm/predicted_path/<ID>` |

The eventual physical-experiment command is:

```bash
# PHYSICAL MOTION: only after coordinates/geofence are verified, every listed
# rover has a fresh MCS pose, and restrained testing has succeeded.
ros2 launch waverover_swarm_controller swarm_controller.launch.py \
  config_file:=/absolute/path/to/verified-experiment.yaml \
  algorithm:=heuristic \
  dry_run:=false

ros2 service call /waverover_swarm/arm std_srvs/srv/SetBool "{data: true}"
ros2 service call /waverover_swarm/arm std_srvs/srv/SetBool "{data: false}"
```

Arming fails unless every configured pose is present, valid, fresh, and
synchronized; configuration/targets are valid; the selected controller and
solver are available; a complete fresh result exists; and all initial
setpoints and predicted paths pass frame, numeric, geofence, edge, and
separation checks.

## FIFO-safe dispatch and stopping

Each controller cycle only replaces the latest PC-side pending point. A point
is published when no active waypoint exists, or after MCS shows the active
point within `0.15 m` continuously through the `0.15 s` handoff delay. The
delay lets the onboard 10 Hz FIFO controller remove the reached point. MPC
paths are never published as waypoint sequences. A carrot shorter than
`0.30 m` is extended in its finite nonzero direction, up to the `0.333333 m`
step limit; a zero direction faults instead of generating NaNs.

An active waypoint older than `10 s` faults the whole experiment and requires
explicit rearming. Explicit disarm, graceful exit, Ctrl-C, SIGTERM, stale pose,
invalid solver output, geofence/separation failure, or timeout stops new
dispatch and sends one reliable `Empty` to every rover this process actually
commanded. ROS is briefly drained before teardown and cleanup is idempotent.
Uncommanded rovers receive no `end_trial`.

A hard PC power loss or `kill -9` cannot execute cleanup. Because rover-side
code is intentionally unchanged, an already active waypoint may still be
completed before the onboard controller loiters. This is a known safety
limitation; use a physical emergency-stop process independent of this PC.

## Visualization and recording

Markers show station, targets, setpoints, and selected edges. MPC paths are
published as `nav_msgs/msg/Path`. Useful recording command:

```bash
ros2 bag record \
  /waverover_swarm/diagnostics /waverover_swarm/markers \
  /waverover_swarm/predicted_path/131 \
  /macortex_bridge/waverover_131/pose \
  /waverover_131/waypoints /waverover_131/end_trial
```

Extend the list for every configured rover. Keep MCS and coordinator clocks in
the bag; pose safety itself uses local monotonic receipt time.

## Staged physical validation

First restrained-rover test:

1. Create a one-rover configuration with a verified, spacious geofence and a
   target more than `0.30 m` away but safely reachable.
2. Physically lift/restraint-test the rover so wheels cannot move the chassis;
   keep an independent emergency stop ready.
3. Run dry-run for several minutes. Confirm frame, pose age, setpoint, and
   geofence diagnostics and inspect RViz markers.
4. Relaunch with `dry_run:=false`, arm once, observe exactly one waypoint, then
   disarm and verify `/waverover_<ID>/end_trial` and zero wheel motion.
5. Only then perform a short clear-floor test at low operational risk.

Staged multi-rover test:

1. Repeat dry-run with two stationary rovers and verify separation rejection.
2. Restrained-test two rovers, then run a short two-rover clear-floor trial.
3. Add one rover at a time, re-verifying MCS skew, geofence, communication
   edges, minimum separation, dispatcher handoffs, and end-trial delivery.
4. Compare heuristic variants before enabling convex/MPC solvers; inspect every
   predicted path and solver status. Never progress after a fault without
   identifying its cause and explicitly rearming.
