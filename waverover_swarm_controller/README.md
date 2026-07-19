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

### Conservative optimization separation model

All optimization controllers constrain every unordered rover pair at every
future prediction step, independently of the communication edges. Centralized
convex and centralized MPC fix a separating-plane normal from the measured
current positions, `n_ij = (p_i - p_j) / ||p_i - p_j||`, and impose
`n_ij.T @ (x_i(k) - x_j(k)) >= minimum_separation + 0.001 m`. This is affine
and therefore remains compatible with CVXPY disciplined convex programming.
The superficially direct constraint
`norm(x_i(k) - x_j(k)) >= minimum_separation` is not used: a convex norm on
the greater-than side makes the feasible set non-convex and violates DCP.

Distributed MPC uses the same fixed current-position directions but divides
the available projected closing distance equally between both agents. Each
agent may consume at most
`(||p_i-p_j|| - minimum_separation - 0.001 m) / 2` toward the other rover at
each future step. Both agents applying their corresponding half-budget means
simultaneous motion cannot consume more than the available projected margin.
Fiedler edges remain communication/connectivity constraints only; collision
limits cover all rover pairs. Coincident rovers, initially unsafe centralized
pairs, and negative distributed closing budgets fail explicitly because there
is no safe direction or margin to infer.

The fixed separating directions are conservative. In particular, longer MPC
horizons cannot plan a side-switch around another rover even when a nonlinear
collision-free route exists. First-waypoint connectivity/lookahead
post-processing is copied into prediction index 1 so safety, visualization,
and dispatch use the same point; later optimized points are left unchanged,
which can make the first path segment less representative of the optimizer's
original dynamics. The independent post-solve safety validator remains
authoritative and fails closed on any current, immediate, or predicted-path
violation. Solver metadata from a rejected result may be shown in diagnostics
as `controller_result_state=rejected`, but that result is never made
commandable or published as a valid prediction.

These controllers remain first-draft, position-level convex approximations.
They are not the complete nonlinear fixed-wing formulation described by the
paper or thesis, and this package does not claim full paper fidelity.

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

| Parameter                |            Default |
| ------------------------ | -----------------: |
| Straight speed           |   `0.333333 m/s` |
| Turn radius              |         `0.15 m` |
| Bank yaw rate            | `2.513274 rad/s` |
| Turning-path speed       |   `0.376991 m/s` |
| Outer control/MPC period |          `1.0 s` |
| MPC maximum step         |     `0.333333 m` |
| Minimum MPC lookahead    |         `0.30 m` |
| MPC horizon              |              `5` |

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
  python3-cvxpy python3-matplotlib python3-yaml \
  ros-jazzy-ros2bag ros-jazzy-rosbag2-py \
  ros-jazzy-rosbag2-storage-default-plugins \
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

| Purpose              | Topic/service                            |
| -------------------- | ---------------------------------------- |
| MCS input            | `/macortex_bridge/waverover_<ID>/pose` |
| FIFO waypoint output | `/waverover_<ID>/waypoints`            |
| Reliable stop        | `/waverover_<ID>/end_trial`            |
| Arm/disarm           | `/waverover_swarm/arm` (`SetBool`)   |
| Diagnostics          | `/waverover_swarm/diagnostics`         |
| Markers              | `/waverover_swarm/markers`             |
| Predicted path       | `/waverover_swarm/predicted_path/waverover_<ID>` |

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

## Synthetic MCS poses for PC-only tests

`synthetic_mcs` publishes synchronized `PoseStamped` inputs for every rover ID
in an experiment file. Backward-compatible `static` mode places multiple
rovers at deterministic, equally spaced points on a station-centered circle
and places a single rover at the station. Dynamic modes integrate only the
calibrated positive-forward-speed primitives from the experiment: straight,
bank left, and bank right. The constant-turn-rate equations are integrated at
the fixed `1/rate_hz` timestep rather than using ROS timer jitter.

Supported modes are:

- `static`: unchanged fixed circular formation.
- `scripted`: repeat YAML `action`/`duration_sec` segments.
- `preset`: `circle`, `racetrack`, or `figure_eight` scripts.
- `random_walk`: seeded random actions and bounded segment durations.
- `noisy_path`: a seeded script/preset with segment variation, bounded process
  noise, and separately sampled MCS measurement noise.

`formation_coupling: rigid` (the backward-compatible default) gives every
rover the same translation and heading evolution, preserving formation
offsets. `formation_coupling: independent` gives each rover explicit position,
heading, segment phase, action, speed, yaw rate, history, and RNG state. A
SHA-256 digest of the actual master seed and complete string rover ID derives
each stream, so YAML ID ordering does not affect a rover's trajectory.

Every tick is an atomic global candidate: either all rover states pass finite,
geofence, and minimum-separation checks and commit, or none do. Near another
rover, a bounded deterministic search tries calibrated positive-speed bank
alternatives and ranks them using interior clearance and a short constant-bank
projection. It reserves three turn radii so corrections begin before the hard
collision boundary. It never stops, reverses, clamps, or teleports a rover.
Exhausting `maximum_transition_attempts` stops publication and makes the
coordinator fail stale. Observation resampling never alters true state.

`connectivity_policy: enforce` (the default) also rejects a disconnected true
or observed station/swarm graph. `observe` permits disconnection while still
enforcing collision and geofence rules; lambda_2, component count, station
reachability, outages, and graph churn remain recordable.

This tool publishes **poses only**: it has no waypoint, `end_trial`, `cmd_vel`,
or wheel-command publisher. It is exclusively for operator-PC development and
smoke tests. **Never use synthetic MCS as localization during physical rover
motion.**

In addition to observed MCS poses, it publishes ground truth and realized
motion on
`/waverover_swarm/synthetic/ground_truth/waverover_<ID>` and
`/waverover_swarm/synthetic/motion/waverover_<ID>`. Canonical JSON metadata on
`/waverover_swarm/synthetic/metadata` records schema version, actual seed,
fixed timestep, actual radius, coupling/policy, per-rover seeds/states/segments,
true and observed graph/separation metrics, and rejection/correction causes.
Schema 2 metadata is periodically republished with transient-local durability;
offline loading remains tolerant of schema 1 bags.

```bash
ros2 launch waverover_swarm_controller synthetic_mcs.launch.py \
  config_file:=/home/derder/ros2_ws/src/waverover_swarm_controller/config/smoke_test_6.yaml \
  rate_hz:=20.0 \
  radius_m:=0.75 \
  angle_offset_rad:=0.0 \
  yaw_rad:=0.0
```

The launch arguments are `config_file`, `rate_hz`, `radius_m`,
`angle_offset_rad`, and `yaw_rad`. The node runs in the `/waverover_swarm`
namespace while its MCS outputs use the canonical absolute fleet topics. Omit
`radius_m` to use `synthetic_mcs.initial_radius_m`; an explicit launch value
wins, and old YAML without the field defaults to 0.5 m.

Existing files without `synthetic_mcs` remain static. A dynamic example is:

```yaml
synthetic_mcs:
  mode: noisy_path
  preset: figure_eight
  seed: 2026                 # null selects OS entropy and logs the actual seed
  duration_sec: 60.0
  formation_coupling: independent
  initial_radius_m: 1.0
  connectivity_policy: observe
  segment_duration_min_sec: 1.0
  segment_duration_max_sec: 5.0
  process_speed_std_mps: 0.005
  process_yaw_rate_std_rad_s: 0.01
  measurement_position_std_m: 0.002
  measurement_heading_std_rad: 0.005
  maximum_transition_attempts: 50
```

Identical configuration, seed, initial formation, and rate produce identical
per-ID true and observed samples. A null seed is replaced from operating-system
entropy and the concrete actual seed is recorded. The loader also verifies that
`turning_path_speed_mps / bank_yaw_rate_rad_s` agrees with `turn_radius_m`
within five percent.

## Target YAML visualizer

Plot any valid targets file interactively, optionally overlaying the station,
geofence, and ideal/maximum communication ranges from an experiment:

```bash
ros2 run waverover_swarm_controller visualize_targets \
  /home/derder/ros2_ws/src/waverover_swarm_controller/config/targets_smoke_6.yaml \
  --experiment-file /home/derder/ros2_ws/src/waverover_swarm_controller/config/smoke_test_6.yaml
```

For SSH or headless WSL, select the noninteractive backend with `--no-show`:

```bash
ros2 run waverover_swarm_controller visualize_targets \
  /home/derder/ros2_ws/src/waverover_swarm_controller/config/targets_smoke_6.yaml \
  --experiment-file /home/derder/ros2_ws/src/waverover_swarm_controller/config/smoke_test_6.yaml \
  --output /tmp/targets_smoke_6.png \
  --no-show
```

Output formats are selected by a `.png`, `.pdf`, or `.svg` extension. Use
`--title` to override the plot title.

## Recorded experiments

`run_experiment` is a supervisor separate from the safety-critical
coordinator. It starts rosbag first, publishes versioned `BEGIN`, launches the
optional synthetic publisher and coordinator, and records without arming. It
always resolves the run configuration with `dry_run: true`.

```bash
ros2 run waverover_swarm_controller run_experiment \
  --config /home/yacin/waverover/src/waverover_swarm_controller/config/dynamic_smoke_test_6.yaml
```

The XDG-compatible default root is `~/.local/share/waverover/runs`; override it
with `recording.root_directory`. Runs are never overwritten:

```text
runs/2026-07-18/
  20260718T120000Z_mpc_distributed_synthetic_2026_a1b2c3/
    manifest.yaml
    working_tree.patch
    config/{experiment.yaml,targets.yaml,rosbag_qos_overrides.yaml}
    bag/recording/
    logs/
    analysis/
```

The manifest is atomically updated through starting, running, and
completed/interrupted/failed states. It includes configuration copies, actual
seed, host/runtime/git metadata, exact argument-list child commands, topics,
storage plugin, exit codes, and failure reason. Dirty worktrees are allowed and
their patch is saved. Ctrl-C/SIGTERM publishes `STOP_REQUESTED`, stops the
experiment nodes, publishes `END` while rosbag remains active, flushes rosbag,
and finalizes the manifest.

`recording.profile: core` records events, synthetic metadata, controller
telemetry, diagnostics, poses, ground truth/motion, predicted paths, waypoints,
`cmd_vel`, `end_trial`, TF, parameter events, and ROS logs. `full` additionally
records IMU, odometry, lidar, and maps. Full-profile lidar and map data can be
large. Missing topics do not prevent recording and may appear later during a
physical experiment. SQLite3 is the default storage plugin.

## Machine-readable controller telemetry

Every completed coordinator cycle publishes canonical schema-versioned JSON on
`/waverover_swarm/controller_telemetry`. It contains measured poses/headings,
station and targets, setpoints, active/pending waypoints, predicted paths,
selected edges, explicit target assignments when the algorithm has them,
solver state/timing, valid/rejected/faulted state, separation pairs/steps,
binary and weighted connectivity, pose ages/skew, stop reason, and latest
handoff. Telemetry publication is observability-only: an exception while
publishing it cannot validate a controller result or enable commands.

## Offline analysis and comparisons

Analyze a run without `ros2 topic echo`:

```bash
ros2 run waverover_swarm_controller analyze_run <run-directory>
ros2 run waverover_swarm_controller compare_runs <run-root-or-run> [more-runs...]
```

The rosbag2 reader selectively deserializes analysis topics and does not load
lidar/maps. `BEGIN`/`END` define elapsed time when available; bag timestamps are
an explicitly reported fallback. Outputs are `summary.yaml`, `summary.json`,
`timeseries.csv`, `events.csv`, `metrics_over_time.png`,
`metric_distributions.png`, `trajectories.png`, and `report.md`.

The instantaneous mission-cost convention is

```text
J = sum_(stored undirected edge once) max(ideal_range, edge_distance)
    + sum_(assigned rover,target) target_weight * target_distance
```

Stored target assignments produce the exact implemented assignment cost.
Algorithms without explicit gamma/assignment produce a clearly labeled
nearest-target proxy, never an “exact paper cost.” Main-target distance reports
assigned-rover distance and the minimum-any-rover proxy separately. Binary
`lambda_2` exactly matches the package maximum-range graph. Weighted
connectivity uses the requested logistic weight with configured alpha (default
5.0), includes the station, and is zero beyond maximum range. Reports also
include computation deadlines/statuses, connectivity outages/components,
station reachability, separation and pairwise-distance distributions, graph
churn, per-rover and aggregate path length, speed, yaw rate, primitive
adherence, true/observed pose error, pose rate, tracking error, and explicit
nulls/warnings for unavailable quantities.

**Interpretation warning:** synthetic trajectories are open loop and ignore
controller waypoints. Actual/current lambda_2 is therefore exogenous scenario
data, not closed-loop controller performance. Controllers can produce
different predicted paths and predicted connectivity, but none can change the
recorded synthetic positions. These position-level fixed-wing-like primitives
are useful first-draft stress inputs, not the complete nonlinear aircraft
formulation from the paper or thesis.

Comparison recursively discovers completed runs and separates incompatible
target layouts, communication ranges, rover counts, or scenarios rather than
silently pooling them. It aggregates means/standard deviations across seeds
for mission cost, target distance, connectivity, solve time, outages,
separation, deadline misses, path length, and tracking error.

## Interactive 2D replay

```bash
ros2 run waverover_swarm_controller replay_run <run-directory>
ros2 run waverover_swarm_controller replay_run <run-directory> \
  --no-show --time 10.0 --output analysis/frame_10s.png
```

Replay renders the arena/geofence, station, weighted targets, every rover with
its own recorded heading,
recent trails, generated/active/pending points, predicted paths, selected
edges, the actual range graph with quality colors, optional communication and
minimum-separation circles, connectivity, solver/stop state, and elapsed and
remaining time. Controls provide play/pause, seek, step, 0.25x–4x speed, and
layer toggles. Position is linearly interpolated and heading takes the shortest
continuous path through ±pi.

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

Markers show station, targets, setpoints, and selected edges. MPC controller
variants publish predicted trajectories as `nav_msgs/msg/Path`; the heuristic
controllers do not produce or publish predicted paths. Useful recording
command:

```bash
ros2 bag record \
  /waverover_swarm/diagnostics /waverover_swarm/markers \
  /waverover_swarm/predicted_path/waverover_131 \
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

## Experiment-pipeline limitations

Synthetic trajectories are open loop and do not respond to controller
setpoints or rover commands. Targets are currently static. The controllers are
position-level approximations, and telemetry/analysis evaluates sampled states
and prediction steps; discrete separation results do not establish
continuous-time physical safety between samples. Recorded `cmd_vel`, tracking,
or delay metrics are unavailable in dry-run when no commands exist and are
reported as unavailable rather than zero. Neither replay nor offline analysis
is part of the authoritative online safety decision.
