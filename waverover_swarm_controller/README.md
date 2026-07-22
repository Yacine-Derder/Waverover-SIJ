# WaveRover swarm controller (operator PC only)

The default target switch period is 20 seconds. Real dispatch treats 0.5 m
separation as a best-effort preference and sends every algorithm through the
same deterministic, geofence-aware repair stage. Collision warnings and
residual violations are telemetry, not trial-latching faults; structural
validation remains fail-closed. Exact acknowledgements enable same-epoch
completed-destination hysteresis (0.05 m match, 0.30 m drift reissue), while
exact `waypoint_failed` messages clear only the affected rover non-fatally.

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
The active point is refreshed at the configured monotonic interval (default
`1.0 s`). The rover controller coalesces consecutive coordinate-equivalent
refreshes, so they do not grow its FIFO.

Each newly activated logical waypoint receives a unique `header.stamp` token.
Refreshes reuse it exactly. When the onboard controller removes that waypoint,
it echoes the original frame, coordinates, and stamp on the reliable
`/waverover_<ID>/waypoint_reached` topic. Only an exact robot/frame/token/point
match clears PC active state and promotes the newest pending result. A bounded
onboard reached-token cache prevents a delayed refresh from requeueing an
acknowledged command. MCS distance is not a handoff signal; a missing
acknowledgement only produces the non-fatal overdue warning.

There is no arming service or armed state. In live mode, fresh synchronized
poses and a valid controller result immediately enable waypoint publication.
Diagnostics, markers, and predicted paths are under `/waverover_swarm`.

## Algorithms

Five controllers were ported from `Yacine-Derder/drone-simulator-master`:

- `heuristic`: current-priority relay chain, deterministic background clustering,
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
  predicted neighbor trajectories, all-target team allocation, and
  first-future-position output. Selected non-station neighbors suppress target
  coefficients when they are better positioned; coefficients are divided by
  relay burden. The configurable inter-agent weight defaults to the
  simulator's `2.0`, while target priorities come directly from snapshot
  weights (normally `10.0` priority and `1.0` background).

Distributed MPC defaults to immutable Jacobi updates: every agent sees the
same previous-cycle predictions. `distributed_update_semantics: gauss_seidel`
reproduces the simulator's sequential update choice where practical.
Telemetry records effective coefficients, dominant targets, selected
neighbors, burden, objective contributions, update semantics, and local
statuses for each agent.

Recovery connectivity slack is penalized by
`controller.connectivity_recovery_slack_penalty` (default `10000.0`). The
penalty is configurable without changing the hard communication radius.

The three optimization controllers use a never-fail hierarchy: normal solve,
connectivity-slack recovery solve, deterministic connectivity recovery, then
measured-position safe hold. A failed or undispatchable hold is diagnosed but
does not fault the dispatcher or publish `end_trial`; a later cycle may return
to its normal solve.

Convex and MPC connectivity constraints use exactly
`maximum_range_m - 2 * turn_radius_m`. Recovery adds one nonnegative slack per
selected edge and prediction step and minimizes their configured penalty
before nominal target/link effort. Deterministic recovery combines every
violated selected-edge direction from the same immutable snapshot and clips
each rover's combined displacement to `mpc_max_step_m`. It reuses solver edges
and constructs a station-rooted replacement only when none are usable.

Post-processing has one owner (the coordinator): it checks a complete finite
mapping, applies bounded geofence/connectivity/movement projections, repairs
first-step crossings and endpoint/active-waypoint separation, reconciles those
constraints for a bounded number of iterations, then performs independent
final validation. Movement projection is last in each reconciliation pass, so
the physical step is never exceeded; any link or separation constraint that
cannot simultaneously be met is reported rather than silently hidden.

Every cycle produces a structured execution outcome. Pending updates and
dispatcher ticks require `dispatch_allowed`, a complete command set, and final
validation simultaneously. Temporary missing/stale pose snapshots produce
`pose_unavailable`, preserve existing active commands without refreshing them,
and automatically retry on the next cycle.

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
The fixed station/GCS remains a communication-graph node but is excluded from
rover-assignable relay positions. Surplus centralized-heuristic rovers hold
their measured position instead of receiving the former station-position
fallback; a rover for which no non-station setpoint exists fails closed.

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

`config/experiment.yaml` is the canonical experiment configuration. Copy it
with its referenced targets file only when site-specific geometry is needed. It
defines the `robotics_lab` frame, explicit rover IDs, pose freshness/skew,
station, controller, vehicle, dispatcher, communication, dry-run, separation,
and geofence settings. `targets_file` is resolved relative to the experiment
file unless it is absolute.

Controller selection is strict and centralized in this file. `controller.common`
contains shared timing and seed values, while `controller.algorithms` contains
validated blocks for `heuristic`, `heuristic_decentralized`, `convex`,
`mpc_centralized`, and `mpc_distributed`. Unknown keys and missing selected
algorithm parameters are errors.

`config/targets.yaml` is an installed editable example:

```yaml
frame_id: robotics_lab
targets:
  - id: target_0
    position: [2.5, 0.0]
  - id: target_1
    position: [0.0, 2.5]
```

Target layouts contain only neutral IDs and static positions. Runtime priority
belongs to the experiment configuration:

```yaml
target_dynamics:
  mode: random_priority
  switch_period_sec: 20.0
  priority_weight: 10.0
  background_weight: 1.0
  seed: 2026
  initial_priority_target_id: null
  avoid_immediate_repeat: true
```

IDs are sorted before selection, so YAML order cannot change the sequence.
Selection uses a dedicated deterministic RNG, absolute monotonic 10-second
epochs, missed-boundary catch-up, and no consecutive repeat when two or more
targets exist. `run_experiment --seed` derives independent synthetic-motion
and target-selection seeds with stable domain-separated hashing and records
both in the versioned manifest. The loader requires unique nonempty IDs,
finite positions inside the geofence, positive finite periods and weights,
and `priority_weight >= background_weight`. Legacy `main_target_id`, static
weights, `reached_distance_m`, and `handoff_delay_sec` still parse with
deprecation warnings but do not control new runtime handoff/priority behavior.
The supplied target coordinates, `1.5/2.0 m` communication ranges, and
`[-4, 4] m` geofence are illustrative. Verify them against the actual
`robotics_lab` origin, floor area, MCS calibration, radio behavior, and rover
footprints before enabling live dispatch.

## Operator-PC dependencies

The package deliberately uses lazy imports for PC-only optimization modules.
It remains discoverable and buildable on a rover without them; selecting an
unavailable controller produces a clear runtime error and publishes no
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
  ros-jazzy-std-msgs ros-jazzy-visualization-msgs

cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select \
  waverover waverover_swarm_controller
source install/setup.bash
```

## End-to-end operator-PC test run

The following commands assume the repository is
`/home/derder/ros2_ws/src` and the workspace is
`/home/derder/ros2_ws`. Use one shell per numbered terminal. Every shell must
use the same ROS domain and middleware settings as the MCS bridge and, during a
physical test, the rovers.

### 1. Build and create a local test configuration

Run once after pulling a new revision:

```bash
cd /home/derder/ros2_ws
source /opt/ros/jazzy/setup.bash

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select \
  waverover waverover_swarm_controller
```

Keep experiment-specific coordinates outside the Git checkout:

```bash
mkdir -p /home/derder/waverover_experiments
cp /home/derder/ros2_ws/src/waverover_swarm_controller/config/experiment.yaml \
  /home/derder/waverover_experiments/pc_test.yaml
cp /home/derder/ros2_ws/src/waverover_swarm_controller/config/targets_setup_1.yaml \
  /home/derder/waverover_experiments/targets_setup_1.yaml

nano /home/derder/waverover_experiments/pc_test.yaml
```

At minimum, verify `robot_ids`, `station.position`,
`communication.*_range_m`, `safety.preferred_separation_m`, and the complete
`safety.geofence`. Edit target coordinates and weights in
`/home/derder/waverover_experiments/targets_setup_1.yaml`. The relative
`targets_file: targets_setup_1.yaml` entry remains valid because both copied
files are in the same directory.

Use this setup block at the start of every terminal:

```bash
source /opt/ros/jazzy/setup.bash
source /home/derder/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

Change these networking values if the MCS bridge and rover fleet use different
ones.

### 2. Fake-MCS computation test (no waypoint publication)

Terminal 1 publishes the fake poses:

```bash
source /opt/ros/jazzy/setup.bash
source /home/derder/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

ros2 launch waverover_swarm_controller synthetic_mcs.launch.py \
  config_file:=/home/derder/waverover_experiments/pc_test.yaml \
  rate_hz:=20.0 \
  radius_m:=0.75
```

Terminal 2 starts the coordinator in dry-run:

```bash
source /opt/ros/jazzy/setup.bash
source /home/derder/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

ros2 launch waverover_swarm_controller swarm_controller.launch.py \
  config_file:=/home/derder/waverover_experiments/pc_test.yaml \
  algorithm:=heuristic \
  dry_run:=true
```

Terminal 3 verifies input and generated results:

```bash
source /opt/ros/jazzy/setup.bash
source /home/derder/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

ros2 topic hz /macortex_bridge/waverover_131/pose
```

Stop `topic hz` with Ctrl-C, then inspect either diagnostics or the
machine-readable result:

```bash
ros2 topic echo /waverover_swarm/diagnostics
ros2 topic echo /waverover_swarm/controller_telemetry
```

Only run one blocking `ros2 topic echo` command at a time. A healthy cycle
reports fresh poses, `controller_result_state=valid`, a valid solver status,
and per-rover `pending_<ID>` setpoints. Dry-run deliberately publishes no
messages on `/waverover_<ID>/waypoints` or `/waverover_<ID>/end_trial`.

The `algorithm` launch value may be `heuristic`,
`heuristic_decentralized`, `convex`, `mpc_centralized`, or
`mpc_distributed`. Stop and relaunch the coordinator to change algorithms.
An omitted/empty launch argument uses `controller.algorithm`; a nonempty value
overrides it for that coordinator process.

### 3. Fake-MCS waypoint-publication test with rovers off

Keep Terminal 1 running. Stop the dry-run coordinator in Terminal 2 and
relaunch it with command publication enabled:

```bash
ros2 launch waverover_swarm_controller swarm_controller.launch.py \
  config_file:=/home/derder/waverover_experiments/pc_test.yaml \
  algorithm:=heuristic \
  dry_run:=false
```

Before starting the live coordinator, use Terminal 3 to watch a rover's output:

```bash
ros2 topic echo /waverover_131/waypoints
```

As soon as all configured poses are fresh and synchronized and the controller
returns a safety-valid result, one active waypoint is published per configured
rover. Inspect other rover topics by replacing `131`. Stop the coordinator
with Ctrl-C when finished; cleanup publishes `end_trial` to commanded rovers.

With stationary fake poses or real rovers that are turned off, the dispatcher
refreshes the exact active waypoint every `refresh_period_sec` while continuing
to replace only the PC-side pending setpoint. It does not hand off pending until
MCS confirms reach continuously through the handoff delay. After
`active_waypoint_warning_sec` (10 seconds by default), diagnostics and telemetry
show a non-fatal overdue warning; refresh and dispatch continue. The legacy
`maximum_active_time_sec` key is accepted as an alias for this warning threshold.

### 4. Real-MCS test with rovers turned off

Do not run `synthetic_mcs`. The coordinator has no fake/real selector: both
sources publish the same canonical MCS topics, so running the synthetic and real
publishers together would mix two pose sources and invalidate the test.

First verify the real bridge in Terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
source /home/derder/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

ros2 topic list | sort
ros2 topic info /macortex_bridge/waverover_131/pose --verbose
ros2 topic echo /macortex_bridge/waverover_131/pose --once
```

Repeat the last two commands for IDs 132 through 136. Every pose must have
`header.frame_id: robotics_lab`, finite coordinates, a valid quaternion, and
a sufficiently fresh publication rate. The configuration must list exactly the
rover IDs supplied by the MCS; dispatch requires a fresh, synchronized pose for
every listed ID.

Start with the dry-run coordinator in Terminal 2:

```bash
ros2 launch waverover_swarm_controller swarm_controller.launch.py \
  config_file:=/home/derder/waverover_experiments/pc_test.yaml \
  algorithm:=heuristic \
  dry_run:=true
```

Inspect diagnostics and telemetry in Terminal 3. Verify the actual MCS
coordinates against the configured station, targets, communication ranges,
geofence, and minimum separation. If the dry-run remains valid, stop Terminal
2, start a waypoint echo in Terminal 3, then relaunch with `dry_run:=false`.
Rover
processes and waypoint subscribers do not need to be running for the ROS
publisher and a PC-side `ros2 topic echo` subscriber to observe the generated
waypoint.

For a non-publishing verification, remain in `dry_run:=true` and inspect
`pending_<ID>` in diagnostics or `setpoints` in controller telemetry. This
is the preferred first test with real MCS data.

### Configuration map

| What to change | Location |
| --- | --- |
| Participating rovers | `robot_ids` in the experiment YAML |
| Allowed pose age and inter-rover timestamp skew | `pose.timeout_sec`, `pose.maximum_snapshot_skew_sec` |
| Station coordinates | `station.position` |
| Target file | `targets_file` in the experiment YAML |
| Target coordinates | the referenced neutral targets YAML |
| Rover motion model | `vehicle` |
| Control period, MPC horizon/step, seed, distributed semantics | `controller` |
| Handoff tolerance/delay, refresh period, and overdue warning | `waypoint_dispatch` |
| Link ranges | `communication` |
| Minimum separation and allowed floor area | `safety` |
| Fake-pose behavior only | `synthetic_mcs` |
| Runtime controller selection | launch argument `algorithm:=...` |
| Command suppression/publication | launch argument `dry_run:=true/false` |

The `recording.pose_source` and `recording.start_synthetic` fields apply to
the `run_experiment` supervisor. They do not select the pose source for a
standalone `swarm_controller.launch.py` process.

## Dry-run, automatic dispatch, and topics

Always begin in dry-run:

```bash
ros2 launch waverover_swarm_controller swarm_controller.launch.py \
  config_file:=<absolute path to experiment.yaml> \
  algorithm:=heuristic \
  dry_run:=true
```

Dry-run computes controllers, validates safety, records telemetry, and
publishes diagnostics and visualization, but never publishes a waypoint or
`end_trial`. Algorithm changes require stopping and relaunching.

| Purpose              | Topic/service                            |
| -------------------- | ---------------------------------------- |
| MCS input            | `/macortex_bridge/waverover_<ID>/pose` |
| FIFO waypoint output | `/waverover_<ID>/waypoints`            |
| Reliable stop        | `/waverover_<ID>/end_trial`            |
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
```

Live dispatch starts automatically only when every configured pose is present,
valid, fresh, and synchronized; configuration/targets are valid; the selected
controller and solver are available; a complete fresh result exists; and all
setpoints and predicted paths pass frame, numeric, geofence, edge, and
separation checks. Start the waypoint echo before launching live mode.

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
  config_file:=/home/derder/ros2_ws/src/waverover_swarm_controller/config/experiment.yaml \
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
  --experiment-file /home/derder/ros2_ws/src/waverover_swarm_controller/config/experiment.yaml
```

For SSH or headless WSL, select the noninteractive backend with `--no-show`:

```bash
ros2 run waverover_swarm_controller visualize_targets \
  /home/derder/ros2_ws/src/waverover_swarm_controller/config/targets_smoke_6.yaml \
  --experiment-file /home/derder/ros2_ws/src/waverover_swarm_controller/config/experiment.yaml \
  --output /tmp/targets_smoke_6.png \
  --no-show
```

Output formats are selected by a `.png`, `.pdf`, or `.svg` extension. Use
`--title` to override the plot title.

## Recorded experiments

`run_experiment` is a supervisor separate from the safety-critical
coordinator. It starts rosbag first, publishes versioned `BEGIN`, launches the
optional synthetic publisher and coordinator. It preserves `safety.dry_run`
from the experiment YAML: `true` records computed/validated results without
waypoints or `end_trial`; `false` allows automatic validated dispatch.

```bash
ros2 run waverover_swarm_controller run_experiment \
  --config /home/derder/ros2_ws/src/waverover_swarm_controller/config/experiment.yaml
```

Use `--algorithm convex` (or any other supported exact identifier) for a
single-run override. The YAML is not edited. The manifest records configured
and effective algorithms plus whether selection came from config or CLI, and
the run directory uses the effective value.

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
solver state/timing, valid/rejected/faulted state, dry-run and command-enabled
state, per-rover active/publication age, monotonic last-publication time,
refresh count, overdue warning, separation pairs/steps,
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
active point is also republished unchanged every `refresh_period_sec`, using
monotonic scheduling. A refresh does not change active age, reached state,
pending state, or safety state. The onboard controller suppresses consecutive
coordinate-equivalent messages while the target is queued, but accepts that
coordinate again after the reached target has been removed. The handoff delay
lets the onboard 10 Hz FIFO controller remove the reached point. MPC
paths are never published as waypoint sequences. A carrot shorter than
`0.30 m` is extended in its finite nonzero direction, up to the `0.333333 m`
step limit; a zero direction produces a measured-position hold.

An active waypoint older than `active_waypoint_warning_sec` produces a
non-fatal warning only. Graceful exit, Ctrl-C, SIGTERM, stale/incomplete poses,
invalid heuristic output, geofence failure, and malformed values still stop
new dispatch and send one reliable `Empty` to every rover this process actually
commanded. Optimization solve/result failures instead use the never-fail
hierarchy above and never terminate the trial.
Faults after dispatch begins stay latched until restart. ROS is briefly drained
before teardown and cleanup is idempotent. Uncommanded rovers receive no
`end_trial`; dry-run never publishes `end_trial`.

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
4. Start a waypoint echo, relaunch with `dry_run:=false`, observe exactly one
   automatic waypoint, then press Ctrl-C and verify
   `/waverover_<ID>/end_trial` and zero wheel motion.
5. Only then perform a short clear-floor test at low operational risk.

Staged multi-rover test:

1. Repeat dry-run with two stationary rovers and verify separation rejection.
2. Restrained-test two rovers, then run a short two-rover clear-floor trial.
3. Add one rover at a time, re-verifying MCS skew, geofence, communication
   edges, minimum separation, dispatcher handoffs, and end-trial delivery.
4. Compare heuristic variants before enabling convex/MPC solvers; inspect every
   predicted path and solver status. Never progress after a fault without
   identifying its cause and restarting the coordinator.

## Experiment-pipeline limitations

Synthetic trajectories are open loop and do not respond to controller
setpoints or rover commands. Targets are currently static. The controllers are
position-level approximations, and telemetry/analysis evaluates sampled states
and prediction steps; discrete separation results do not establish
continuous-time physical safety between samples. Recorded `cmd_vel`, tracking,
or delay metrics are unavailable in dry-run when no commands exist and are
reported as unavailable rather than zero. Neither replay nor offline analysis
is part of the authoritative online safety decision.
