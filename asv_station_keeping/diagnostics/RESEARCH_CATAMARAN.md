# Research catamaran baseline

## Purpose

`research_catamaran.urdf` is a clean, visible ROS 2 Humble / Gazebo Fortress
baseline for the project. It is intentionally separate from the original
package launch so the known-good model remains available while this model is
developed.

The model contains:

- Two parallel hulls using the original `hull.stl` mesh.
- The original 350 mm hull centre spacing.
- Four diagonal thruster housings using the original `thruster.stl` mesh.
- Explicit port/starboard and fore/rear names.
- Fixed thruster mounting joints.
- Four continuous propeller joints with Fortress thruster-system interfaces.
- A visible deck, two cross beams, payload platform and sensor mast using
  primitive geometry.
- A pool world with water, floor, walls, daylight and a camera aimed at the
  vehicle.
- Fortress joint-state and odometry publishers.
- IMU and NavSat sensors with ROS 2 bridge topics.
- A closed-loop PID station-keeping controller with integral action, command
  slew limiting, attitude protection and CSV metric logging.
- A configurable current-disturbance publisher for dynamic station-keeping
  experiments.
- A live diagnostic status stream containing target, pose, errors, velocities,
  attitude, gains and all four thruster commands.
- A ROS-native RViz visualization layer with target, tolerance, error, current,
  velocity, heading and signed-thruster markers.
- A dependency-light browser dashboard with live plots, editable controller
  parameters, safety state, real-time-factor monitoring and JSON snapshot export.

## Launch

After rebuilding and sourcing the workspace on the ROS 2 Humble computer:

```bash
ros2 launch asv_station_keeping research_catamaran.launch.py
```

The model can be selected explicitly when comparing variants:

```bash
ros2 launch asv_station_keeping research_catamaran.launch.py \
  model_file:=research_catamaran.urdf
```

The launch waits three seconds before spawning the model. This avoids the
race in the older launch where `ros_gz_sim create` could run before Gazebo had
finished creating the world.

The default `research_pool_dynamics.sdf` is the motion and algorithm world. It
starts the model near the simplified hulls' buoyant equilibrium, uses graded
buoyancy, and does not use a rigid water collision plane. The deterministic
contact-surface display world remains available explicitly with:

```bash
ros2 launch asv_station_keeping research_catamaran.launch.py \
  world_file:=research_pool.sdf
```

The dynamics world enables Fortress graded buoyancy. The hulls use
simulation-only displaced-volume boxes, the other visual links do not
contribute extra buoyant collision volume, and a hidden simulation ballast
lowers the centre of mass. The launch starts at `z=-0.10 m`, near the
equilibrium of this simplified volume model. The thruster line is raised to
reduce the surge-induced pitch moment, and hydrodynamic damping is applied to
the physical `base_link`. The hydrodynamics block now declares both linear and
quadratic damping terms explicitly, plus a zero default current, so Fortress
does not have to infer a missing linear-velocity configuration. These are
simulation parameters rather than measured vessel data.

For the controller and metric path:

```bash
ros2 launch asv_station_keeping algorithm_test.launch.py \
  target_x:=0.15 target_y:=0.0 target_yaw:=0.0
```

The default controller run is a position-hold experiment with yaw control
disabled while the diagonal thruster sign convention is being calibrated. To
include the experimental yaw loop:

```bash
ros2 launch asv_station_keeping algorithm_test.launch.py \
  target_x:=0.15 target_y:=0.0 target_yaw:=0.0 \
  enable_yaw_control:=true
```

The controller writes `/tmp/research_catamaran_run.csv`. Evaluate it with:

```bash
ros2 run asv_station_keeping evaluate_research_run \
  /tmp/research_catamaran_run.csv
```

## Dynamic station keeping

The primary research experiment is the closed-loop dynamic station-keeping
launch. It starts the buoyancy world, publishes an optional water current to
the Fortress hydrodynamics system, and starts the PID controller after the
vehicle has spawned and settled:

```bash
ros2 launch asv_station_keeping dynamic_station_keeping.launch.py
```

The default experiment holds the origin against a small `+X` current of
`0.10 m/s`. The controller uses body-frame position error, measured body
velocity feedback, bounded integral action for steady disturbance rejection,
command slew limiting, and roll/pitch protection.

Run a cross-current experiment with a shifted target:

```bash
ros2 launch asv_station_keeping dynamic_station_keeping.launch.py \
  target_x:=0.50 target_y:=0.25 target_yaw:=0.20 \
  current_x:=0.10 current_y:=0.06
```

The controller prints a live status line in the launch terminal. The complete
machine-readable status is also published on:

```text
/research_catamaran/station_keeping/status
```

Display it from another terminal:

```bash
ros2 topic echo /research_catamaran/station_keeping/status
```

The diagnostic values include target, current (`x`, `y`, `z`), pose, roll/pitch/yaw, body
velocities, body-frame errors, RMSE, maximum error, all four thruster commands,
and the active PID gains. The run is logged to:

```text
/tmp/research_dynamic_station_keeping.csv
```

Evaluate it after stopping the experiment with `Ctrl+C`:

```bash
ros2 run asv_station_keeping evaluate_research_run \
  /tmp/research_dynamic_station_keeping.csv
```

Generate machine-readable and browser-readable research artifacts in one pass:

```bash
ros2 run asv_station_keeping evaluate_research_run \
  /tmp/research_dynamic_station_keeping.csv \
  --json-output /tmp/research_dynamic_station_keeping.json \
  --html-output /tmp/research_dynamic_station_keeping.html
```

The HTML report is self-contained and includes position, heading and thruster
plots together with the numerical metrics and input-file provenance.

## Live demonstration console

The complete demonstration stack starts Gazebo Fortress, the controller, the
current disturbance, the RViz visualization and the browser dashboard:

```bash
ros2 launch asv_station_keeping visual_station_keeping.launch.py
```

The dashboard is served at the URL printed by the dashboard node, normally:

```text
http://127.0.0.1:8765
```

The default visual launch enables a two-phase scripted demonstration. It first
commands a shifted station, then changes the target and current after 12 s so
the approach, correction and final holding behavior are visible in Gazebo,
RViz and the dashboard. Press **Run scripted demo** in the dashboard to repeat
the sequence.

The launch opens the default browser when a desktop session is available. To
run without opening a browser or RViz, use:

```bash
ros2 launch asv_station_keeping visual_station_keeping.launch.py \
  open_dashboard:=false start_rviz:=false
```

Run a cross-current live demonstration with a shifted station:

```bash
ros2 launch asv_station_keeping visual_station_keeping.launch.py \
  target_x:=0.50 target_y:=0.25 target_yaw:=0.20 \
  current_x:=0.10 current_y:=0.06
```

The dashboard displays the complete closed-loop path:

```text
target pose -> body-frame error -> PID wrench -> thruster allocation ->
Gazebo dynamics -> odometry/IMU -> error, safety and performance metrics
```

It also provides live updates for target pose, current, PID gains, thrust
limit, slew rate, tolerances and yaw/tilt enable flags. The reset button sends
an explicit integrator-reset command without restarting Gazebo. Export snapshot
stores the currently visible telemetry and history as a JSON file.

RViz receives these topics:

```text
/research_catamaran/station_keeping/visualization/markers
/research_catamaran/station_keeping/visualization/actual_path
/research_catamaran/station_keeping/visualization/target_path
```

The visualization node also broadcasts `world -> base_footprint`, allowing the
robot model and the research markers to move together in the RViz `world`
fixed frame.

The Gazebo joint-state bridge publishes the continuous propeller states on
`/research_catamaran/joint_states`; the research launch remaps that topic into
`robot_state_publisher`. This is required for RViz to resolve all four moving
propeller links instead of showing `No transform` for them.

The CSV evaluator now reports additional research-run quantities:

```text
mean_position_error_m
heading_max_error_rad
final_heading_error_rad
control_effort_Ns
saturation_fraction
max_horizontal_speed_mps
rms_horizontal_speed_mps
max_abs_yaw_rate_radps
max_abs_integral_state
```

## Motion commands

The reusable mixer listens on `/research_catamaran/cmd_vel`. The command is a
normalized planar command, not a measured velocity: `linear.x` is forward,
`linear.y` is left sway, and `angular.z` is a left turn. The mixer applies a
configurable slew limit and then zeros all four thrusters after its safety
timeout.

Run a complete one-maneuver demo with:

```bash
ros2 launch asv_station_keeping motion_demo.launch.py \
  maneuver:=forward duration:=3.0
```

The motion launch uses the dynamics world, starts near buoyant equilibrium,
and applies a normalized command slew limit before publishing thrust. This
prevents a step input from exciting the vehicle's pitch mode. For a more
aggressive experiment, increase `command` or `slew_rate` one at a time and
record the odometry/IMU response. The model also enforces a conservative
`+/-6` thrust-command cap so a manual topic command cannot bypass the slow
baseline.

Available maneuvers are:

```text
forward
backward
left
right
strafe_left
strafe_right
stop
```

Examples:

```bash
ros2 launch asv_station_keeping motion_demo.launch.py \
  maneuver:=backward duration:=3.0

ros2 launch asv_station_keeping motion_demo.launch.py \
  maneuver:=left duration:=2.0

ros2 launch asv_station_keeping motion_demo.launch.py \
  maneuver:=right duration:=2.0
```

For an automated direction-and-stability check, run the complete validation
launch:

```bash
ros2 launch asv_station_keeping motion_validation.launch.py
```

The validator applies forward, backward, left-turn and right-turn commands,
then prints `dx`, `dy`, `dyaw`, maximum roll, maximum pitch, surge z
excursion, and either `MOTION_VALIDATION_PASS` or
`MOTION_VALIDATION_FAIL`. The default test uses command `0.15`, three seconds
per maneuver, a `0.35 rad` attitude limit, and a `0.30 m` surge z-excursion
limit. The mixer also uses a low thrust scale and a tilt guard that fades
thrust out between `0.10 rad` and `0.35 rad` of roll or pitch. A stronger test
can be requested explicitly:

```bash
ros2 launch asv_station_keeping motion_validation.launch.py \
  command:=0.25 maneuver_time:=4.0
```

The underlying four-thruster mixer is:

```text
port_front       = surge + sway + yaw
starboard_front  = surge - sway - yaw
port_rear        = -surge + sway - yaw
starboard_rear   = -surge - sway + yaw
```

Therefore the intended command patterns are:

```text
forward       (+,+,-,-)
backward      (-,-,+,+)
left turn     (+,-,-,+)
right turn    (-,+,+,-)
left sway     (+,-,+,-)
right sway    (-,+,-,+)
```

If the model rotates opposite to the expected visual direction on a specific
Gazebo installation, invert only the `angular.z` command first; do not change
the forward/backward signs without checking the thruster axes.

## Realistic sea state and noisy localization

`dynamic_station_keeping.launch.py` and `visual_station_keeping.launch.py`
now run with a repeatable disturbance environment and with the controller
closed on noisy sensors instead of simulator ground truth.

```bash
ros2 launch asv_station_keeping dynamic_station_keeping.launch.py \
  wind_speed:=4.0 wind_direction_deg:=90.0 wind_gust_std:=0.4 \
  wave_height:=0.15 wave_period:=2.0 current_x:=0.1 seed:=7
```

| Argument | Default | Meaning |
|---|---|---|
| `current_x/y/z` | 0.10 / 0 / 0 | Steady water current (m/s, world frame) |
| `current_turbulence_std` | 0.02 | Gauss-Markov turbulence on the current (m/s) |
| `wind_speed` | 3.0 | Mean wind (m/s); `0` disables wind |
| `wind_direction_deg` | 135 | Direction wind blows **toward** (0 = +X, 90 = +Y) |
| `wind_gust_std` | 0.30 | Gust std as a fraction of mean wind speed |
| `wind_direction_wander_deg` | 12 | Gust-driven direction wander (std) |
| `wave_height`, `wave_period`, `wave_direction_deg` | 0.10, 2.5, 135 | Regular wave surface orbital velocity |
| `seed` | 1 | Random seed, so runs are repeatable |
| `use_estimator` | true | `false` closes the loop on ground truth |

All of these are combined by `research_current_disturbance` into a single
effective water velocity on `/ocean_current`, which Fortress's hydrodynamics
system turns into hull drag. Wind force `0.5*rho_air*Cd*A*|V|*V` is converted
to the water speed that gives the same hull drag at rest, so the model is
accurate for a stationary hull (the station-keeping case). Approximations:
wind yaw moments and wave-induced heave/pitch are not modelled, the wind
windage area (`0.15 m^2`) and `Cd` are estimates, and the water plane is
visually static. The components are published separately on
`/research_catamaran/environment/{wind,wave_velocity,water_current}`.

Sensors carry noise (BNO085-class IMU, 5 cm GNSS). Fortress applies navsat
horizontal noise in **degrees**, so the URDF uses `4.5e-7` for 5 cm.
`research_state_estimator` fuses GPS (10 Hz) and IMU (100 Hz) with a
position/velocity Kalman filter per axis and publishes
`/research_catamaran/estimated_odometry`; the controller feeds back on it
when `use_estimator:=true`. Performance columns in the CSV (`position_error`,
RMSE) are always scored against ground truth, with `true_x`, `true_y`,
`true_yaw` and `estimate_error` appended, so a bad estimate cannot hide a real
position error. `/research_catamaran/estimation_error` publishes the live
estimate-minus-truth error.

Reference results with the defaults (3 m/s wind, gusts, waves, turbulent
current, noisy sensors): about 6 cm RMS and 11 cm peak position error, and
about 2 cm RMS estimation error.

## Demonstration and evidence for review

**Live demo** (Gazebo + RViz + dashboard, with a visible green target marker
that follows the active target and a wind shift in phase 2):

```bash
ros2 launch asv_station_keeping visual_station_keeping.launch.py
```

**Uncontrolled baseline.** `hold_enabled:=false` runs the identical stack with
the thrusters off, so you can show the bare hull being carried away by the
same wind, waves and current:

```bash
ros2 launch asv_station_keeping dynamic_station_keeping.launch.py hold_enabled:=false
```

**Thruster realism.** The controller output passes through an ESC deadband
(`thruster_deadband`), first-order lag (`thruster_lag_tau`) and weaker reverse
thrust (`thruster_reverse_scale`). Disable with `actuator_model:=false`.

**Automated comparison suite.** Runs 10 headless conditions (three sea states,
controlled vs. uncontrolled, three seeds, estimator and thruster-model
ablations) and writes `report.html` with paths, error-vs-time and a metrics
table, scored on simulator ground truth:

```bash
python3 src/asv_station_keeping/tools/run_experiments.py --out ~/asv_results
python3 src/asv_station_keeping/tools/run_experiments.py --out ~/asv_results --report-only
```

## Showcase and advanced features

**One-command guide demo** (Gazebo + RViz + dashboard; an orange uncontrolled
twin boat drifts away while the blue boat holds; wind/current arrows with
speed readouts, a path trail and a green target marker are drawn in Gazebo;
a narrated fault timeline runs automatically):

```bash
ros2 launch asv_station_keeping showcase.launch.py
```

The narrated timeline (`scenario:=narrated`) injects, in order: a wind step,
GPS loss, a thruster failure (followed by thrust re-allocation), a GPS jump
and a wind-direction swing, and turns the disturbance observer on. Custom
timelines: `scenario:=custom scenario_events:='[[t, "/node", "param", value,
"text"]]'`.

| Feature | Where | Notes |
|---|---|---|
| Real wind/wave forces | `sea_state_gz_bridge` (C++) + `current_disturbance` | Wind force, wind **yaw moment**, wave heave/roll/pitch via Fortress `ApplyLinkWrench`. Wave moments are scaled by `wave_moment_scale` (0.08) to the simulator's buoyancy stiffness. |
| Side-by-side twin | `ghost:=true` | Passive copy, same sea. |
| Gazebo overlays | `sea_state_gz_bridge` + MarkerManager | Wind/current arrows and labels, controlled-boat path trail. |
| Fault injection | `scenario_runner`, estimator/controller params | `gps_enabled`, `gps_offset_x/y`, `imu_bias_x/y`, `thruster_gain_i` (hardware), `allocator_health_i` (what the controller knows). |
| Fault-tolerant allocation | `twist_to_thruster.allocate` | Least-squares re-solve over healthy thrusters; identical to the mixer when all are healthy. |
| Disturbance observer | `use_dob:=true` | Feeds the estimated wind/current force forward; estimate on `/research_catamaran/station_keeping/disturbance_estimate`. |
| Missions | `mission:=weathervane` / `waypoints` | Weathervane turns the bow into the wind using only the observer's estimate. |
| Lidar + map error | `research_map_quality` | Projects scans with the estimated and the true pose and measures distance to the true pool walls. `/research_catamaran/mapping/error` = (RMS est, RMS true, P95). |
| Energy index | CSV `power_index`, `energy_index` | Sum of |thrust|^1.5, relative units. |
| Anti-windup, thrust budget | controller | Integrators freeze while saturated; `yaw_reserve` keeps thrust available for heading. |
| Robust estimator | `state_estimator` | Acceleration clamp and GPS-innovation gating with automatic reset. |
| Parameter identification | `tools/identify_hydrodynamics.py` | Fits drag and thruster deadband/asymmetry from real decay and bollard-pull logs. |
| Reproducible environment | `docker/` | Not built in CI; report problems. |

## Honest limitations (read before presenting)

- **Strong seas are near the limit.** The thruster lever arms about the centre
  are about 5 cm, so heading authority is small. Around 4.5 m/s wind with
  gusts, holding degrades to tens of centimetres, and well beyond that it is
  lost. This is a finding about the current thruster layout, not a bug.
- **The disturbance observer does not help under steady/moderate wind.** The
  PID integrator already rejects steady wind; the observer's feed-forward adds
  noise. It is included as a comparison, not as an improvement.
- **Added mass and Coriolis are off.** Enabling plausible values degraded
  holding with the current gains; they need retuning and real data.
- Not done: upgrade to Gazebo Harmonic / ROS 2 Jazzy, MPC/sliding-mode
  controllers, a SLAM package (a map-consistency metric is provided instead),
  battery voltage-sag model, and any fit to real-vehicle data.
- All hydrodynamic, windage and thrust parameters are estimates.

## Fixed in this revision

- The hydrodynamics plugin targeted `base_link`, which Gazebo lumps into
  `base_footprint`; it logged `no linear vel` constantly and applied neither
  damping nor current. It now targets `base_footprint`.
- The sway sign in the thruster mixer was inverted, which made the sway loop
  unstable (position error grew exponentially). The mixer and the RViz
  thruster arrows now match the measured thruster axes.
- Controller gains were raised because the previous values left most of the
  thrust range unused.

## What this proves

This baseline proves the following narrow path first:

```text
package installation -> world creation -> model spawn -> visible hulls,
deck, supports and four thruster locations -> ROS clock/state/sensors ->
four thrust command topics -> baseline controller -> CSV evaluation
```

It does not yet claim:

- CAD-accurate placement of every SolidWorks occurrence.
- Steering control.
- Validated wave motion or current disturbance.
- Validated buoyancy coefficients.
- Measured thruster coefficients.

The pool display world uses a visible plane with a collision surface. It is a
display environment, not the motion-validation world. The four thruster
systems are active, but their coefficient is a software-test baseline rather
than measured T200 data. The dynamics world enables buoyancy and
simulation-tuned damping. The current publisher and hydrodynamics
configuration are repeatable disturbance-test interfaces; they do not yet
claim validated drag, waves, current coefficients or physical thruster
coefficients.

## ROS interfaces

### State and sensors

```text
/clock
/model/research_catamaran/odometry       nav_msgs/msg/Odometry
/research_catamaran/joint_states         sensor_msgs/msg/JointState
/research_catamaran/imu                   sensor_msgs/msg/Imu
/research_catamaran/gps                   sensor_msgs/msg/NavSatFix
/research_catamaran/station_keeping/status diagnostic_msgs/msg/DiagnosticArray
/research_catamaran/station_keeping/reset_integrators std_msgs/msg/Empty
/research_catamaran/station_keeping/start_demo std_msgs/msg/Empty
/ocean_current                             geometry_msgs/msg/Vector3
```

Visualization outputs:

```text
/research_catamaran/station_keeping/visualization/markers
/research_catamaran/station_keeping/visualization/actual_path
/research_catamaran/station_keeping/visualization/target_path
```

### Thruster commands

Each command is a `std_msgs/msg/Float64` value bridged to the Fortress
`ignition.msgs.Double` thruster topic:

```text
/model/research_catamaran/joint/port_front_propeller_joint/cmd_thrust
/model/research_catamaran/joint/starboard_front_propeller_joint/cmd_thrust
/model/research_catamaran/joint/port_rear_propeller_joint/cmd_thrust
/model/research_catamaran/joint/starboard_rear_propeller_joint/cmd_thrust
```

The baseline controller uses the diagonal-thruster allocation matrix implied
by the four old mounting directions. Position control is the default verified
path. Yaw control is separately switchable because the Fortress thruster
plugin's force sign and moment convention must be calibrated for a reliable
heading result; it is not silently presented as validated.

## Geometry source and dimensions

The hull and thruster dimensions are carried over from the original visible
URDF:

- Visual hull mesh and original placement are retained from the old model.
- Dynamics hull volume: `0.80 x 0.16 x 0.24 m` per hull, simulation-only.
- Hull centre positions: `y = +0.175 m` and `y = -0.175 m`.
- Thruster positions relative to each hull: `x = +0.25 m` or `x = -0.25 m`,
  `z = -0.05 m` in the research stability model.
- Mesh scale: `0.001 0.001 0.001`, because the STL exports are millimetres.

The deck and sensor structures are intentionally primitive and clearly named.
They can be replaced link-by-link after a native SolidWorks assembly export is
available. The `simulation_ballast` link is intentionally hidden and exists
only to stabilize roll/pitch in the simulation model.

## Reference patterns checked

The structure follows established ROS/Gazebo conventions rather than copying
another vehicle's geometry:

- `ros_gz_sim` is used to start Gazebo and insert the URDF.
- `robot_state_publisher` publishes the fixed and continuous joint tree.
- The Gazebo Fortress world uses the `ignition-gazebo-*` system plugin naming
  used by the Fortress-era examples.
- The four propeller joints are separated from the fixed thruster housings so
  later commands can rotate only the propellers.

The open-source VRX/WAM-V family is useful as a reference for USV simulation
architecture, but its hull dimensions, plugins and hydrodynamics are not the
same as this ASV. Its files should not be substituted blindly for this model.
