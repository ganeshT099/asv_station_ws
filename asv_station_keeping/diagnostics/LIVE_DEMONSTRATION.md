# Live demonstration and experiment protocol

This protocol is intended for a repeatable research presentation of the
simulation-only dynamic station-keeping system.

## 1. Build and source

On the ROS 2 Humble computer:

```bash
cd /home/gt0908/asv_station_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select asv_station_keeping --symlink-install
source install/setup.bash
```

The NVIDIA renderer should be used for the interactive demonstration. Keep
`software_rendering:=false` unless the desktop driver is unavailable.

## 2. Start the complete console

```bash
ros2 launch asv_station_keeping visual_station_keeping.launch.py
```

The expected windows are:

1. Gazebo Fortress: the simulated catamaran and pool.
2. RViz2: target station, tolerance, paths, current, velocity, heading and
   four signed thruster vectors.
3. A browser at `http://127.0.0.1:8765`: telemetry, plots, parameters and
   safety state.

The full visual launch is intentionally a **station-keeping-only** launch. It
does not start `motion_demo` or `twist_to_thruster`. With the default
`demo_mode:=true`, it runs a repeatable two-phase presentation scenario:

1. Move from the spawn point to approximately `(0.40, 0.20, 0.20 rad)` while
   rejecting the configured current.
2. After 12 seconds, change to approximately `(-0.35, -0.22, -0.20 rad)` and
   change the current direction. The controller then holds the new station.

The browser's **Run scripted demo** button restarts this two-phase sequence.
To use only a manually selected setpoint, disable the scenario:

```bash
ros2 launch asv_station_keeping visual_station_keeping.launch.py \
  demo_mode:=false \
  target_x:=0.50 target_y:=0.25 target_yaw:=0.20 \
  current_x:=0.10 current_y:=0.06
```

Do not leave an old `motion_demo.launch.py` process running at the same time;
it publishes independent thruster commands and would compete with the
station-keeping controller. Stop old ROS launch terminals with `Ctrl+C` before
starting this demonstration.

The live baseline uses a 500 Hz physics step, filtered velocity feedback,
bounded integral leakage, thrust slew limiting and target-setpoint slew
limiting. This prevents a dashboard target change from becoming an
instantaneous force step while keeping the higher-fidelity research world
available separately.

The dashboard binds on all local interfaces and the ROS terminal prints the
actual URL. If port `8765` is occupied, it automatically selects a free port;
open the printed URL exactly.

The default experiment holds the origin against a `+X` current of `0.10 m/s`.
The default CSV is:

```text
/tmp/research_dynamic_station_keeping.csv
```

## 3. Demonstration sequence

Use the following sequence when presenting the system:

1. Show Gazebo's 3D model, water surface and four thruster locations.
2. Show the green target marker and the actual path in RViz.
3. Point out the current vector and the error vector.
4. Show the controller status changing from `WARN` while correcting to `OK`
   when the target tolerances are met.
5. Point out the four signed thrust values and their corresponding RViz arrows.
6. Change `current_y` in the dashboard and apply it. The disturbance node and
   controller display update without restarting Gazebo.
7. Change `target_x`, `target_y` or `target_yaw` and observe the new response.
8. Use **Reset integrators** when beginning a fresh trial from the same world.
9. Export a JSON snapshot for the presentation record.

## 4. Repeatable research runs

For a cross-current test:

```bash
ros2 launch asv_station_keeping visual_station_keeping.launch.py \
  target_x:=0.50 target_y:=0.25 target_yaw:=0.20 \
  current_x:=0.10 current_y:=0.06 \
  log_file:=/tmp/cross_current_station_keeping.csv
```

After stopping the run with `Ctrl+C`:

```bash
ros2 run asv_station_keeping evaluate_research_run \
  /tmp/cross_current_station_keeping.csv
```

For paper-ready artifacts:

```bash
ros2 run asv_station_keeping evaluate_research_run \
  /tmp/cross_current_station_keeping.csv \
  --json-output /tmp/cross_current_station_keeping.json \
  --html-output /tmp/cross_current_station_keeping.html
```

The HTML file is self-contained and can be archived with the CSV and launch
arguments as an experiment record.

Record the printed metrics together with the exact launch arguments and the
controller YAML file. The model and thruster coefficients are simulation
baselines and must be described as such in a paper; they are not measured
vehicle coefficients.

## 5. Acceptance observations

For each trial, record:

- Position RMSE and maximum position error.
- Final position and heading error.
- Settling time.
- Heading RMSE and maximum heading error.
- Maximum roll, pitch and vertical excursion.
- Maximum thruster command and saturation fraction.
- Control effort.
- Maximum horizontal speed and yaw rate.
- Gazebo real-time factor.
- Whether any telemetry source became stale.

The experiment is not considered valid if odometry or diagnostic data are
stale, Gazebo is running materially below real time, or a safety limit is
silently exceeded.

## 6. Runtime troubleshooting

If the robot is visible but the trajectory is not, verify that the
visualization node is running and that the path topic contains poses:

```bash
ros2 node list | grep station_keeping
ros2 topic hz \
  /research_catamaran/station_keeping/visualization/actual_path
ros2 topic echo \
  /research_catamaran/station_keeping/visualization/actual_path --once
```

The path is published in the `world` frame and is also duplicated as a blue
MarkerArray line slightly above the vehicle so it remains visible over the
water surface. If the topic is absent, rebuild and source the workspace again;
the installed launch must contain the visualization executable.

For the browser console, use the exact URL printed by:

```bash
ros2 launch asv_station_keeping visual_station_keeping.launch.py
```

Then verify the HTTP endpoint independently:

```bash
curl http://127.0.0.1:8765/api/health
```

If the launch reports a different port, replace `8765` with that port. A
previous dashboard process or an old launch can keep the requested port busy;
the current node automatically selects a free port and prints it.

Check the renderer before a live presentation:

```bash
nvidia-smi
glxinfo -B | grep -E "OpenGL vendor|OpenGL renderer"
```

The renderer should be NVIDIA rather than `llvmpipe`. If Gazebo is slow while
the renderer is hardware accelerated, the remaining bottleneck is likely the
`0.002 s` physics step, hydrodynamics, shadows or the high-detail thruster
visual meshes. Preserve the research world and create a separate fast
interactive world rather than changing the experiment silently.
