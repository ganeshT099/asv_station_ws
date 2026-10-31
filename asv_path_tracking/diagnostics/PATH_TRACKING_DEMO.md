# Standalone geometric path-tracking demonstration

This package is intentionally separate from `asv_station_keeping`. It reuses
the existing research catamaran model and dynamics parameters, but starts its
own path generator, path controller, Gazebo marker bridge, RViz configuration
and browser dashboard. Do not launch the station-keeping PID or the manual
motion demo at the same time.

## Build

The package containing the Python nodes is `asv_path_tracking`. The optional
Fortress Gazebo marker transport bridge is `asv_path_tracking_gazebo`.

```bash
cd /home/gt0908/asv_station_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select asv_station_keeping asv_path_tracking \
  asv_path_tracking_gazebo \
  --symlink-install
source install/setup.bash
```

## Launch

```bash
ros2 launch asv_path_tracking path_tracking_demo.launch.py
```

The selected default experiment is a circle at `0.10 m/s` with a `+X`
current of `0.10 m/s`. Gazebo displays the yellow reference and cyan live
actual trace through the Fortress MarkerManager bridge. RViz displays the same
reference and actual paths together with the robot, TF, current, velocity,
cross-track error, reference point, heading and thruster vectors.

Select a different shape without changing the existing station-keeping
package:

```bash
ros2 launch asv_path_tracking path_tracking_demo.launch.py \
  path_type:=figure_eight speed:=0.08 path_width:=1.40 path_height:=0.80
```

Available shapes:

```text
circle
ellipse
figure_eight
rounded_rectangle
triangle
```

The rectangle and triangle use continuous corner smoothing so the tangent
heading and reference velocity do not jump at a sharp geometric corner.

## Topics

Reference data:

```text
/asv_path_tracking/reference_path       nav_msgs/msg/Path
/asv_path_tracking/reference_pose       geometry_msgs/msg/PoseStamped
/asv_path_tracking/reference_twist      geometry_msgs/msg/TwistStamped
/asv_path_tracking/reference_markers    visualization_msgs/msg/MarkerArray
/asv_path_tracking/path_status           diagnostic_msgs/msg/DiagnosticArray
```

Actual and controller data:

```text
/asv_path_tracking/actual_path           nav_msgs/msg/Path
/asv_path_tracking/markers               visualization_msgs/msg/MarkerArray
/asv_path_tracking/status                diagnostic_msgs/msg/DiagnosticArray
/asv_path_tracking/run                   std_msgs/msg/Bool
/asv_path_tracking/reset                 std_msgs/msg/Empty
/asv_path_tracking_gazebo_marker_bridge  Gazebo Fortress marker transport
```

The controller publishes directly to the four existing Fortress thrust
topics. It does not publish to the old station-keeping command path.

## Dashboard

The launch prints the exact URL, normally:

```text
http://127.0.0.1:8780
```

The dashboard can start, pause, reset, change shape, change dimensions and
change speed. It also shows the path status, vehicle state, errors, current and
controller diagnostics.

## Acceptance checks

For each shape, verify:

1. The complete reference path is visible in Gazebo and RViz.
2. The cyan actual path grows while the vehicle moves.
3. The vehicle remains inside the pool walls.
4. The reference pose moves continuously along the path.
5. Cross-track and heading errors remain finite.
6. Thruster commands remain bounded and slew-limited.
7. Roll and pitch stay below the configured safety limits.
8. `/asv_path_tracking/status` remains fresh.
9. The dashboard and RViz use the same path and actual-pose data.
10. The CSV `/tmp/asv_path_tracking.csv` contains the complete run.

Useful checks:

```bash
ros2 node list | grep asv_path
ros2 topic hz /asv_path_tracking/reference_pose
ros2 topic hz /asv_path_tracking/actual_path
ros2 topic echo /asv_path_tracking/status --once
curl http://127.0.0.1:8780/api/health
```

If Gazebo shows the vehicle but not the paths, inspect the marker bridge
terminal and confirm that the world loaded `MarkerManager`. RViz path messages
remain the authoritative numerical trajectory record even when a renderer is
unavailable.
