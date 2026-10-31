# ASV Dynamic Keeping Gazebo package

This package is the repaired version of the Fusion 360 / ACDC export in the
archive.  The export had duplicate link names and no joints, so Gazebo could
not construct an articulated model.  `urdf/asv_assem_sim.urdf` now has a
single connected tree, four independently controllable steering joints, and
four continuous propeller joints:

```text
left_front_steering_joint
left_rear_steering_joint
right_front_steering_joint
right_rear_steering_joint
left_front_propeller_joint
left_rear_propeller_joint
right_front_propeller_joint
right_rear_propeller_joint
```

The T200 visual mesh is reused for each instance.  CAD triangle meshes are
visual-only; primitive hull, equipment, clamp and thruster collisions are used
to make contacts stable and fast.  The very small repeated IMU component
meshes from the export are represented by the PCB mesh and one IMU link to
avoid duplicate link names and duplicate mass.

## Build and launch (ROS 2 Humble + Gazebo Fortress)

Copy the `asv_assem_sim` directory into the `src` directory of a ROS 2
workspace, then run:

```bash
cd ~/asv_station_ws
colcon build --symlink-install --packages-select asv_assem_sim
source install/setup.bash
ros2 launch asv_assem_sim gazebo_asv.launch.py
```

The launch file starts `asv_test.sdf`, publishes the URDF TF tree, waits for
Gazebo to become ready, and spawns the model as `asv`.  The model is not a
static visual: all four steering joints can move and the native Gazebo
maritime thruster systems apply force to the model.

The launch file automatically selects the XCB Qt backend and the Ogre 1
rendering engine. Ogre 1 is more tolerant of older or hybrid GPU drivers
than the Fortress default Ogre 2 renderer. These settings avoid the black 3D
viewport seen with some Ubuntu GNOME Wayland and Gazebo Fortress combinations.
If the viewport is still black because of the graphics driver, use Mesa
software rendering:

```bash
ros2 launch asv_assem_sim gazebo_asv.launch.py software_rendering:=true
```

## Move a thruster in Gazebo

The URDF adds Fortress joint-position controllers.  In another terminal,
publish a position in radians (the range is -pi/2 to +pi/2):

```bash
ign topic -t /model/asv/joint/left_front_steering_joint/cmd_pos \
  -m ignition.msgs.Double -p 'data: 0.5'
```

Use the corresponding joint name for the other three thrusters.  On newer
Gazebo releases the equivalent command is:

```bash
gz topic -t /model/asv/joint/left_front_steering_joint/cmd_pos \
  -m gz.msgs.Double -p 'data: 0.5'
```

To apply thrust, use Newtons.  For example, command all four thrusters
forward:

```bash
for joint in left_front left_rear right_front right_rear; do
  ign topic -t /model/asv/joint/${joint}_propeller_joint/cmd_thrust \
    -m ignition.msgs.Double -p 'data: 20'
done
```

On newer Gazebo releases replace `ign topic` / `ignition.msgs.Double` with
`gz topic` / `gz.msgs.Double`.

The launch file also starts `ros_gz_bridge`, so the same commands can be sent
from ROS 2 with `std_msgs/msg/Float64`:

```bash
ros2 topic pub --once \
  /model/asv/joint/left_front_steering_joint/cmd_pos \
  std_msgs/msg/Float64 "data: 0.5"

ros2 topic pub --once \
  /model/asv/joint/left_front_propeller_joint/cmd_thrust \
  std_msgs/msg/Float64 "data: 20.0"
```

Joint states are published on `/asv/joint_states`.  The model also contains
native Gazebo IMU, GPS and camera sensors on `/asv/imu`, `/asv/gps` and
`/asv/camera/image`; the launch bridge republishes those topics to ROS 2.

The included contact plane is a standalone model/spawn test.  It is not a
full water buoyancy model; the thruster commands are ready for motion tests,
and a buoyancy/hydrodynamics system can be added later without changing the
joint tree.
