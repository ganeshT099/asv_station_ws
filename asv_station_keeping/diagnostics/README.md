# Incremental black-screen diagnostic

The normal `asv_station_keeping` launch uses the original two-hull / four-
thruster model. The diagnostic launch now defaults to `00_original.urdf`, a
byte-for-byte copy of that visible model.

**`00_base.urdf` contains no `<visual>` geometry. An invisible robot in this
test is expected; it is not evidence of a rendering failure.** The surrounding
world should still be visible. An entirely black world is a different symptom.

The existing variants `01` through `32` are a separate cumulative sequence
using the earlier manually estimated assembly positions. Controllers and
sensors are omitted, while collisions and inertias remain. These tests check
whether parts can load; they do not verify CAD mounting accuracy. Even the
last variant uses estimated placements.

If all visible variants render, as reported, there is no need to repeat the
sequence. Obtain the assembled CAD data described in `ASSEMBLY_STATUS.md` to
correct mounting.

Generate or regenerate the variants from the workspace source package:

```bash
python3 src/asv_station_keeping/tools/generate_diagnostic_variants.py
```

Build the package:

```bash
colcon build --symlink-install --packages-select asv_station_keeping
source install/setup.bash
```

Run one variant at a time. Stop Gazebo with `Ctrl+C` before starting the next:

```bash
ros2 launch asv_station_keeping diagnostic_variant.launch.py \
  variant:=00_original.urdf

ros2 launch asv_station_keeping diagnostic_variant.launch.py \
  variant:=01_hull_left.urdf

ros2 launch asv_station_keeping diagnostic_variant.launch.py \
  variant:=02_hull_right.urdf
```

For reference, the original sequence is listed in
`diagnostics/variants/README.txt`. A rendering change between two variants
identifies a change to investigate; it does not by itself prove a defective
mesh. Compare logs, model pose and camera position as well.
