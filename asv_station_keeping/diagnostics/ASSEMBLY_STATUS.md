# Mounting accuracy: source assembly data required

## What the current evidence shows

- The user reports that the original vehicle and the visible incremental
  variants render in Gazebo.
- `00_base.urdf` has two links with inertial/frame information only and zero
  visuals. It is expected to be invisible.
- The earlier full assembly URDF uses manually estimated mounting positions.
  Describing these positions as recovered from CAD was incorrect.
- Partial variants omit parts; the final variant also has incorrect placement
  because its transforms are estimates. Adding fixed joints connects the
  kinematic tree but does not place the parts at their actual mounting points.

## What the original archive contains

Inspection of `ASV_Dynamic_keeping_SIM.zip` found:

- 59 link declarations, 26 unique link names, and zero joints.
- All visual origins have zero translation and rotation.
- Four thruster declarations reference the same `thruster.stl` without
  distinct assembly poses. The hull instances have the same problem.
- No STEP, F3D/F3Z, or occurrence-transform JSON file is included.

The STL files retain part shapes, sometimes with offsets inside their own
mesh coordinates. They do not record the missing relative placements of
repeated occurrences. Inertial origins describe centres of mass, not parent-
child mounting transforms. Mesh centering and dimensions cannot uniquely
recover the full CAD assembly.

## Export needed from Fusion 360

Preferred input: a **STEP export of the entire assembled design**.

1. Open the complete design in Fusion 360 and return to the assembled pose.
2. Activate the root/top-level component so the complete assembly is exported.
3. Use **File > Export**, choose STEP (`.step` or `.stp`), and export the full
   design rather than saving individual components as separate STL files.
4. Put the STEP file in Downloads or the project and provide its path.
5. Include an isometric screenshot (and ideally a top view) to verify the
   intended orientation and mounts against the imported assembly.

A native complete Fusion archive (`.f3d`, or `.f3z` for a design with external
references) is also useful. STEP is preferred here because it can be read
without running Fusion. If using an occurrence-transform export instead,
include a unique full occurrence path, parent occurrence path, local-to-root
4x4 transform and units for every instance, plus matching local mesh data.

## How the assembly will be reconstructed

Use the CAD assembly transforms rather than manual spacing. For a parent and
child expressed in one assembly frame, compute:

```text
T_parent_child = inverse(T_assembly_parent) * T_assembly_child
```

Joint origins must account for any mesh-local offsets and the choice of ROS
body frame (X forward, Y left, Z up). Correctly transformed inertial properties
are required as well. Once the fixed assembly matches the CAD, add only the
degrees of freedom actually present in the design. A fixed thruster housing
must not be spun as if it were its propeller, and steering axes/limits require
the real mount geometry.

Until this source data is supplied, exact CAD mounting is unresolved.
