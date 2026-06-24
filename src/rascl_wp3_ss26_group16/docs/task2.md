# Task 2 — Online motion planning

The node accepts either:

- Cartesian cube centers on `/goal_poses` as required by the tasksheet.
- Cylindrical cube centers `(r, theta, z)` on `/cube_pose_cylindrical`, encoded in
  the `x`, `y`, and `z` fields of `geometry_msgs/msg/Point`.

Every accepted input is represented internally as `(r, theta, z)` and published
as Cartesian `(x, y, z)` on `/goal_pose`. The node validates the radius and
shoulder angle, calculates analytical IK for the three arm joints, then generates
online minimum-jerk segments for this sequence:

1. home/open
2. approach cube
3. lower to cube
4. close gripper
5. lift
6. return upper-arm and lower-arm joints to their initial positions
7. rotate only the shoulder joint to the fixed goal angle
8. extend above the known target
9. lower to target
10. open gripper
11. retreat
12. home

During steps 6 and 7, the gripper remains closed. While folding, the shoulder
stays at the cube angle. Shoulder rotation starts only after the upper-arm and
lower-arm joints have reached their configured home positions.

The fourth joint uses the same open/closed values as Task 1. Robot dimensions,
joint signs/offsets, target position, feasible radii, approach height, and gripper
values are calibration parameters in `config/task2.yaml`.

The cube may arrive anywhere inside the feasible radial and angular region. The
goal is fixed and known: it uses the configured target radius and right-side
angle:

```text
cube:   (cube_radius, cube_theta, cube_z)
target: (target_radius, target_theta, target_z)
```

The current configuration uses `target_radius=0.19 m` and
`target_theta=-1.5 rad`. Because the URDF shoulder axis has the opposite sign,
this produces a shoulder command of `+1.5 rad`, matching the right-side goal
used by Task 1 while retaining margin from the `+π/2` joint limit.

## Confirmed dimensions

The supplied `BoxDimensions.pdf`, `urdfCreation.pdf`, and `rascl.urdf` give:

- cube footprint: 40 mm × 40 mm
- cube height: 41 mm
- cube center above the plate: 20.5 mm
- grasping end-effector midpoint:
  15 mm vertically above the center of the cube's top surface
- shoulder pitch-axis height: 57.441 mm + 65.560 mm = 123.001 mm
- shoulder-to-elbow planar distance:
  `sqrt(170² + 80²)` mm = 187.8829423 mm
- elbow-to-end-effector-axis planar distance: 129.09 mm

The URDF also contains offsets of 11.6 mm, 5.7 mm, 21.83 mm, and 17.9 mm.
These place the mesh coordinate systems and joint axes in 3D; they are not
additional planar link lengths.

The URDF mechanical-zero geometry is also included in the joint conversion:
the upper link starts at `1.1177305070 rad` and the relative elbow angle starts
at `-1.1169401780 rad`. Therefore, the raw two-link IK angles are converted to
motor angles before applying the ±π/2 joint-limit checks.

The configured `radial_tool_offset` and `vertical_tool_offset` are zero for the
tested setup. The configured `target_z=0.0205` assumes that the box-plate
surface is `z=0` in `base_link`.
For a cube-center input `cube_z`, the IK grasp height is:

```text
end_effector_z = cube_z + cube_height / 2 + 0.015
```

Thus a cube centered 20.5 mm above the plate gives an end-effector midpoint
height of 56 mm. The analytical IK controls this point only; it does not impose
an orientation or require the end effector to be parallel to the cube top.

Example cylindrical input:

```bash
ros2 topic pub --once /cube_pose_cylindrical geometry_msgs/msg/Point \
  "{x: 0.15, y: 0.75, z: 0.03}"
```

Example tasksheet-compatible Cartesian input:

```bash
ros2 topic pub --once /goal_poses geometry_msgs/msg/Point \
  "{x: 0.15, y: 0.0, z: 0.03}"
```

For cylindrical input, `Point.x=r`, `Point.y=theta`, and `Point.z` is the cube
centre height. For Cartesian input, the radius is calculated as
`sqrt(x² + y²)`.

## RViz simulation

```bash
ros2 launch rascl_wp3_ss26_group16 wp3_tsk2_sim.launch.py
```

Then publish one valid cube center, for example:

```bash
ros2 topic pub --once /cube_pose_cylindrical geometry_msgs/msg/Point \
  "{x: 0.15, y: 0.75, z: 0.03}"
```

The simulation publishes `/joint_states` for `robot_state_publisher` and replays
each minimum-jerk segment in real time.

## Real robot

Start ROS 2 control:

```bash
ldconfig
ros2 launch rascl_description ros2_control.launch.py
```

In another sourced terminal, confirm that `joint_trajectory_controller` is
active, then launch Task 2:

```bash
ros2 control list_controllers
ros2 launch rascl_wp3_ss26_group16 wp3_tsk2.launch.py
```

Publish one cube centre at a time from a third sourced terminal. Valid messages
are queued and executed sequentially. The current minimum-jerk segment duration
is `4.0 s`.

```bash
ros2 topic pub --once /cube_pose_cylindrical geometry_msgs/msg/Point \
  "{x: 0.13, y: 0.0, z: 0.03}"

ros2 topic pub --once /cube_pose_cylindrical geometry_msgs/msg/Point \
  "{x: 0.15, y: 0.78, z: 0.03}"

ros2 topic pub --once /cube_pose_cylindrical geometry_msgs/msg/Point \
  "{x: 0.21, y: -0.4, z: 0.03}"
```
