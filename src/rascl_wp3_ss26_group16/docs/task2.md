# Task 2 — Online motion planning

Task 2 accepts cube centres as Cartesian `geometry_msgs/msg/Point` messages on
`/goal_poses`. The fields are `x`, `y`, and `z` in metres in the calibrated task
frame. Accepted messages are republished unchanged on `/goal_pose`, converted
internally to cylindrical coordinates, queued, and processed sequentially.

The configured Cartesian input bounds are:

- `-0.40 <= x <= 0.40 m`
- `0.03 <= y <= 0.32 m`

All three coordinates must be finite. The `z` coordinate has no separate input
bound, but every generated TCP waypoint must have a valid IK solution within
the arm joint limits. A pose can therefore pass the rectangular input check and
still be rejected during planning.

Example input:

```bash
ros2 topic pub --once /goal_poses geometry_msgs/msg/Point \
  "{x: 0.04, y: 0.18, z: 0.03}"
```

There is no `/cube_pose_cylindrical` subscription in the current node.

## Motion sequence

For each queued cube, the node generates one continuous joint trajectory whose
waypoint transitions are minimum-jerk segments:

1. configured home pose
2. approach the cube with the gripper open
3. lower to the grasp TCP pose
4. close the gripper
5. lift back to the approach pose
6. fold the upper-arm and lower-arm joints to their home positions while
   retaining the cube shoulder angle
7. rotate the folded arm to the shoulder angle of the pre-place waypoint
8. extend to the pre-place TCP pose with the gripper closed
9. move to the placement TCP pose
10. open the gripper
11. retreat to the pre-place TCP pose with the gripper open
12. return home

The gripper remains closed while the arm folds, rotates, and moves to the
placement point. Each waypoint transition takes `4.0 s` by default and is
sampled every `0.02 s`.


## Inverse kinematics

Task 2 uses Robotics Toolbox as its only IK solver. It:

- loads the project robot model from `rascl_description/urdf/rascl.urdf`;
- solves position-only Levenberg–Marquardt IK;
- models the TCP as `[0.0, 0.038, 0.0] m` from the URDF end-effector frame;
- applies a `-pi/2` task-to-URDF angular offset;
- reuses the previous solution as the next seed and tries fallback seeds; and
- accepts a TCP position error up to `0.1 mm`.

The first three resulting joints must remain within the URDF limits of
`[-pi/2, pi/2]`. Robotics Toolbox and SpatialMath are installed in
`/opt/rascl_venv` by the project Dockerfile; the node adds that environment's
site-packages when necessary.

## RViz simulation

Build and source the workspace, then run:

```bash
ros2 launch rascl_wp3_ss26_group16 wp3_tsk2_sim.launch.py
```

In another sourced terminal, publish a cube centre:

```bash
ros2 topic pub --once /goal_poses geometry_msgs/msg/Point \
  "{x: 0.04, y: 0.18, z: 0.03}"
```

The simulation publishes `/joint_states` for `robot_state_publisher` and
replays the generated trajectory in real time. The node remains active for
additional cube messages.

## Real robot

Start ROS 2 control:

```bash
ldconfig
ros2 launch rascl_description ros2_control.launch.py
```

In another sourced terminal, confirm that the trajectory controller is active
and then start Task 2:

```bash
ros2 control list_controllers
ros2 launch rascl_wp3_ss26_group16 wp3_tsk2.launch.py
```

Publish one Cartesian cube centre at a time from a third sourced terminal:

```bash
ros2 topic pub --once /goal_poses geometry_msgs/msg/Point \
  "{x: 0.04, y: 0.18, z: 0.03}"
```

On real hardware, planning waits until a complete `/joint_states` message has
provided all four configured joints. The generated trajectory is sent to
`/joint_trajectory_controller/follow_joint_trajectory`.

All topics, limits, geometry, IK settings, TCP targets, gripper positions, and
timing values described above are parameters in `config/task2.yaml`.
