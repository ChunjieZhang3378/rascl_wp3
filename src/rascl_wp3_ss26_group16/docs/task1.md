# Task 1 — Offline motion planning

Task 1 picks three cubes and builds a stable tower in the order cube 1, cube 2,
cube 3.

## Placement used

- Cube 1: `r=0.23 m`, `theta=-0.75 rad`
- Cubes 2 and 3: `r=0.17 m`, `theta=0 rad`
- Cubes 2 and 3 initially form a stack, with cube 2 below cube 3
- Goal: right side of the robot

All radii are measured from the robot base centre/shoulder rotation axis to the
cube centre.

## Implementation

The node:

1. reads the four ordered joint-space waypoint files;
2. generates fifth-order minimum-jerk samples for every segment;
3. saves positions, velocities, and accelerations for inspection; and
4. executes the trajectories using the joint trajectory controller.

Execution order:

1. `cube_1_to_goal.csv`
2. `cube_2_to_bottom.csv`
3. `cube_3_to_goal.csv`
4. `cube_2_to_goal.csv`

Each input CSV contains:

```text
duration,shoulder_joint,upperarm_joint,lowerarm_joint,end_effector_joint
```

The first row is the initial configuration with duration zero. Every following
duration is the time assigned to its minimum-jerk segment.

## Simulation

```bash
ros2 launch rascl_wp3_ss26_group16 wp3_tsk1_sim.launch.py
```

The simulation publishes the generated trajectory as `/joint_states` for RViz.

## Real robot

Start ROS 2 control first:

```bash
ros2 launch rascl_description ros2_control.launch.py
```

Then launch Task 1 from another sourced terminal:

```bash
ros2 launch rascl_wp3_ss26_group16 wp3_tsk1.launch.py
```

Task 1 uses CSP mode, a `0.02 s` sample period, and the
`/joint_trajectory_controller/follow_joint_trajectory` action.
