# RASCL WP3 SS26 — Group 16

ROS 2 package for both Work Package 3 tasks.

## Nodes

- `wp3_tsk1`: offline minimum-jerk trajectory execution
- `wp3_tsk2`: online planning using cube positions from `/goal_poses`

## Launch files

- `wp3_tsk1.launch.py`
- `wp3_tsk2.launch.py`

Task 1 reads the ordered waypoint CSV segments in `trajectories/input`,
generates minimum-jerk trajectories, and sends them to
`joint_trajectory_controller`.
