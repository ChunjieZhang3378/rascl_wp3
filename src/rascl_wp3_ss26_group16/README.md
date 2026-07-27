# RASCL WP3 SS26 — Group 16

ROS 2 package implementing both Work Package 3 pick-and-place tasks.

## Contents

- `wp3_tsk1`: offline minimum-jerk trajectories for stacking three cubes
- `wp3_tsk2`: online IK and minimum-jerk planning for an arbitrary cube pose
- `config/`: controller, geometry, workspace, and timing parameters
- `trajectories/input/`: Task 1 joint-space waypoint CSV files
- `trajectories/output/`: generated Task 1 trajectory samples
- `docs/`: task workflow and cube-placement documentation

## Build

Run inside the ROS 2 workspace container:

```bash
ldconfig
rosbuild
rossetup
```

## Simulation

Allow container GUI windows on the host:

```bash
xhost +local:root
```

Launch either RViz preview:

```bash
ros2 launch rascl_wp3_ss26_group16 wp3_tsk1_sim.launch.py
ros2 launch rascl_wp3_ss26_group16 wp3_tsk2_sim.launch.py
```

Task 2 remains active and waits for Cartesian cube-centre positions:

```bash
ros2 topic pub --once /goal_poses geometry_msgs/msg/Point \
  "{x: 0.04, y: 0.18, z: 0.03}"
```

The fields are Cartesian `x`, `y`, and `z` values in metres. The current input
bounds are `x=-0.40–0.40 m` and `y=0.03–0.32 m`; IK and joint-limit checks may
reject points inside that rectangle if the complete motion is unreachable.

## Real robot

Start ROS 2 control in terminal 1:

```bash
ldconfig
ros2 launch rascl_description ros2_control.launch.py
```

In another sourced container terminal:

```bash
docker exec -it rascl-wp3-gruppe16 bash
rossetup
ros2 control list_controllers
```

Confirm that `joint_trajectory_controller` is active before launching a task.

Task 1:

```bash
ros2 launch rascl_wp3_ss26_group16 wp3_tsk1.launch.py
```

Task 2:

```bash
ros2 launch rascl_wp3_ss26_group16 wp3_tsk2.launch.py
```

Publish each Task 2 cube position from a third sourced terminal. The node queues
valid inputs and processes cubes sequentially.

