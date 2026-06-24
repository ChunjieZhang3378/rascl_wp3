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

For a standard ROS 2 build, the equivalent is:

```bash
colcon build --packages-select rascl_wp3_ss26_group16 --symlink-install
source install/setup.bash
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

Task 2 remains active and waits for cube positions. Example cylindrical input:

```bash
ros2 topic pub --once /cube_pose_cylindrical geometry_msgs/msg/Point \
  "{x: 0.15, y: 0.75, z: 0.03}"
```

For this convenience topic, the `Point` fields mean:

- `x`: radius `r` in metres
- `y`: angle `theta` in radians
- `z`: cube-center height in metres

The tasksheet-compatible Cartesian input is:

```bash
ros2 topic pub --once /goal_poses geometry_msgs/msg/Point \
  "{x: 0.15, y: 0.0, z: 0.03}"
```

For `/goal_poses`, the fields are ordinary Cartesian `x`, `y`, and `z` values.
The Cartesian radius `sqrt(x² + y²)` must be between `0.13 m` and `0.21 m`.

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

## Configuration summary

- Task 1 controller mode: CSP
- Task 1 sample period: `0.02 s`
- Task 2 feasible radius: `0.13–0.21 m`
- Task 2 feasible angle: `-pi/2–pi/2 rad`
- Task 2 fixed goal: `r=0.19 m`, `theta=-1.5 rad`
- Task 2 minimum-jerk segment duration: `4.0 s`

See [docs/task1.md](docs/task1.md), [docs/task2.md](docs/task2.md), and
[docs/cube_placement.md](docs/cube_placement.md) for details.
