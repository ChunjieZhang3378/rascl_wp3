# RASCL Description

`rascl_description` contains the files that describe the RASCL robot to ROS 2.
These files define the robot's kinematic structure and geometry and provide the
configuration needed to publish and visualize the model.

## Robot Model

The main robot model is defined in [`urdf/rascl.urdf`](urdf/rascl.urdf). The
URDF describes:

- the robot's links and kinematic structure;
- fixed and revolute joints;
- joint origins, axes, and position limits;
- visual and collision geometry; and
- the interfaces that connect the model to ROS 2 Control.

We are currently using a cuboid as a gripper. We will update it with our own gripper soon, as it is still being refined and improved.

The model has a fixed connection from `world` to `base_link` and four revolute
joints:

| Joint | Parent link | Child link | Axis | Position limits |
| --- | --- | --- | --- | --- |
| `shoulder_joint` | `base_link` | `shoulder` | Z | -1.57 to 1.57 rad |
| `upperarm_joint` | `shoulder` | `upperarm_link` | Z | -1.57 to 1.57 rad |
| `lowerarm_joint` | `upperarm_link` | `lowerarm_link` | Z | -1.57 to 1.57 rad |
| `end_effector_joint` | `lowerarm_link` | `end_effector_link` | Y | -1.57 to 1.57 rad |

DAE meshes provide the visual geometry. STL meshes provide collision geometry
for the base, upper arm, lower arm, and end effector. The current end-effector
mesh is a simplified gripper model.

## Launch Files

### `display.launch.py`

Loads `urdf/rascl.urdf` by default and starts:

- `robot_state_publisher` to publish the robot transforms;
- `joint_state_publisher_gui` to set joint positions interactively; and
- RViz using `rviz/urdf.rviz`.

The launch file accepts the `model` and `rviz_config` arguments so that another
URDF or RViz configuration can be selected.

### `ros2_control.launch.py`

Loads the robot description, starts `controller_manager`, and spawns the joint
state broadcaster and joint trajectory controller configured in
`config/controllers.yaml`.

## ROS 2 Control Integration

This package includes ROS 2 Control information because the controllable joints
and their interfaces are part of the robot description. The `<ros2_control>`
section in `urdf/rascl.urdf` connects the described joints to
`rascl_hardware_interface/RasclHardwareInterface`.

Each revolute joint provides a `position` command interface and these state
interfaces:

- `position`;
- `velocity`;
- `actual_position`; and
- `status_word`.

[`config/controllers.yaml`](config/controllers.yaml) defines:

- `joint_state_broadcaster`, which publishes the joint states; and
- `joint_trajectory_controller`, which sends position trajectories to all four
  revolute joints.

[`launch/ros2_control.launch.py`](launch/ros2_control.launch.py) loads this
description and controller configuration. The implementation and operation of
the physical hardware are documented in the `rascl_hardware_interface`
package.
