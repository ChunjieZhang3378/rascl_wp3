# rascl_wp3
# RASCL ROS2 Workspace

This is the group 16 root directory of of the ROS2 workspace of used to run the RASCL robot.

## Testing

The `rascl_hardware_interface` package contains tests for its
initialization and configuration validation. The tests parse the project's real
URDF and check valid and invalid hardware parameters and joint interfaces. They
do not connect to EtherCAT, enable the motors, or move the robot.

The unit tests check that:

- the hardware interface initializes from the project URDF;
- the expected state and command interfaces are exported;
- a missing EtherCAT adapter is rejected;
- a missing `drive_units_per_radian` parameter is rejected;
- a joint without a position command interface is rejected;
- a joint without position limits is accepted; and
- a joint without a velocity state interface is rejected.

The test suite also runs `cppcheck`, `cpplint`, `lint_cmake`, `uncrustify`, and
`xmllint` to check static analysis, C++ style, CMake style, C++ formatting, and
XML validity. 

The tests only check `on_init()` and interface export. They do not test
EtherCAT configuration, activation, homing, cyclic reads and writes, or physical
motor movement.

Build and run the tests inside the ROS 2 container:

```bash
./rosws.sh
rosbuild
rossetup
ldconfig
colcon test --packages-select rascl_hardware_interface \
  --event-handlers console_direct+
colcon test-result --verbose
```

The test source is located at
`src/rascl_hardware_interface/test/test_generic_system.cpp`.

## RViz Visualization

The RViz display launch file starts `robot_state_publisher`,
`joint_state_publisher_gui`, and RViz with the robot model loaded from
`src/rascl_description/urdf/rascl.urdf`.

### Launch Visulization

1. Allow the container to open GUI windows on the host:

   ```bash
   xhost +local:root
   ```

2. Start the ROS workspace container from the repository root:

   ```bash
   ./rosws.sh
   ```

3. Inside the container, build the workspace:

   ```bash
   rosbuild
   ```

4. Source the workspace setup:

   ```bash
   rossetup
   ```

5. Launch the robot visualization:

   ```bash
   ros2 launch rascl_description display.launch.py
   ```

RViz should open with the robot model, TF frames, and a grid display. Use the
joint state publisher GUI window to move the revolute joints.

We are currently using a cuboid as a gripper. We will update it with our own gripper soon, as it is still being refined and improved.

If RViz fails with `could not connect to display`, close the container, run
`xhost +local:root` on the host again, and restart the container with
`./rosws.sh`.

## ROS 2 Control

The ROS 2 control launch file starts `robot_state_publisher`,
`controller_manager`, `joint_state_broadcaster`, and the
`joint_trajectory_controller`. The hardware interface reads the robot model from
`src/rascl_description/urdf/rascl.urdf` and communicates with the Faulhaber
EtherCAT controllers through the network adapter configured as `robot_interface`. 
If the adapter id is wrong. Show the available adapter id and change it in rascl.urdf
```bash
ip link show
```

### Launch ROS2 Control

1. Start the ROS workspace container from the repository root:

   ```bash
   ./rosws.sh
   ```

2. Inside the container, build the workspace:

   ```bash
   ldconfig
   rosbuild
   ```

3. Source the workspace setup:

   ```bash
   rossetup
   ```

4. Check that the EtherCAT adapter is available:

   ```bash
   ip link show robot_interface
   ```

5. Launch ROS 2 control:

   ```bash
   ros2 launch rascl_description ros2_control.launch.py
   ```
   The joints will automatically be homed to the initial state. The homing method for arm and shoulder joints is 28. The upper arm cannot start with negative limit position becaue it will trigger intern limit and stop the homing process.
   The homing method for end effector is 35. It sets the current position as home because our gripper does not need specific home position.

6. In another sourced terminal, check that the controllers are loaded:

   ```bash
   docker exec -it rascl-gruppe16 bash
   rossetup
   ros2 control list_controllers
   ```

7. Check that the hardware command/state interfaces are available:

   ```bash
   ros2 control list_hardware_interfaces
   ```

The current hardware interface always initializes SOEM/EtherCAT during
configuration. Launching without the real hardware connected, or without the
`robot_interface` adapter, will fail during controller manager startup.

### Control Real Hardware

Use a second terminal inside the container after ROS2 Control launch:

```bash
docker exec -it rascl-gruppe16 bash
rossetup
```

Confirm that the trajectory controller is active:

```bash
ros2 control list_controllers
```

Echo the joint states in a different terminal:

```bash
while true; do
  ros2 topic echo --once /dynamic_joint_states
  sleep 1
done
```


Test motion:

```bash
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [shoulder_joint, upperarm_joint, lowerarm_joint, end_effector_joint], points: [{positions: [0.75, 0.75, 0.75, 0.75], time_from_start: {sec: 10, nanosec: 0}}]}}"
```


Return the robot to zero:

```bash
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [shoulder_joint, upperarm_joint, lowerarm_joint, end_effector_joint], points: [{positions: [0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 10, nanosec: 0}}]}}"
```  

Before sending motion commands, make sure the robot has clearance, the power
supply is current-limited appropriately, and nobody is backdriving the motors.
