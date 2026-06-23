# RASCL Hardware Interface

`rascl_hardware_interface` is a `ros2_control` system plugin for the RASCL
robot. It uses SOEM to communicate with the four Faulhaber drives over
EtherCAT and implements homing and position control through the CiA 402 object
dictionary.

The exported plugin is:

```text
rascl_hardware_interface/RasclHardwareInterface
```

## Lifecycle and control flow

The hardware interface implements the standard `ros2_control` lifecycle:

1. `on_init()` parses and validates the URDF hardware and joint parameters. It
   does not access or move the real robot.
2. `on_configure()` opens the configured Ethernet adapter, discovers the
   EtherCAT slaves, maps CSP process data, and enters OPERATIONAL state.
3. `on_activate()` resets drive faults, selects Homing mode, enables and homes
   all joints, and then switches the drives to CSP mode. This step
   can move the physical robot.
4. `read()` receives each statusword and actual position through TxPDO process
   data and estimates joint velocity from consecutive position samples.
5. `write()` validates and converts position commands, clamps joints with URDF
   limits, and sends CSP targets through RxPDO process data.
6. `on_deactivate()` disables drive voltage, and `on_cleanup()` closes the SOEM
   connection.

The arm joints use homing method 28. `end_effector_joint` uses homing method 37.

## Hardware parameters

The parameters are defined in the `<ros2_control>` section of
`rascl_description/urdf/rascl.urdf`.

| Parameter | Required | Description |
| --- | --- | --- |
| `adapter` | Yes | Network-interface name used by the SOEM master. |
| `use_fake_hardware` | No | Currently ignored; configuration always opens the real EtherCAT adapter. |
| `slave_id` | No | EtherCAT slave number; defaults to the joint index plus one. |
| `drive_units_per_radian` | Yes | Conversion factor between ROS radians and drive position counts. |

Each joint must provide one `position` command interface and at least the
`position` and `velocity` state interfaces. A command interface may provide
both `min` and `max` limits or neither. Commands outside configured limits are
clamped before conversion to drive units. The current end-effector command
interface intentionally has no software position limits.

## Exported interfaces

Each joint exports one command interface:

- `position`: target position in radians.

Each joint exports four state interfaces:

- `position`: measured position converted to radians.
- `velocity`: velocity in radians per second, estimated by finite difference.
- `actual_position`: raw drive position count represented as a `double`.
- `status_word`: raw CiA 402 statusword represented as a `double`.


## Safety

- Clear the robot workspace and prepare the emergency stop before activation.
- Activation automatically enables and homes all configured drives. 
- Verify the adapter, slave IDs, homing methods, and conversion factors before
  applying power.
- URDF limits are a final software boundary, not a replacement for mechanical
  stops, drive limits, or an emergency-stop circuit.
