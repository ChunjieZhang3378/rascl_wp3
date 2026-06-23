# Task 1 — Offline motion planning

The node implements the three-cube stacking task as offline joint-space
trajectories:

- Reads `cube_1.csv`, `cube_2.csv`, and `cube_3.csv` waypoint files
- Generates fifth-order minimum-jerk samples for every waypoint segment
- Writes the generated position, velocity, and acceleration samples to the
  configured output directory
- Executes each trajectory through `joint_trajectory_controller`
- Uses the fixed pick and stack order cube 1, cube 2, cube 3

The input format is documented in `trajectories/input/README.md`. Physical
waypoints are deliberately data, not source code: they must be filled with the
collision-checked joint configurations for the measured cube placement.
