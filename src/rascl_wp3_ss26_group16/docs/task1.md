# Task 1 — Offline motion planning

The node implements the three-cube stacking task as offline joint-space
trajectories:

- Reads `cube_1_to_goal.csv`, `cube_2_to_bottom.csv`, `cube_2_to_goal.csv`,
  and `cube_3_to_goal.csv` waypoint files
- Generates fifth-order minimum-jerk samples for every waypoint segment
- Writes the generated position, velocity, and acceleration samples to the
  configured output directory
- Executes each trajectory through `joint_trajectory_controller`
- Uses the fixed segment order cube 1 to goal, cube 2 to bottom,
  cube 3 to goal, then cube 2 to goal

Each input CSV contains a `duration` column followed by the four joint columns.
Physical waypoints are deliberately data, not source code: they must be filled
with the collision-checked joint configurations for the measured cube placement.
