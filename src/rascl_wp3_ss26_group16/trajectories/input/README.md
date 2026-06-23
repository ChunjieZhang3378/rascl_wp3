# Task 1 trajectory inputs

Provide exactly these files:

- `cube_1.csv`
- `cube_2.csv`
- `cube_3.csv`

They execute in that order. Each file describes joint-space waypoints:

```csv
duration,shoulder_joint,upperarm_joint,lowerarm_joint,end_effector_joint
0.0,0.0,0.0,0.0,0.0
2.0,0.3,-0.2,0.4,0.0
1.0,0.3,-0.2,0.4,0.5
```

`duration` is the travel time in seconds from the preceding row. The first row
must use duration zero. Joint values are radians. Every segment starts and ends
with zero velocity and acceleration and is sampled as a fifth-order
minimum-jerk trajectory.

Use only collision-checked waypoints measured for the real cube positions.
