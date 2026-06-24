# Cube placement and feasible region

All cylindrical radii are measured from the centre of the robot base/shoulder
rotation axis to the centre of the cube.

## Cube dimensions

- Footprint: `40 × 40 mm`
- Height: `41 mm`
- Nominal centre above the plate: `20.5 mm`

The base collision mesh has an approximate radius of `80 mm`. At the minimum
Task 2 cube-centre radius of `130 mm`, the nearest cube face is approximately
`110 mm` from the base centre, leaving about `30 mm` radial clearance.

## Task 1 placement

- Cube 1: `r=230 mm`, `theta=-0.75 rad`
- Cube 2: `r=170 mm`, `theta=0 rad`
- Cube 3: stacked on cube 2 at `r=170 mm`, `theta=0 rad`
- Goal: on the right side of the robot

The cubes are moved individually and stacked at the goal in the order 1–2–3.

## Task 2 feasible region

- Minimum cube-centre radius: `130 mm`
- Maximum cube-centre radius: `210 mm`
- Shoulder angular region: `-pi/2–pi/2 rad`
- Fixed goal radius: `190 mm`
- Fixed goal angle: `-1.5 rad`, on the robot's right side

The minimum and maximum radii define the working collision-free and reachable
region for the current robot/gripper setup. The configured fixed goal is at a
radius of `190 mm`.

Cube faces must remain parallel to the gripper contact faces according to the
box plate. Only one cube is present during each Task 2 cycle.

## Grasp geometry

The controlled end-effector midpoint is placed `15 mm` above the centre of the
cube's top surface:

```text
end_effector_z = cube_center_z + 41 mm / 2 + 15 mm
```

For the nominal cube centre at `20.5 mm`, this gives an end-effector target
height of `56 mm` relative to the plate reference. No end-effector orientation
constraint is imposed by the analytical IK.
