# Cube placement and feasible region

Task 2 cube positions are expressed in Cartesian task-frame coordinates.

## Cube dimensions

- Footprint: `40 × 40 mm`
- Height: `41 mm`
- Nominal centre above the plate: `20.5 mm`

## Task 1 placement

- Cube 1: `r=230 mm`, `theta=-0.75 rad`
- Cube 2: `r=170 mm`, `theta=0 rad`
- Cube 3: stacked on cube 2 at `r=170 mm`, `theta=0 rad`
- Goal: on the right side of the robot

The cubes are moved individually and stacked at the goal in the order 1–2–3.

## Task 2 feasible region

- Cube-centre `x`: `-400–400 mm`
- Cube-centre `y`: `30–320 mm`


These Cartesian bounds are the node's initial input filter, not a guarantee
that a point is reachable. The complete approach, grasp, and placement sequence
must also produce Robotics Toolbox IK solutions inside the arm joint limits.

