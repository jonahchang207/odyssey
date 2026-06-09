# The Coordinate System

Odyssey uses the same field conventions as LemLib, so diagrams and paths made
for LemLib transfer directly.

## Position

- Units are **inches**.
- `+y` points **away from your driver station** ("up" the field).
- `+x` points **to the right**.
- `(0, 0)` is wherever you say it is — `setPose` defines the origin. Most
  teams use the center of the field or their starting tile.

## Heading

Headings are **compass bearings** in degrees:

- `0°` = facing `+y` (away from the driver station)
- `90°` = facing `+x` (right)
- `180°` = facing `-y` (toward the driver station)
- `270°` = facing `-x` (left)
- Clockwise is positive, like a compass.

```text
                 0°
                 +y
                 ▲
                 │
   270° ◄────────┼────────► 90°
   -x            │            +x
                 │
                 ▼
                -y
                180°
```

`getPose().theta` is continuous — after two full clockwise rotations it reads
`720`, not `0`. Turn targets are wrapped automatically, so
`turnToHeading(90, ...)` always takes the shortest path (unless you force a
direction).

## Setting the starting pose

At the start of autonomous, tell the robot where it physically is:

```cpp
void autonomous() {
    // 24" right of center, at the field edge, facing up-field
    chassis.setPose(24, -60, 0);
    ...
}
```

Measure to the **tracking center** of the robot (midway between the drive
wheels), not the edge of the chassis.

!!! note "Radians for the math-inclined"
    Every pose function takes an optional `radians` flag. With
    `getPose(true)` / `setPose(pose, true)` angles are **standard math
    radians**: 0 = +x, counterclockwise positive. The library uses this
    frame internally — see [The Odometry Math](../explanation/odometry-math.md).
