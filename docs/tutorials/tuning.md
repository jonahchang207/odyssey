# Tuning the PIDs

The default gains in the example get the robot moving, but every drivetrain
is different. Plan on 20-30 minutes with the robot on a field.

## How PID works (30-second version)

Each tick, the controller computes an output from the error
(target − current):

- **kP** (proportional) — the workhorse. Output proportional to error.
- **kI** (integral) — accumulates leftover error. Rarely needed on
  drivetrains; leave at 0 unless the robot consistently stops short.
- **kD** (derivative) — reacts to how fast the error is shrinking. Damps
  oscillation and overshoot.

## Tune angular first

Turning affects driving (moveToPoint uses both controllers), so tune the
angular controller first. Use a simple test routine:

```cpp
void autonomous() {
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(90, 2000);
    chassis.waitUntilDone();
    pros::delay(1000);
    chassis.turnToHeading(0, 2000);
}
```

1. Set `kI = 0`, `kD = 0`, and start with the example `kP`.
2. Raise `kP` until the robot turns quickly and **oscillates slightly**
   around the target (overshoots, comes back, overshoots less...).
3. Raise `kD` until the oscillation just disappears. If the robot gets
   twitchy or grinds, `kD` is too high.
4. Repeat 2-3: you can usually push `kP` higher once `kD` is damping it.
5. Test with several angles (45°, 90°, 180°) — a tune that only works for
   90° turns has too little `kP` or too much `kD`.

## Then tune lateral

Same procedure with a drive test:

```cpp
chassis.setPose(0, 0, 0);
chassis.moveToPoint(0, 24, 4000);
```

Test at 24" and 48". Watch for:

- **Stops short, no oscillation** → raise `kP`.
- **Overshoots / wiggles at the end** → raise `kD` (or lower `kP`).
- **Jerks violently at the start** → lower the `slew` value in your lateral
  settings (it limits acceleration). If the start feels sluggish, raise it.

## Exit conditions

Once the gains are good, tighten the exit conditions to stop wasted time at
the end of motions:

| Field | Meaning | Make smaller when... | Make larger when... |
| ----- | ------- | -------------------- | ------------------- |
| `smallError` / `smallErrorTimeout` | "settled" window | motions end sloppy | motions hang at the end |
| `largeError` / `largeErrorTimeout` | "good enough" bailout | accuracy matters | speed matters |

A motion exits when error stays inside `smallError` for
`smallErrorTimeout` ms, **or** inside `largeError` for `largeErrorTimeout`
ms, **or** at the motion's overall timeout.

## Verifying odometry accuracy

Bad odometry looks exactly like a bad tune, so verify tracking before
blaming the PID:

1. With the pose printed on the brain screen, **push the robot by hand**
   in a big square back to its start. The pose should return to within
   ~1" and ~2°.
2. If x/y drift when you **spin the robot in place**, a tracking wheel
   `offset` is wrong (see below).
3. If distances are consistently a few percent off, the wheel `diameter`
   is wrong: drive an exact distance and scale the diameter by
   `actual / reported`.

### Calibrating offsets by spinning

Spin the robot in place exactly 10 times (use the printed heading: 3600°).
If the pose translates while spinning, adjust the offset of each tracking
wheel:

```text
true offset = measured offset + (drift per full rotation) / (2 * pi)
```

Tweak until 10 spins move the pose less than an inch.

!!! tip "IMU scaling"
    If heading reads 3590° after exactly 10 physical rotations, your IMU
    under-reports by ~0.3%. Small IMU error is normal; tracking-wheel
    heading (two parallel wheels, no IMU) can actually beat a poorly-scaled
    IMU if your build quality is good.
