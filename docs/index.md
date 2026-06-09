# Odyssey

**Odometry and motion control template for VEX V5, built on [PROS](https://pros.cs.purdue.edu/).**

Odyssey gives your robot a constantly-updated (x, y, heading) position on the
field, and a full set of autonomous motion functions that use it. It is
inspired by [LemLib](https://github.com/LemLib/LemLib) and
[EZ-Template](https://github.com/EZ-Robotics/EZ-Template).

```cpp
void autonomous() {
    chassis.setPose(0, 0, 0);              // where the robot starts
    chassis.moveToPoint(0, 24, 4000);      // drive 24" forward
    chassis.turnToHeading(90, 1000);       // face 90 degrees
    chassis.moveToPose(24, 24, 90, 4000);  // arrive at (24, 24) facing 90
    chassis.follow("skills.txt", 12, 8000); // pure pursuit from the SD card
}
```

## Features

<div class="grid cards" markdown>

- **Robust odometry**

    Arc-based position tracking with any combination of tracking wheels and
    an inertial sensor. Automatically falls back to drive encoders if you
    have no tracking wheels, and to wheel-based heading if the IMU dies
    mid-match.

- **PID motions**

    `turnToHeading`, `turnToPoint`, `swingToHeading`, and `moveToPoint` with
    configurable exit conditions, speed limits, slew rate control, and
    motion chaining.

- **Boomerang controller**

    `moveToPose` drives to a position *and* a final heading in one smooth
    arc, with automatic speed limiting through tight curves.

- **Pure pursuit**

    Follow paths drawn in [path.jerryio](https://path.jerryio.com/) straight
    from the micro SD card, or define waypoints in code.

- **Async motions**

    Start a motion, trigger your intake partway through with
    `waitUntil(10)`, then `waitUntilDone()`.

- **Driver control**

    Tank, arcade, and curvature drive with customizable exponential input
    curves, deadbands, and minimum outputs.

</div>

## Where to start

1. [Installation](getting-started/installation.md) — add Odyssey to a PROS project
2. [Hardware Setup](getting-started/hardware.md) — tracking wheel placement advice
3. [Configuration](getting-started/configuration.md) — describe your robot in code
4. [The Coordinate System](getting-started/coordinates.md) — how positions and headings work
5. [Your First Autonomous](getting-started/first-autonomous.md) — make the robot move

## Credits

- [LemLib](https://github.com/LemLib/LemLib) — API design and motion algorithms
- [EZ-Template](https://github.com/EZ-Robotics/EZ-Template) — swing turns and ease-of-use philosophy
- 5225A Pilons — *[Introduction to Position Tracking](http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf)*
