# Your First Autonomous

With the chassis configured and calibrated, autonomous routines are short and
readable.

## The basics

```cpp
void autonomous() {
    // 1. always set the starting pose first
    chassis.setPose(0, 0, 0);

    // 2. drive somewhere. (x, y, timeout in ms)
    chassis.moveToPoint(0, 24, 4000);

    // 3. turn to a compass heading
    chassis.turnToHeading(90, 1000);

    // 4. drive to a position AND arrive at a heading (boomerang)
    chassis.moveToPose(24, 24, 90, 4000);
}
```

Every motion takes a `timeout` in milliseconds — the maximum time it may run.
This is your safety net: if the robot gets stuck on a game object, the motion
gives up instead of burning out motors for the rest of the match.

## Motions are asynchronous

By default, a motion call **returns immediately** and the motion runs in the
background. Calling another motion queues it behind the current one, so
sequential code still behaves sequentially:

```cpp
chassis.moveToPoint(0, 24, 4000);  // starts
chassis.turnToHeading(90, 1000);   // waits for the move, then turns
```

The power of async is doing other things mid-motion:

```cpp
chassis.moveToPoint(0, 48, 4000);
chassis.waitUntil(10);   // block until the robot has driven 10"
intake.move(127);        // ...then start the intake while still driving
chassis.waitUntilDone(); // block until the motion finishes
```

To make a motion blocking, pass `async = false` as the last argument.

!!! tip "`waitUntil` units"
    `waitUntil` measures **inches** for drive motions and **degrees** for
    turns and swings.

## Driving backwards

```cpp
// drive backwards to (0, 0)
chassis.moveToPoint(0, 0, 4000, {.forwards = false});

// back into a goal at (12, -24), rear facing 180
chassis.moveToPose(12, -24, 180, 4000, {.forwards = false});
```

## Optional parameters

Each motion accepts a params struct using designated initializers — set only
what you need:

```cpp
chassis.turnToHeading(270, 1500, {
    .direction = odyssey::AngularDirection::CW_CLOCKWISE, // force the long way
    .maxSpeed = 80,
});

chassis.moveToPoint(36, 36, 3000, {.maxSpeed = 100});
```

## Chaining motions smoothly

Normally each motion decelerates to a stop. For flowing multi-step routes,
give a motion a `minSpeed` and an `earlyExitRange`: it will exit while still
moving once it gets within that range, and the next motion takes over:

```cpp
// blast through (24, 24) at speed, then continue to (48, 0)
chassis.moveToPoint(24, 24, 2000, {.minSpeed = 70, .earlyExitRange = 8});
chassis.moveToPoint(48, 0, 3000);
```

## Swing turns

A swing turn locks one side of the drive and pivots around it — useful in
corners or against field walls:

```cpp
chassis.swingToHeading(90, odyssey::DriveSide::LEFT, 1500);
```

## Canceling motions

```cpp
chassis.cancelMotion();     // stop the current motion (queued ones still run)
chassis.cancelAllMotions(); // stop everything
```

## Next steps

- [Tune your PIDs](../tutorials/tuning.md) — required before motions are accurate
- [Pure pursuit paths](../tutorials/pure-pursuit.md) — smooth multi-point routes
