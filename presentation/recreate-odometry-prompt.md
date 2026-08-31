# The Prompt — Recreate Odyssey's Odometry

Copy everything below the line and hand it to a coding assistant. It's written
the way I'd actually explain it to someone — but it still has all the math and
the gotchas, so the thing it builds should come out basically the same as mine.

---

Hey, I want your help building an odometry system for a robot — basically the
part that figures out where the robot is on the field while it drives. It's a
tank-drive (differential) robot, flat field, 2D. I want it to keep track of an
`(x, y, heading)` pose by reading sensors a hundred times a second and adding
up all the little movements.

I'm running a VEX V5 robot on PROS in C++, so do it in C++ unless I say
otherwise. One thing though: keep the actual math in plain, normal functions
so I can test it on my laptop without a robot plugged in. Don't bury it in
hardware calls.

**The method I want is arc-based odometry** — the one from the 5225A Pilons
"Introduction to Position Tracking" paper. Please don't just do the lazy
two-wheel-difference version. I want each tick treated like a little arc of a
circle, with the chord/half-angle correction. That's the whole point.

### What it reads

Up to five sensors, and any of them might not be there, so handle nulls:
- two "vertical" tracking wheels that face forward,
- two "horizontal" tracking wheels that face sideways,
- one inertial sensor (a gyro / IMU).

Every 10 ms, on a background thread, it should look at how much each sensor
changed since last time and update the pose. And I need `getPose()` and
`setPose()` to be safe to call from other threads.

### The conventions — please get these exactly right, this is where everything breaks

Honestly most of the pain in odometry is just sign and angle mistakes, so let
me be really clear:

- Everything is in **inches**.
- There are two different angle systems and you have to keep them straight:
  - **Inside the math, use radians** the normal way: `0` points along `+x`,
    and turning counterclockwise is positive.
  - **For anything I call from outside, use compass degrees:** `0` faces `+y`
    (away from the driver), and clockwise is positive.
  - To go between them: `compass = 90 - degrees(math_angle)`, and the same in
    reverse. Only convert right at the edge where I call your functions —
    never mix the two systems in the middle of the math. That's a classic way
    to get a robot that drives sideways into a wall.
- Field: `+x` is to the right, `+y` is away from me at the driver station.
- On the robot itself: `y` is forward, `x` is to the right.
- Wheel offsets are signed distances from the robot's tracking center. For a
  forward wheel, **negative means it's to the left** of center. For a sideways
  wheel, **negative means it's behind** center.
- The IMU reads clockwise-positive in degrees, but my internal angle is
  counterclockwise-positive in radians — so a positive reading from the IMU is
  actually a *negative* change internally. Easy to forget.

### The pieces

A `Pose` is just `x, y, theta`. A `TrackingWheel` wraps an encoder plus the
wheel diameter, its signed offset, and an optional gear ratio (default 1), and
it can tell me total inches traveled — which is basically
`encoder_revs * gearRatio * PI * wheelDiameter`. Bundle the five sensor
pointers in an `OdomSensors` struct where any can be null.

Quick tip to bake in: real VEX omni wheels aren't the size on the box, so use
measured diameters — 2" wheels are really ~2.125", and 3.25" wheels are 3.25",
4" ones are about 4.18" for the old style. Using the real numbers noticeably
improves accuracy.

### What happens every 10 ms

Keep the pose in radians internally and remember each sensor's last reading.
Each tick:

1. Get the change in each wheel since last time (`current - previous`), then
   save the current values for next time.

2. Figure out how much we turned this tick (call it `dTheta`, radians, CCW
   positive). Use whatever's available, in this order of preference:
   - the **IMU** first: `dTheta = -radians(imu_now - imu_prev)`. If the IMU
     gives back a garbage/non-finite value this tick, just skip it and use the
     next option.
   - else **two vertical wheels**: `(dV1 - dV2) / (offsetV1 - offsetV2)`
     (don't divide by zero if the offsets are equal).
   - else **two horizontal wheels**: `(dH2 - dH1) / (offsetH1 - offsetH2)`
     — note the order is flipped on these; again guard the divide.
   - if there's nothing, `dTheta = 0` and we just track straight-line motion.

3. A wheel that's off-center sweeps a little arc even when the robot only
   spins, so subtract that out to get the real translation:
   - forward part of a vertical wheel: `dV - offsetV * dTheta`
   - sideways part of a horizontal wheel: `dH + offsetH * dTheta`
   If I gave you two wheels of the same kind, average their answers — it
   cancels noise. Now you've got `forwardArc` and `rightwardArc`.

4. Turn those arcs into straight chords (this is the part people skip, don't
   skip it):
   - if `dTheta` is 0, just use `forwardArc` and `rightwardArc` as-is
     (otherwise you'd divide by zero),
   - otherwise multiply each by `2 * sin(dTheta/2) / dTheta`. So
     `localY = 2*sin(dTheta/2) * forwardArc / dTheta`, same idea for `localX`.

5. Rotate that little chord into field coordinates using the **average**
   heading over the tick (using the average, not the start, is what makes it
   accurate), and add it on:
   ```
   avg = pose.theta + dTheta/2
   pose.x += localY*cos(avg) + localX*sin(avg)
   pose.y += localY*sin(avg) - localX*cos(avg)
   pose.theta += dTheta
   ```

That's it, really — the core is only about 60 lines.

### Startup and threading

Have an init function that reads every sensor once to set the starting
baselines, then kicks off a loop that runs the update and sleeps 10 ms over and
over. Put a mutex around the pose so reading and writing it is safe. Default
the robot to the origin facing compass 0 (which is `PI/2` in the internal
radians).

### Things that'll go wrong if you don't handle them

- If I don't have a forward tracking wheel, let me hand you the two drive
  motor groups as stand-in wheels (offsets at plus/minus half the track
  width), converting motor turns to inches. It's less accurate because drive
  wheels slip, but sometimes that's all someone has.
- If the IMU dies in the middle of a match, don't crash — just quietly fall
  back to the wheel-based heading for that tick.
- No sideways wheel? Then sideways motion just reads as zero. That's fine,
  just mention it's blind to getting shoved sideways.
- Keep heading continuous — if I spin twice clockwise it should read 720, not
  wrap back to 0. The driving code worries about wrapping, not the tracker.

### The functions I want to call

`setSensors(...)`, `initOdometry()`, `update()`, `getPose(radians=false)`, and
`setPose(pose, radians=false)`. By default `getPose` should hand me compass
degrees, and `setPose` should take compass degrees and convert in. Throw in
small radians/degrees converters and an angle-wrap helper too.

### How I'll know it actually works

Please write me a little test I can run without a robot — feed it fake sensor
changes and check the pose comes out right. Make it show:
1. Drive 24 inches straight forward from the origin → I should end up around
   `(0, 24)` still facing 0.
2. **Spin in place → the position should basically not move.** This is the big
   one; if the position drifts while spinning, your offset math is wrong.
3. A quarter-circle arc should land exactly where the geometry says it should
   — that's the test that catches a missing chord correction.
4. Drive a 48-inch square back to the start: with clean data it should come
   home almost perfectly, and if you add a little IMU drift, the error should
   visibly build up. That's realistic.

And drop this in a comment somewhere because it saved me hours: if it drifts
while *spinning*, the wheel offsets are wrong; if it drifts while driving
*straight*, the wheel diameter is wrong.

### Wrapping up

Split it into sensible files (pose, tracking wheel, sensors, the odometry
itself), comment the public stuff, include those tests, and write a short
readme that derives steps 3–5 with the equations so future-me understands it.
It should compile clean, the core should stay tiny, and all four tests should
pass — especially the spin-in-place one.

If you need real numbers to make a demo config, just ask, or use normal VEX
defaults: track width 11.5", 3.25" drive wheels, 2.125" tracking wheels, the
forward wheel 1.25" left of center, the sideways wheel 2.5" behind center.
