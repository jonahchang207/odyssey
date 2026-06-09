# The Odometry Math

This page derives the tracking algorithm in `src/odyssey/odometry.cpp`. It is
the arc-based method described in 5225A's *Introduction to Position
Tracking*, adapted to Odyssey's conventions. You don't need this to use the
library — it's here so you can verify, debug, and extend it.

## Conventions

Internally (everything on this page):

- $\theta$ is in **standard math radians**: $0$ = facing $+x$,
  counterclockwise positive. (The user-facing API converts to compass
  degrees: $\theta_{compass} = 90° - \theta_{math}$.)
- Vertical wheel offsets $s_V$: negative = left of the tracking center,
  positive = right.
- Horizontal wheel offsets $s_H$: negative = behind the tracking center,
  positive = in front.
- The robot's local frame: $y$ = forward, $x$ = rightward.

The tracking loop runs every 10 ms. Each tick it computes the change in pose
and accumulates it.

## Step 1 — sensor deltas

For each tracking wheel, the distance traveled since the last tick:

$$\Delta V_i = V_i - V_i^{prev}, \qquad \Delta H_i = H_i - H_i^{prev}$$

## Step 2 — heading change

Preferred source: the IMU. Its rotation is clockwise-positive, so

$$\Delta\theta = -\operatorname{rad}(\text{imu} - \text{imu}^{prev})$$

If the IMU is absent (or returns an error this tick), heading comes from two
parallel wheels. Both vertical wheels measure the same forward translation
plus a rotation term that depends on their offset, so subtracting them
isolates the rotation:

$$\Delta\theta = \frac{\Delta V_1 - \Delta V_2}{s_{V1} - s_{V2}}$$

(For two horizontal wheels the sign flips: $\Delta\theta = \frac{\Delta H_2 - \Delta H_1}{s_{H1} - s_{H2}}$.)

## Step 3 — removing rotation from the wheel readings

When the robot rotates by $\Delta\theta$, a wheel mounted off-center sweeps
an arc even if the robot doesn't translate. A vertical wheel at rightward
offset $s_V$ moves **forward** by $s_V\,\Delta\theta$ under counterclockwise
rotation; a horizontal wheel at forward offset $s_H$ moves **left** by
$s_H\,\Delta\theta$. Subtracting these gives the robot's actual local
translation arcs:

$$\text{forward arc} = \Delta V - s_V\,\Delta\theta$$

$$\text{rightward arc} = \Delta H + s_H\,\Delta\theta$$

If two wheels of the same orientation exist, Odyssey averages their
estimates, which cancels some measurement noise.

## Step 4 — arc to chord

Over one tick the robot is modeled as moving along a circular arc of total
rotation $\Delta\theta$. The straight-line displacement (chord) of an arc of
length $L$ is:

$$\text{chord} = 2\sin\!\left(\frac{\Delta\theta}{2}\right)\frac{L}{\Delta\theta}$$

Applying this to both local axes:

$$\Delta y_{local} = 2\sin\!\left(\tfrac{\Delta\theta}{2}\right)\frac{\Delta V - s_V \Delta\theta}{\Delta\theta},
\qquad
\Delta x_{local} = 2\sin\!\left(\tfrac{\Delta\theta}{2}\right)\frac{\Delta H + s_H \Delta\theta}{\Delta\theta}$$

When $\Delta\theta = 0$ both reduce to the raw deltas (the limit of the
formula), which the code special-cases to avoid dividing by zero.

## Step 5 — rotating into the field frame

The chord is oriented at the **average** heading of the tick,
$\bar\theta = \theta + \frac{\Delta\theta}{2}$ — this half-angle trick is
what makes the arc model exact for constant-curvature motion. With forward
unit vector $(\cos\bar\theta, \sin\bar\theta)$ and rightward unit vector
$(\sin\bar\theta, -\cos\bar\theta)$:

$$x \mathrel{+}= \Delta y_{local}\cos\bar\theta + \Delta x_{local}\sin\bar\theta$$

$$y \mathrel{+}= \Delta y_{local}\sin\bar\theta - \Delta x_{local}\cos\bar\theta$$

$$\theta \mathrel{+}= \Delta\theta$$

That's the whole algorithm — about 60 lines of real code.

## Error behavior (why the recommendations are what they are)

- **Errors compound through rotation.** A heading error of $\epsilon$
  radians misdirects all subsequent translation by $\approx \epsilon \times$
  distance. This is why the IMU (or careful two-wheel heading) matters most.
- **No horizontal wheel = blind to sideways motion.** $\Delta x_{local}$ is
  simply 0, so bumps and drift accumulate silently.
- **Offsets only matter while rotating.** If your pose drifts when spinning
  in place, the offsets are wrong; if it drifts while driving straight, the
  wheel diameter or mounting alignment is wrong. This separation is what
  makes the [calibration procedure](../tutorials/tuning.md#verifying-odometry-accuracy)
  work.
