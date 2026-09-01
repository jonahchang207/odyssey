# Driver Control

Odyssey provides three drive schemes plus input shaping. All of them go in
`opcontrol()`:

The helpers command the same motor groups used by autonomous motion. Call one
drive method repeatedly; see the [Chassis API](../api/chassis.md#driver-control)
for exact signatures.

```cpp
void opcontrol() {
    while (true) {
        const int throttle = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        const int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(throttle, turn);
        pros::delay(20);
    }
}
```

## Drive schemes

### Tank

Each stick controls one side directly.

```cpp
chassis.tank(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
             master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
```

### Arcade

One input for speed, one for turn rate. Simple and predictable.

```cpp
chassis.arcade(throttle, turn);
```

### Curvature

Like arcade, but the turn stick controls the **curvature of the arc** rather
than the turn rate. The same stick deflection produces the same shaped arc
whether you are creeping or at full speed, which most drivers find more
natural once they adjust. At zero throttle it falls back to spinning in
place.

```cpp
chassis.curvature(throttle, turn);
```

## Input curves

An `ExpoDriveCurve` reshapes stick input for finer control:

```cpp
//                              deadband, minOutput, curve
odyssey::ExpoDriveCurve throttleCurve(3, 10, 1.019);
odyssey::ExpoDriveCurve steerCurve(3, 10, 1.019);

odyssey::Chassis chassis(drivetrain, lateralSettings, angularSettings, sensors,
                         &throttleCurve, &steerCurve);
```

- **deadband** — inputs below this are ignored. Fixes joysticks that don't
  quite return to zero (3-5 is typical).
- **minOutput** — the smallest output once you leave the deadband. Set it
  just high enough that the robot actually creeps instead of humming
  (8-12 is typical).
- **curve** — the exponential gain. `1` is linear. `1.01`-`1.05` makes the
  center of the stick progressively gentler while full deflection stays at
  full power. Try `1.019` and adjust to driver taste.

Pass `true` as the third argument of any drive function to bypass the curves
temporarily:

```cpp
chassis.arcade(throttle, turn, true); // raw input
```

## Brake mode

```cpp
chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST); // default, easiest on motors
chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE); // stops faster
chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);  // actively holds position
```

A common pattern is `HOLD` while climbing or defending and `COAST` otherwise,
toggled on a controller button.

!!! tip "Tune for the driver, not the graph"
    Raise deadband only enough to stop joystick drift, then adjust `minOutput`
    until the robot moves cleanly at low command. Change the curve last, with
    the primary driver testing it on a realistic route.

<div class="odyssey-next" markdown>

<p><strong>Need exact constructor fields or helper behavior?</strong><br>The reference pages list drive curves, brake modes, and utility functions.</p>

[Open the helper API →](../api/helpers.md#drive-curves)

</div>
