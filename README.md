# Odyssey

**Odometry and motion control template for VEX V5, built on [PROS](https://pros.cs.purdue.edu/).**

Odyssey gives your robot a constantly-updated (x, y, heading) position on the field and a
full set of motion functions that use it — inspired by [LemLib](https://github.com/LemLib/LemLib)
and [EZ-Template](https://github.com/EZ-Robotics/EZ-Template), and by 5225A's
*Introduction to Position Tracking*.

📖 **[Read the full documentation](https://YOUR_GITHUB_USERNAME.github.io/odyssey/)**

## Features

- **Arc-based odometry** — accurate position tracking with any combination of
  tracking wheels and a V5 inertial sensor. Falls back to drive motor encoders
  automatically if you have no tracking wheels, and to wheel-based heading if
  the IMU fails mid-match.
- **PID motions** — `turnToHeading`, `turnToPoint`, `swingToHeading`,
  `moveToPoint`, all with configurable exit conditions, speed limits, slew
  rate control, and motion chaining (`minSpeed` / `earlyExitRange`).
- **Boomerang controller** — `moveToPose` drives to a position *and* a final
  heading in one smooth arc.
- **Pure pursuit** — follow paths made in [path.jerryio](https://path.jerryio.com/)
  from the SD card, or define waypoints in code.
- **Async motions** — start a motion, do other things (`waitUntil(10)` then
  raise the intake), then `waitUntilDone()`.
- **Driver control** — tank, arcade, and curvature drive with customizable
  exponential input curves.

## Quick example

```cpp
#include "odyssey/api.hpp"

void autonomous() {
    chassis.setPose(0, 0, 0);              // where the robot starts
    chassis.moveToPoint(0, 24, 4000);      // drive 24" forward
    chassis.turnToHeading(90, 1000);       // face 90 degrees
    chassis.moveToPose(24, 24, 90, 4000);  // arrive at (24, 24) facing 90
    chassis.follow("skills.txt", 12, 8000); // pure pursuit from the SD card
}
```

## Installation

1. Create a PROS project (`pros c create my-robot` or the PROS VS Code extension).
2. Copy `include/odyssey/` into your project's `include/` folder.
3. Copy `src/odyssey/` into your project's `src/` folder.
4. Copy `example/main.cpp` over your `src/main.cpp` and edit the ports and
   dimensions to match your robot.

See the [installation guide](https://YOUR_GITHUB_USERNAME.github.io/odyssey/getting-started/installation/)
for full instructions.

## Documentation

The docs live in `docs/` and are built with [MkDocs Material](https://squidfunk.github.io/mkdocs-material/).
They deploy to GitHub Pages automatically on every push to `main` (see
`.github/workflows/docs.yml`). To preview locally:

```sh
pip install mkdocs-material
mkdocs serve
```

## Credits

- [LemLib](https://github.com/LemLib/LemLib) — API design and motion algorithms
- [EZ-Template](https://github.com/EZ-Robotics/EZ-Template) — swing turns and ease-of-use philosophy
- 5225A Pilons — *[Introduction to Position Tracking](http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf)*, the basis of the odometry math

## License

MIT — see [LICENSE](LICENSE).
