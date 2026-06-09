# Odyssey

**Odometry and motion control template for VEX V5, built on [PROS](https://pros.cs.purdue.edu/).**

Odyssey gives your robot a constantly-updated (x, y, heading) position on the field and a
full set of motion functions that use it — inspired by [LemLib](https://github.com/LemLib/LemLib)
and [EZ-Template](https://github.com/EZ-Robotics/EZ-Template), and by 5225A's
*Introduction to Position Tracking*.

📖 **[Read the full documentation](https://jonahchang207.github.io/odyssey/)**

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

Odyssey installs as a PROS template, just like LemLib:

```sh
# one-time: register the Odyssey depot
pros c add-depot odyssey https://raw.githubusercontent.com/jonahchang207/odyssey/depot/stable.json

# inside your PROS project:
pros c apply odyssey
```

Then copy [`src/main.cpp`](src/main.cpp) from this repo into your project and
edit the ports and dimensions to match your robot. Prefer hacking on the
source directly? Copy `include/odyssey/` and `src/odyssey/` into your project
instead. See the
[installation guide](https://jonahchang207.github.io/odyssey/getting-started/installation/)
for all options.

## Team workflow & the Odyssey CLI

The repo uses a two-branch model: **`main`** is stable and protected (changes
arrive only through pull requests), **`dev`** is the everyday working branch.
A menu-driven CLI wraps the whole workflow so nobody needs to memorize git:

```powershell
.\odyssey          # interactive menu
.\odyssey commit   # or jump straight to: status, commit, push, sync,
                   # branch, pr, merge, release, setup
```

It handles conventional-prefix commits (including a robotics-specific
`tune:` type), dev → main pull requests, semver releases with
auto-generated notes, and one-time GitHub repo setup including branch
protection. See the
[workflow guide](https://jonahchang207.github.io/odyssey/tutorials/workflow/).

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
