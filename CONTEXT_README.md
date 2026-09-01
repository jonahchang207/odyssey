# Odyssey GitHub Pages Handoff

This file is context for a future model improving the documentation website at
<https://jonahchang207.github.io/odyssey/>. The task is primarily a docs-site
UX, information architecture, visual design, and copy improvement task. Keep
the C++ library behavior stable unless a documentation mismatch requires a
small, explicitly justified correction.

## Project in one paragraph

Odyssey is an odometry and motion-control template for VEX V5 robots running
PROS. It tracks `(x, y, heading)` and provides PID turns/drives, swing turns,
boomerang `moveToPose`, pure-pursuit paths, asynchronous motion management, and
driver-control helpers. An optional Monte Carlo localization (MCL) layer uses
V5 Distance Sensors in shadow or correction mode, while raw odometry remains
the safe default. The target audience is VEX teams: students and mentors who
need to install the template, configure sensors, get a first autonomous
running, then tune and understand the system.

## Repository map

| Path | Role |
| --- | --- |
| `docs/` | MkDocs source for GitHub Pages |
| `docs/index.md` | Current landing page |
| `docs/getting-started/` | Installation, hardware, configuration, coordinates, first autonomous |
| `docs/tutorials/` | PID tuning, driver control, pure pursuit, team workflow/CLI |
| `docs/explanation/` | Odometry math and MCL architecture/validation |
| `docs/api/` | Chassis, odometry, localization, helper API reference |
| `docs/assets/logo.svg` | Material-style route and waypoint mark |
| `docs/assets/banner.svg` | Indigo/cyan Odyssey wordmark |
| `mkdocs.yml` | MkDocs Material theme, palette, extensions, navigation |
| `.github/workflows/docs.yml` | Publishes stable and developer-preview docs from `main` and `dev` |
| `include/odyssey/` | Public C++ headers; source of API truth |
| `src/odyssey/` | C++ implementation |
| `src/main.cpp` | Example robot configuration referenced by docs |
| `presentation/index.html` | Separate offline AP CS A slideshow/simulator; not deployed by MkDocs |
| `presentation/speaker-notes.md` | Notes for the separate slideshow |

The repository also contains PROS kernel headers/libraries under `include/pros/`
and `firmware/`; do not redesign or edit those for a Pages task.

## Current site structure

`mkdocs.yml` currently uses Material with navigation tabs/sections, search,
back-to-top, code-copy buttons, admonitions, tabbed content, tables, and
MathJax. Navigation is:

1. Home
2. Getting Started: Installation → Hardware Setup → Configuration → The Coordinate System → Your First Autonomous
3. Tutorials: Tuning the PIDs → Driver Control → Pure Pursuit Paths → Team Workflow & CLI
4. How It Works: The Odometry Math → Monte Carlo Localization
5. API Reference: Chassis → Odometry → Localization → Helpers

There is no custom CSS, no custom JS besides MathJax, no generated API docs,
and no site screenshots or diagrams referenced by the pages. The landing page
has a code example, a Material grid-card feature list, a five-step starting
path, and credits.

## Brand and visual direction

The documentation uses a Material-first navigation identity:

- Material indigo (`#4051b5`, `#283593`) communicates motion and control
- cyan (`#00acc1`, `#80deea`) marks live position, routes, and optional systems
- neutral white/light-lavender surfaces in light mode and charcoal surfaces in dark mode
- Roboto and Roboto Mono typography through Material for MkDocs
- a route-and-waypoint logo, rounded surfaces, restrained elevation, and pill actions
- diagrams use the same indigo/cyan semantics and high-contrast neutral labels

Do not reintroduce the former brass/copper steampunk treatment. Keep additions
close to Material Design conventions, accessible in both color modes, and
focused on navigation, robotics, coordinates, and movement.

## Technical truths the docs must preserve

- Units are inches. Public headings default to compass degrees: `0 = +y`
  (facing away from the driver station), clockwise positive. With
  `radians = true`, the math frame is standard radians: `0 = +x`,
  counterclockwise positive.
- Public API is normally available through `#include "odyssey/api.hpp"`.
- `Chassis::calibrate()` is called in `initialize()`, calibrates the IMU when
  requested, substitutes drive encoders for missing vertical tracking wheels,
  zeroes sensors, and starts 10 ms odometry tracking.
- Motion calls are asynchronous by default and queue behind a current motion.
  `waitUntil()` measures inches for drive motions and degrees for turns/swings;
  `waitUntilDone()` waits for completion.
- Pure pursuit accepts `std::vector<Waypoint>` or a micro-SD file in
  path.jerryio format: `x, y, speed` per line, ending with `endData`.
- MCL is optional. `getOdometryPose()` is always raw; the selected chassis
  pose is corrected only in correction mode after freshness/confidence gates.
  The full behavior is in `docs/explanation/monte-carlo-localization.md` and
  `docs/api/localization.md`.
- Configuration examples and API signatures should be checked against
  `include/odyssey/*.hpp` and `src/main.cpp`, not invented from memory.

## Current strengths

- The learning sequence is sensible for a new VEX team.
- Most pages use concrete, copyable C++ and shell examples.
- Hardware guidance includes practical sensor placement, wheel-diameter,
  offset, track-width, RPM, and encoder-direction advice.
- The coordinate-system page explicitly explains the two angle frames.
- The first-autonomous page gives a useful progression from one motion to
  chaining, reverse motion, speed limits, early exit, and swing turns.
- API pages expose actual signatures and parameter tables rather than only
  prose.
- MCL has unusually detailed architecture, safety gates, telemetry, replay,
  deterministic tests, and physical-validation guidance.

## Highest-value improvement opportunities

Prioritize these in roughly this order:

1. **Make the homepage decision-oriented.** In the first screen, state what
   Odyssey is, who it is for, the fastest install action, and a clear “first
   autonomous” route. Add obvious actions for Install, Configure, First
   Autonomous, API, GitHub, and Releases. Keep the attractive code sample but
   make its purpose and prerequisites explicit.
2. **Give readers a visual mental model.** Add a small branded architecture or
   field-coordinate diagram showing sensors → odometry → pose → motion
   controllers, plus the optional MCL branch. SVG or Mermaid is preferable to
   a heavy image pipeline. A coordinate diagram would reduce the most likely
   beginner confusion.
3. **Improve navigation by user intent.** Consider a short “Choose your path”
   section: new team, already using PROS, tuning a robot, following a path,
   or exploring localization. Keep the existing nav depth manageable on mobile.
4. **Make installation more scannable.** Surface prerequisites, the one-time
   depot command, the project command, expected result, and next link as a
   compact quickstart. Separate library users from contributors.
5. **Add consistent page affordances.** Use admonitions for prerequisites,
   safety notes, common mistakes, and “when to use this”; add explicit
   previous/next learning links where Material navigation alone is not enough.
6. **Improve API usability.** Keep API reference factual, but add a compact
   “when to use it” summary and cross-links from each concept to its tutorial.
   Ensure examples use the exact current signatures and avoid unexplained
   parameters.
7. **Use the brand consistently.** Extend the indigo/cyan Material system,
   style code blocks and links with care, and preserve comfortable contrast in
   both light and dark modes.
8. **Add trust and project-status signals.** Link to GitHub source, releases,
   issues/contributing, and license in prominent but non-distracting places.
   Do not claim benchmarks, compatibility, or production guarantees that are
   not present in the repository.

## Constraints and workflow

- Edit `docs/` and `mkdocs.yml` for the website. Add small assets under
  `docs/assets/` only when they materially improve comprehension.
- Keep internal links relative and preserve the navigation order unless there
  is a clear information-architecture reason to change it.
- Avoid changing library semantics while doing a docs redesign.
- Deployment runs on pushes to `main` or `dev` via
  `.github/workflows/docs.yml`. It publishes the `main` build at `/odyssey/`
  and the warning-marked `dev` build at `/odyssey/dev/` in one atomic
  `gh-pages` update.
- The repository guidance says work happens on `dev`; `main` is protected and
  changes land through a PR. Follow the existing branch/commit conventions.
- Do not edit `include/pros/`, `firmware/`, or `common.mk` for this task.

## Verification checklist

Before handing off a Pages change:

1. Run `python -m mkdocs build --strict --clean` from the repository root.
2. Check that every page in `mkdocs.yml` still builds and that no internal link
   warnings/errors remain.
3. Inspect the rendered homepage and one long API page at desktop and narrow
   mobile widths. Check code wrapping, tables, nav behavior, contrast, and
   image alt text.
4. Confirm the docs examples still match `src/main.cpp` and public headers.
5. Confirm the deploy workflow still triggers only from `main` and that the
   site URL remains `https://jonahchang207.github.io/odyssey/`.

At the time this context was written, the local Python environment did not have
MkDocs installed, so strict build verification was not run locally. CI or a
Python environment with `mkdocs-material` installed is required.

## Suggested prompt for the next model

> Improve the Odyssey GitHub Pages documentation in `docs/` and `mkdocs.yml`.
> Read `CONTEXT_README.md` first, then inspect the existing pages, SVG assets,
> `src/main.cpp`, and public headers. Make the site feel like a polished,
> approachable VEX robotics toolkit: prioritize homepage conversion,
> beginner onboarding, coordinate/architecture comprehension, consistent
> indigo/cyan Material visual identity, accessible responsive layout, and precise
> cross-linking. Preserve the technical truths and existing learning content.
> Implement the changes in the repository, run `python -m mkdocs build
> --strict --clean` after installing the documented dependency if needed, and
> report files changed plus any remaining verification limits.
