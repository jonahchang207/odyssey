# CLAUDE.md

Guidance for Claude Code when working in this repository.

## What this is

**Odyssey** — an odometry + motion control library for VEX V5 robots, built
on PROS (kernel 4.2.2). Inspired by LemLib and EZ-Template. Distributed as a
PROS template (compiled `odyssey.a` + headers) via GitHub releases and a
depot branch.

- Docs site: https://jonahchang207.github.io/odyssey/ (MkDocs Material, in `docs/`)
- Library source: `include/odyssey/` (headers) + `src/odyssey/` (implementation)
- `src/main.cpp` is the example robot config — excluded from the library archive
- `include/pros/`, `firmware/`, `common.mk` are the PROS kernel — do not edit

## Commit rules (important)

- Author every commit as **Jonah Chang <jonahchang207@gmail.com>** (set in
  repo-local git config). Never add `Co-Authored-By: Claude` trailers or
  credit Claude anywhere.
- Work on **`dev`**. `main` is protected (ruleset `protect-main`): changes
  land only via pull request. Use `gh pr create --base main --head dev` then
  `gh pr merge --merge`, and merge `main` back into `dev` afterwards.
- Commit prefixes: `feat:`, `fix:`, `docs:`, `tune:` (PID/odometry values),
  `refactor:`, `chore:`.
- `tools/odyssey.ps1` (run as `.\odyssey`) is the team's menu CLI wrapping
  this workflow — keep it working when changing the workflow.

## Building and testing

- **No ARM toolchain on this machine** — the C++ cannot be compiled locally.
  CI (`.github/workflows/build-template.yml`) compiles every push/PR with
  ARM GCC 14.2 (kernel 4.2 needs `gnu++26`, so GCC >= 14). To verify a code
  change compiles: push to `dev`, then
  `gh run watch <id> --exit-status` on the "Build PROS template" run.
- **Docs**: `python -m mkdocs build --strict` (mkdocs-material is installed).
  Always run this before committing docs changes; strict mode catches broken
  links. Docs deploy to the `gh-pages` branch automatically on push to main.
- **Releases**: `gh release create vX.Y.Z --target main` (or `.\odyssey release`).
  CI then builds `odyssey@X.Y.Z.zip` from the tag (the tag version is sed-ed
  into the Makefile), attaches it to the release, and force-pushes
  `depot/stable.json` so `pros c add-depot` users get the new version.
  Keep `VERSION:=` in the Makefile roughly in sync for local sanity.

## Code conventions

- Units: **inches** and, user-facing, **compass degrees** (0 = +y, clockwise
  positive). Internally odometry stores standard math radians (0 = +x, CCW
  positive); conversion is `theta_compass = 90 - deg(theta_math)`.
- Sign conventions (documented in `docs/explanation/odometry-math.md`):
  vertical tracking wheel offset negative = left of center; horizontal
  offset negative = behind center. Turn functions work in the compass frame
  (positive output = clockwise = left side forward); drive motions work in
  the math frame (positive angular = CCW = right side faster). Keep any new
  motion internally consistent and say which frame it uses.
- One motion at a time, enforced by `motionMutex` +
  `requestMotionStart`/`endMotion`; async motions re-invoke themselves in a
  `pros::Task` with `[=, this]` capture (never `[&]`). `distTraveled` is -1
  when idle — `waitUntil`/`waitUntilDone` depend on that.
- PROS 4 API only (`pros::MotorGroup`, `pros::v5::MotorBrake`, enum classes —
  the C-style enums often don't convert implicitly).
- Doxygen-style comments on public APIs; docs pages in `docs/` must be
  updated alongside API changes (mkdocs.yml `nav` lists every page).

## Environment gotchas

- Windows PowerShell 5.1: `Get-Content -Raw` + `Set-Content` corrupts UTF-8
  (em-dashes, emoji). Use `[System.IO.File]::ReadAllText/WriteAllText` with
  `UTF8Encoding($false)` instead.
- `pros.exe` is pip-installed but not on PATH:
  `$env:LOCALAPPDATA\Packages\PythonSoftwareFoundation.Python.3.12_qbz5n2kfra8p0\LocalCache\local-packages\Python312\Scripts\pros.exe`
- The git history was deliberately rewritten once (2026-06-09) to fix
  authorship — do not "restore" old refs/original backups.
