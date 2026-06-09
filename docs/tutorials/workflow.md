# Team Workflow & the Odyssey CLI

The repository uses a two-branch model, and ships with a menu-driven CLI so
nobody on the team needs to memorize git commands.

## The branch model

```text
feature/* ──► dev ──► (pull request) ──► main ──► (tag) ──► release
```

- **`main`** — stable. Protected on GitHub: no direct pushes, no force
  pushes, no deletion. Code only lands here through a pull request from
  `dev`. If it's on `main`, it should be safe to load onto the robot at a
  competition.
- **`dev`** — the everyday working branch. Commit here.
- **`feature/*`** — optional short-lived branches off `dev` for bigger
  experiments (a new motion, an odometry change), merged back into `dev`.

## The Odyssey CLI

From the repo root, run:

```powershell
.\odyssey
```

```text
current branch: dev

  1) Status and history
  2) Commit changes
  3) Push
  4) Sync (pull latest)
  5) New feature branch (from dev)
  6) Open PR  (dev -> main)
  7) Merge PR (into main)
  8) Release  (tag + GitHub release from main)
  9) First-time GitHub setup
  Q) Quit
```

You can also jump straight to an action: `.\odyssey commit`,
`.\odyssey release`, etc.

### Committing (option 2)

Shows your changes, asks for a commit type (`feat`, `fix`, `docs`, `tune`,
`refactor`, `chore`), a message, then commits and offers to push. If you're
accidentally on `main`, it offers to move you (and your changes) to `dev`
first.

The `tune:` type is robotics-specific on purpose — PID and offset changes
are frequent and worth being able to find quickly in history:

```text
tune: increase angular kD to 12 after carpet change
```

### Stabilizing (options 6 + 7)

When `dev` is tested and working, option 6 opens a pull request into
`main`, and option 7 merges it. After merging, the CLI offers to sync
`dev` with the new `main` so the branches don't drift.

### Releasing (option 8)

Cuts a release from `main`:

1. Pick a version bump — patch (fixes/tuning), minor (new features), major
   (breaking changes) — following [semantic versioning](https://semver.org/).
2. The CLI tags `main`, pushes the tag, and creates a GitHub Release with
   auto-generated notes from the merged PRs.

Tag a release before every competition. If something breaks in the pits,
`git checkout v1.2.0` gets you back to exactly the code that worked.

### First-time setup (option 9)

One-time bootstrap for a new clone of this template:

1. Checks you're logged in to the GitHub CLI (`gh auth login` if not).
2. Creates the GitHub repository (public — GitHub Pages and branch
   rulesets are free on public repos) and pushes `main` and `dev`.
3. Fills your GitHub username into the docs URLs.
4. Applies the `protect-main` ruleset: pull requests required, force
   pushes and deletion blocked. Zero approvals are required so a solo
   maintainer can still merge their own PRs — raise it in repo Settings →
   Rules once your team grows.

## Day-to-day cheat sheet

| You want to... | Do |
| -------------- | --- |
| save your work | `.\odyssey commit` (on `dev`) |
| get a teammate's work | `.\odyssey sync` |
| try something risky | `.\odyssey branch` |
| ship tested code to stable | `.\odyssey pr`, then `merge` |
| freeze code for a competition | `.\odyssey release` |
