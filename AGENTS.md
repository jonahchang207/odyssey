# Repository instructions for agents

These instructions apply to the entire Odyssey repository.

## Ownership and attribution

- All commits must be authored as `Jonah Chang <jonahchang207@gmail.com>`.
- Use the repository-local Git configuration for that identity.
- Do not add AI attribution, `Co-Authored-By` trailers, or agent names to
  commits, pull requests, source files, or documentation.

## Branch workflow

- Do normal work on `dev`. Never commit directly to `main`.
- Push completed work to `origin/dev` so GitHub Actions can test it.
- Documentation pushed to `dev` is published as an unreleased preview at
  `https://jonahchang207.github.io/odyssey/dev/`; verify that URL before
  promoting documentation to `main`.
- Use optional `feature/*` branches for larger experiments, then merge them
  into `dev` before stabilization.
- Treat `main` as the stable, competition-ready branch. Do not force-push or
  delete `main` or `dev`.

## Commits and pushes

- Review `git status`, `git diff`, and the staged diff before committing.
- Preserve unrelated user changes and secrets. Do not commit generated build
  output such as `site/`, local logs, caches, or editor files.
- Use a concise imperative subject with one of these prefixes:
  `feat:`, `fix:`, `docs:`, `tune:`, `refactor:`, or `chore:`.
- Keep each commit internally coherent and include required tests and docs in
  the same commit.
- Before pushing, run the checks appropriate to the changed files. For docs,
  run `python -m mkdocs build --strict --clean`. C++ changes require the
  relevant host tests when available and the GitHub Actions ARM build.
- Push the current working branch normally. Never use `--force` unless the
  user explicitly requests it and the target has been verified safe.

## Merging `dev` into `main`

- Merge only after the change has been tested and is ready to be considered
  stable. Documentation changes require a strict MkDocs build and rendered
  desktop/mobile checks of the developer preview; C++ changes require green
  GitHub Actions, including the PROS template build.
- Confirm `dev` is pushed and up to date, the worktree is clean, and all
  required checks pass before opening the pull request.
- Open a pull request from `dev` into `main`; never bypass the protected branch
  with a direct push.
- Review the pull-request diff and check results. Merge only when they are green
  and the user has authorized shipping the tested change to `main`.
- Use a normal merge commit unless the user specifies another strategy. Never
  force-merge or bypass required checks.
- After merging, update local `main`, merge `main` back into `dev`, and push
  `dev` so the branches do not drift.

## Protected project areas

- Do not edit `include/pros/`, `firmware/`, or `common.mk` unless the user
  explicitly requests a PROS kernel change.
- Public API changes require matching Doxygen comments and documentation under
  `docs/`.
