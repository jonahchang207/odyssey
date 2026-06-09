<#
Odyssey CLI - menu-driven version control for the Odyssey repository.

Run it from the repo root:
  .\odyssey              interactive menu
  .\odyssey commit       jump straight to one action:
                         status, commit, push, sync, branch, pr, merge,
                         release, setup

Workflow this tool enforces:
  - 'main' is the stable, protected branch. Nothing is committed to it
    directly; changes arrive via pull requests from 'dev'.
  - 'dev' is the everyday working branch.
  - Releases are tagged from 'main'.
#>
param(
    [string]$Action = ""
)

$ErrorActionPreference = "Continue"

# repo root = parent of the tools folder this script lives in
$RepoRoot = Split-Path -Parent $PSScriptRoot
Set-Location $RepoRoot

function Write-Title($text) {
    Write-Host ""
    Write-Host "== $text ==" -ForegroundColor Magenta
}
function Write-Ok($text) { Write-Host $text -ForegroundColor Green }
function Write-Warn($text) { Write-Host $text -ForegroundColor Yellow }

function Get-Branch { return (git rev-parse --abbrev-ref HEAD) }

function Test-Gh {
    $gh = Get-Command gh -ErrorAction SilentlyContinue
    if ($null -eq $gh) {
        Write-Warn "GitHub CLI (gh) not found. Install it with: winget install GitHub.cli"
        return $false
    }
    return $true
}

# ----------------------------------------------------------------------------
# actions
# ----------------------------------------------------------------------------

function Invoke-Status {
    Write-Title "Working tree"
    git status
    Write-Title "Recent history"
    git log --oneline --graph --decorate --all -15
}

function Invoke-Commit {
    $pending = git status --porcelain
    if (-not $pending) {
        Write-Warn "Nothing to commit - working tree is clean."
        return
    }

    if ((Get-Branch) -eq "main") {
        Write-Warn "You are on 'main', which is protected. Day-to-day work belongs on 'dev'."
        $switch = Read-Host "Switch to dev and bring these changes along? (Y/n)"
        if ($switch -ne "n") { git checkout dev }
    }

    Write-Title "Changes to commit"
    git status --short

    Write-Host ""
    Write-Host "Commit type:"
    Write-Host "  1) feat      - new feature"
    Write-Host "  2) fix       - bug fix"
    Write-Host "  3) docs      - documentation only"
    Write-Host "  4) tune      - PID / odometry tuning values"
    Write-Host "  5) refactor  - code change, same behavior"
    Write-Host "  6) chore     - build, cleanup, misc"
    Write-Host "  7) none      - no prefix"
    $t = Read-Host "Choose (1-7)"
    $prefix = switch ($t) {
        "1" { "feat: " }
        "2" { "fix: " }
        "3" { "docs: " }
        "4" { "tune: " }
        "5" { "refactor: " }
        "6" { "chore: " }
        default { "" }
    }

    $msg = Read-Host "Commit message"
    if (-not $msg) {
        Write-Warn "No message - canceled."
        return
    }

    git add -A
    git commit -m "$prefix$msg"
    if ($LASTEXITCODE -ne 0) { return }
    Write-Ok "Committed: $prefix$msg"

    $push = Read-Host "Push now? (Y/n)"
    if ($push -ne "n") { Invoke-Push }
}

function Invoke-Push {
    $branch = Get-Branch
    git push -u origin $branch
    if ($LASTEXITCODE -eq 0) { Write-Ok "Pushed '$branch' to GitHub." }
    else { Write-Warn "Push failed. If this repo isn't on GitHub yet, run: .\odyssey setup" }
}

function Invoke-Sync {
    $branch = Get-Branch
    Write-Title "Pulling latest '$branch'"
    git pull origin $branch
    if ($branch -ne "main") {
        $merge = Read-Host "Also merge the latest main into '$branch'? (y/N)"
        if ($merge -eq "y") {
            git fetch origin main
            git merge origin/main
        }
    }
}

function Invoke-Branch {
    $name = Read-Host "New branch name (e.g. feature/intake-auto)"
    if (-not $name) { return }
    git checkout -b $name dev
    if ($LASTEXITCODE -eq 0) {
        Write-Ok "Created '$name' from dev. Commit here, then merge it back into dev."
    }
}

function Invoke-PR {
    if (-not (Test-Gh)) { return }
    Write-Title "Opening pull request: dev -> main"
    git push -u origin dev
    $title = Read-Host "PR title (Enter = autofill from commits)"
    if ($title) {
        gh pr create --base main --head dev --title $title --body "Stabilization PR from dev."
    } else {
        gh pr create --base main --head dev --fill
    }
}

function Invoke-MergePR {
    if (-not (Test-Gh)) { return }
    Write-Title "Open pull requests into main"
    gh pr list --base main
    $num = Read-Host "PR number to merge (Enter = the PR for the current branch)"
    if ($num) { gh pr merge $num --merge }
    else { gh pr merge --merge }
    if ($LASTEXITCODE -ne 0) { return }
    Write-Ok "Merged into main."

    $sync = Read-Host "Update dev with the merged main? (Y/n)"
    if ($sync -ne "n") {
        git checkout dev
        git fetch origin main
        git merge origin/main
        git push origin dev
    }
}

function Invoke-Release {
    if (-not (Test-Gh)) { return }

    $last = git tag --list "v*" --sort=-v:refname | Select-Object -First 1
    if (-not $last) { $last = "v0.0.0" }
    $parts = $last.TrimStart("v").Split(".")
    $major = [int]$parts[0]
    $minor = [int]$parts[1]
    $patch = [int]$parts[2]

    Write-Title "New release (latest: $last)"
    Write-Host "  1) patch -> v$major.$minor.$($patch + 1)  (fixes / tuning)"
    Write-Host "  2) minor -> v$major.$($minor + 1).0  (new features)"
    Write-Host "  3) major -> v$($major + 1).0.0  (breaking changes)"
    Write-Host "  4) custom"
    $c = Read-Host "Choose (1-4)"
    $version = switch ($c) {
        "1" { "v$major.$minor.$($patch + 1)" }
        "2" { "v$major.$($minor + 1).0" }
        "3" { "v$($major + 1).0.0" }
        "4" { Read-Host "Version (vX.Y.Z)" }
        default { "" }
    }
    if (-not $version) {
        Write-Warn "Canceled."
        return
    }

    # releases always come from the stable branch
    Write-Host "Releases are cut from main. Switching..."
    git checkout main
    git pull origin main
    git tag $version
    git push origin $version
    gh release create $version --title $version --generate-notes
    if ($LASTEXITCODE -eq 0) { Write-Ok "Released $version" }
    git checkout dev
}

function Invoke-Setup {
    if (-not (Test-Gh)) { return }

    gh auth status
    if ($LASTEXITCODE -ne 0) {
        Write-Warn "Not logged in to GitHub. Run:  gh auth login   then re-run setup."
        return
    }

    # 1. create the GitHub repository if there is no remote yet.
    # Public is deliberate: GitHub Pages and branch rulesets are free on
    # public repos
    $remote = git remote
    if (-not $remote) {
        Write-Title "Creating GitHub repository 'odyssey' (public)"
        gh repo create odyssey --public --source . --remote origin
        if ($LASTEXITCODE -ne 0) {
            Write-Warn "Repository creation failed."
            return
        }
    }

    # 2. fill in the GitHub username placeholders in the docs config
    $login = gh api user --jq .login
    if ($login -and (Select-String -Path mkdocs.yml -Pattern "YOUR_GITHUB_USERNAME" -Quiet)) {
        Write-Title "Setting docs URLs for github.com/$login"
        git checkout main
        # use .NET IO so UTF-8 content survives Windows PowerShell 5.1
        $utf8 = New-Object System.Text.UTF8Encoding $false
        foreach ($f in @("mkdocs.yml", "README.md", "docs\getting-started\configuration.md")) {
            $path = Join-Path $RepoRoot $f
            $text = [System.IO.File]::ReadAllText($path)
            [System.IO.File]::WriteAllText($path, $text.Replace("YOUR_GITHUB_USERNAME", $login), $utf8)
        }
        git add mkdocs.yml README.md docs/getting-started/configuration.md
        git commit -m "chore: set GitHub username in docs URLs"
    }

    # 3. push both long-lived branches
    Write-Title "Pushing main and dev"
    git checkout main
    git push -u origin main
    git checkout dev
    git merge main
    git push -u origin dev

    # 4. protect main: no direct pushes, no force pushes, no deletion.
    # Changes land through pull requests (0 approvals required, so a solo
    # maintainer can still merge their own PRs)
    Write-Title "Protecting main"
    $ruleset = @'
{
  "name": "protect-main",
  "target": "branch",
  "enforcement": "active",
  "conditions": { "ref_name": { "include": ["refs/heads/main"], "exclude": [] } },
  "rules": [
    { "type": "deletion" },
    { "type": "non_fast_forward" },
    {
      "type": "pull_request",
      "parameters": {
        "required_approving_review_count": 0,
        "dismiss_stale_reviews_on_push": false,
        "require_code_owner_review": false,
        "require_last_push_approval": false,
        "required_review_thread_resolution": false
      }
    }
  ]
}
'@
    $tmp = Join-Path $env:TEMP "odyssey-ruleset.json"
    Set-Content -Path $tmp -Value $ruleset -Encoding Ascii
    gh api "repos/{owner}/{repo}/rulesets" -X POST --input $tmp | Out-Null
    if ($LASTEXITCODE -eq 0) { Write-Ok "Ruleset applied: main only accepts pull requests." }
    else { Write-Warn "Could not apply the ruleset (it may already exist). Check repo Settings -> Rules." }
    Remove-Item $tmp -ErrorAction SilentlyContinue

    Write-Ok "Setup complete. You are on 'dev' - work here."
    Write-Host ""
    Write-Host "GitHub Pages: the docs deploy automatically on the next push to main."
    Write-Host "After the first 'Deploy documentation' action finishes, enable Pages once:"
    Write-Host "  repo Settings -> Pages -> Source: Deploy from a branch -> gh-pages"
}

# ----------------------------------------------------------------------------
# dispatch
# ----------------------------------------------------------------------------

function Invoke-Action($name) {
    switch ($name) {
        "status" { Invoke-Status }
        "commit" { Invoke-Commit }
        "push" { Invoke-Push }
        "sync" { Invoke-Sync }
        "branch" { Invoke-Branch }
        "pr" { Invoke-PR }
        "merge" { Invoke-MergePR }
        "release" { Invoke-Release }
        "setup" { Invoke-Setup }
        default { Write-Warn "Unknown action '$name'." }
    }
}

if ($Action) {
    Invoke-Action $Action.ToLower()
    return
}

while ($true) {
    $branch = Get-Branch
    Write-Host ""
    Write-Host '  ___      _                          ' -ForegroundColor Magenta
    Write-Host ' / _ \  __| |_   _ ___ ___  ___ _   _ ' -ForegroundColor Magenta
    Write-Host '| | | |/ _` | | | / __/ __|/ _ \ | | |' -ForegroundColor Magenta
    Write-Host '| |_| | (_| | |_| \__ \__ \  __/ |_| |' -ForegroundColor Magenta
    Write-Host ' \___/ \__,_|\__, |___/___/\___|\__, |' -ForegroundColor Magenta
    Write-Host '             |___/              |___/ ' -ForegroundColor Magenta
    Write-Host ""
    Write-Host "current branch: " -NoNewline
    if ($branch -eq "main") { Write-Host $branch -ForegroundColor Red -NoNewline; Write-Host "  (protected - switch to dev to work)" }
    else { Write-Host $branch -ForegroundColor Green }
    Write-Host ""
    Write-Host "  1) Status and history"
    Write-Host "  2) Commit changes"
    Write-Host "  3) Push"
    Write-Host "  4) Sync (pull latest)"
    Write-Host "  5) New feature branch (from dev)"
    Write-Host "  6) Open PR  (dev -> main)"
    Write-Host "  7) Merge PR (into main)"
    Write-Host "  8) Release  (tag + GitHub release from main)"
    Write-Host "  9) First-time GitHub setup"
    Write-Host "  Q) Quit"
    $choice = Read-Host "Choose"

    switch ($choice.ToUpper()) {
        "1" { Invoke-Status }
        "2" { Invoke-Commit }
        "3" { Invoke-Push }
        "4" { Invoke-Sync }
        "5" { Invoke-Branch }
        "6" { Invoke-PR }
        "7" { Invoke-MergePR }
        "8" { Invoke-Release }
        "9" { Invoke-Setup }
        "Q" { return }
        default { }
    }
}
