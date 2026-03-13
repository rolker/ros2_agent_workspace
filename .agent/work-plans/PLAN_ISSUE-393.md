# Plan: Integrate git-bug: bootstrap, sync, agent workflows, and project onboarding

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/393

## Context

The workspace evaluated git-bug in #353 and confirmed it works well for
bidirectional GitHub issue sync. Currently all agent issue lookups go through
`gh` API calls, which require network access and add latency. git-bug stores
issues as git objects under `refs/bugs/*`, enabling offline-capable, fast local
queries. The parent issue #385 identifies git-bug integration as an
infrastructure component of local-first agent orchestration.

Key constraints from #353:
- Identity is repo-global (one identity per repo, stored in `.git/config`)
- Refs aren't included in `git clone` — explicit `git bug pull` needed
- Bridge auth uses GitHub token (can source from `gh auth token`)

## Approach

### 1. Write ADR-0010: Adopt git-bug for local issue tracking

Record the decision to adopt git-bug. Content draws from #353 evaluation.
Covers: why git-bug, single-identity strategy, graceful degradation approach,
sync frequency (worktree entry, `make sync`, dashboard refresh).

### 2. Add git-bug binary install to `bootstrap.sh`

Add a section after the rosdep block (~line 147) that:
- Checks if `git-bug` is already installed at the pinned version (v0.10.1)
- Downloads the correct architecture binary from GitHub releases
- Installs to `/usr/local/bin/git-bug`
- Follows the existing `DRY_RUN` / `PENDING_COMMANDS` pattern

### 3. Create `git_bug_setup.sh` for bridge configuration

New script that handles post-install setup (separate from bootstrap because
it needs repo context and a GitHub token):
- Create git-bug user identity if none exists (human user's name/email)
- Adopt the identity
- Configure GitHub bridge for the current repo using `gh auth token`
- Run initial `git bug pull`
- Idempotent — safe to re-run

### 4. Add Makefile stamp `$(STAMP)/git-bug.done`

New stamp depending on `bootstrap.done`:
- Calls `git_bug_setup.sh` for the workspace repo
- Added as a dependency of `setup-all` (not `build` — git-bug is optional
  infrastructure, not a build prerequisite)
- Add `skip-git-bug` target for environments where git-bug isn't wanted

### 5. Add git-bug sync to `sync_repos.py`

Add a `sync_gitbug()` function that:
- Checks if `git-bug` is installed and a bridge is configured for the repo
- Runs `git bug pull` then `git bug push`
- Called after each `sync_repo()` for repos with git-bug configured
- Respects `--dry-run`

### 6. Update `worktree_enter.sh` — prefer local issue lookup

Before the existing `gh issue view` block (~line 234):
- Try `git bug show` first for issue title (using workspace root repo)
- Also trigger a background `git bug pull` to keep data fresh
- Fall back to `gh` if git-bug is not configured or returns nothing
- No change to the exported `WORKTREE_ISSUE_TITLE` variable

### 7. Update `worktree_create.sh` — local issue status check

Before the existing `gh issue view` block (~line 578):
- Try `git bug show` for issue title and state
- Normalize state values (git-bug uses lowercase, existing code expects uppercase)
- Fall back to `gh` for the issue check; PR check stays `gh`-only

### 8. Update `dashboard.sh` — local issue counts and git-bug health

Two changes:
- Health checks section (~line 128): add git-bug presence check (optional tool,
  warn if missing, don't fail)
- Issue counts section (~line 493): prefer `git bug ls --status open` over
  `gh api search/issues` when git-bug is configured

### 9. Update `gh_create_issue.sh` — optional local creation

Add an environment variable `GITBUG_CREATE=1` opt-in:
- When set and git-bug is available, create via `git bug new` + `git bug push`
  instead of `gh issue create`
- Extract `--title`, `--body`/`--body-file`, `--label` from existing arg parsing
- Default behavior (no env var) is unchanged — pure `gh` path

### 10. Update `onboard-project` skill — offer git-bug bridge setup

Add a new audit check to the onboard-project skill:
- Check: is git-bug installed? Is a bridge configured for this project repo?
- Fix: offer to run `git bug bridge configure` + initial pull
- Opt-in per repo — allows gradual rollout

### 11. Update AGENTS.md script reference table

Add `git_bug_setup.sh` to the script reference table. Update any script
descriptions that change behavior (e.g., `sync_repos.py` now includes git-bug).

## Files to Change

| File | Change |
|------|--------|
| `docs/decisions/0010-adopt-git-bug-for-local-issue-tracking.md` | New ADR |
| `.agent/scripts/bootstrap.sh` | Add git-bug binary install (~line 147) |
| `.agent/scripts/git_bug_setup.sh` | New: bridge config, identity, initial pull |
| `Makefile` | Add `git-bug.done` stamp + `skip-git-bug` target |
| `.agent/scripts/sync_repos.py` | Add `sync_gitbug()` function |
| `.agent/scripts/worktree_enter.sh` | Prefer git-bug for issue title lookup |
| `.agent/scripts/worktree_create.sh` | Prefer git-bug for issue status check |
| `.agent/scripts/dashboard.sh` | Add git-bug health check + local issue counts |
| `.agent/scripts/gh_create_issue.sh` | Optional `GITBUG_CREATE` path |
| `.claude/skills/onboard-project/SKILL.md` | Add git-bug bridge audit check |
| `AGENTS.md` | Add `git_bug_setup.sh` to script reference table |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Enforcement over documentation | git-bug is tooling, not a rule. Graceful degradation means no enforcement needed — scripts work with or without it |
| Capture decisions | ADR-0010 records the adoption decision and design choices |
| A change includes its consequences | AGENTS.md script table, onboard-project skill, and all affected scripts updated in the same PR |
| Only what's needed | Each script change is minimal (try git-bug first, fall back to gh). No speculative features |
| Improve incrementally | Project repos opt in via onboarding — no big-bang rollout |
| Workspace vs. project separation | git-bug infra is generic; project bridge setup is opt-in via onboarding skill |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0001 — Adopt ADRs | Yes | ADR-0010 written as step 1 |
| 0003 — Project-agnostic | Yes | git-bug is generic issue tooling, not project-specific |
| 0004 — Enforcement hierarchy | No | git-bug is optional tooling, not a compliance rule |
| 0007 — Retain Make | Yes | New stamp follows existing pattern; depends on `bootstrap.done` |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Scripts in `.agent/scripts/` | Script reference table in `AGENTS.md` | Yes (step 11) |
| `Makefile` targets | AGENTS.md build section if new targets are user-facing | Yes (step 4 adds to setup-all, not a new user target) |
| `onboard-project` skill | Skill list in non-Claude adapters (if interface changes) | No — interface unchanged, just new audit check |
| Worktree scripts | `.agent/WORKTREE_GUIDE.md` | No — behavior is transparent (same outputs, faster path) |

## Open Questions

None — all decisions resolved in the review follow-up discussion.

## Estimated Scope

Single PR. ~11 files changed, but most changes are small (5-20 line insertions
with graceful degradation pattern). ADR and new setup script are the largest
new content.
