# Plan: Add behind-check to worktree_enter.sh

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/310

## Context

When agents enter a worktree, they get no warning if their feature branch is
behind the default branch. They discover this only at commit time (pre-commit
hook) or when the PR has merge conflicts. `check_branch_updates.sh` already
exists and handles upstream fetching, shallow clones, and fetch failures
gracefully — it just needs to be called from `worktree_enter.sh`.

**Key constraint**: for layer worktrees, the git context at the worktree root
is the *workspace* repo, but the feature branch lives in a *project* repo
inside `<layer>_ws/src/<project_repo>/`. The behind-check must run in the
project repo's directory. The existing `wt_layer_branch` helper already
iterates project repos to find branches — the same pattern locates the
correct directory.

Owner [requested](https://github.com/rolker/ros2_agent_workspace/issues/310#issuecomment-3973387938)
that the check fetches upstream changes. `check_branch_updates.sh` already
does this (lines 61-69: `git fetch origin "$DEFAULT_BRANCH"`).

## Approach

1. **Add behind-check call to `worktree_enter.sh`** — insert after the
   branch/PWD display (line 291) and before the "Helpful commands" block
   (line 295). For workspace worktrees, run `check_branch_updates.sh`
   directly. For layer worktrees, find the first non-symlinked project
   repo directory (same pattern as `wt_layer_branch`) and run
   `check_branch_updates.sh` from there via a subshell `(cd "$pkg_dir" && ...)`.

2. **Handle execution from a sourced script** — `worktree_enter.sh` is
   sourced, but `check_branch_updates.sh` must be executed (has a
   source-guard). Call it in a subshell to avoid polluting the current
   shell and to isolate its `set -e`.

3. **Suppress errors gracefully** — if the check fails for any reason
   (no network, missing remote), it should not break worktree entry.
   The script already exits 0 on fetch failures, but wrap the call with
   `|| true` as a safety net.

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/worktree_enter.sh` | Add ~10 lines: determine the correct git directory, run `check_branch_updates.sh` in a subshell |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Agents see staleness immediately on entry — more transparent |
| Enforcement over documentation | Informational only (no blocking). Appropriate for this stage. |
| A change includes its consequences | Script reference table already lists both scripts. No doc update needed — behavior is additive and non-breaking. |
| Only what's needed | Minimal change: no new flags, no changes to `check_branch_updates.sh` |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0002 — Worktree isolation | Yes | Enhances worktree workflow. Works for both workspace and layer worktrees by detecting the correct git context. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `.agent/scripts/worktree_enter.sh` | Script reference in `AGENTS.md` | Not needed — entry already listed, behavior is additive |
| `.agent/scripts/worktree_enter.sh` | `.agent/WORKTREE_GUIDE.md` | Not needed — guide doesn't document entry output details |

## Open Questions

None — scope is clear and owner feedback has been addressed by existing
script behavior.

## Estimated Scope

Single PR, single commit.
