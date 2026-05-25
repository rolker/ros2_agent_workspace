# Plan: Add make merge-pr — merge + clean up worktree/branches + make sync

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/488

## Context

The lifecycle ends at review; merge + cleanup + sync are manual, and the sync
step gets forgotten (local `main` drifted 61 commits behind in one session).
`rolker/agent_workspace` solves this with a hardened `.agent/scripts/merge_pr.sh`
(+ a root-resolution regression test) run via `make merge-pr PR=<N>`. It can't be
ported verbatim: downstream is a 2-repo model (`workspace` + one `project/`);
this workspace is a multi-repo **layered** model (`layers/main/*_ws/src/*`, many
independent repos). Adapt it.

## Approach

1. **Add `.agent/scripts/merge_pr.sh`** (`--pr <N>` [`--repo-slug <slug>`]
   [`--no-wait`]), keeping downstream's hardened bits:
   - Root resolution via `git worktree list --porcelain | head -1` (works when
     invoked from inside a worktree — their #146).
   - Collision-safe PR-owner resolution: query OPEN state; error on ambiguity.
   - `-R <owner/repo>` on every `gh` call so it targets the resolved repo, not cwd.
2. **Adapt PR-owner resolution for the layered model.** Query the **workspace
   repo first** (`gh pr view <N> -R <ws-origin>`). If not found there, narrow to
   candidate project repos *locally* via `worktree_list.sh | grep -E
   "issue-<slug>-<N>($|[^0-9])"` (cheap; avoids gh-scanning every layer repo),
   then `gh pr view` to confirm. `--repo-slug` overrides/disambiguates. Extract
   the issue number from the PR head branch (`feature/issue-<N>`).
3. **Field-mode guard** — after resolving the repo, if `is_field_mode <repo>`
   (source `field_mode.sh`), error: field-mode repos have no GitHub PR (they
   push without PRs); they use the field workflow, not `merge-pr`. (ADR-0011.)
4. **Wait for CI** — `gh pr checks <N> -R <repo> --watch --fail-fast`; `--no-wait`
   to skip when CI is known green (their #186 race fix).
5. **Merge** — `gh pr merge <N> -R <repo> --merge` (workspace merge-commit convention).
6. **Remove worktree** — `cd` to root first, then
   `worktree_remove.sh --issue <issueN> [--repo-slug <slug>]` (note: this
   workspace's remover takes `--issue`/`--repo-slug`, **not** downstream's `--type`).
7. **Delete branches** — local + remote `feature/issue-<N>` on the owning repo
   (workspace repo, or the project repo under `layers/main/.../src/<slug>`).
8. **Sync** — `make sync` (`sync_repos.py`, all repos + git-bug), replacing
   downstream's hand-rolled `git pull` on two repos.
9. **Drop** the downstream roadmap-update step (downstream-specific; we use
   progress.md/plan.md).
10. **Add the Makefile target** `merge-pr` (PR= validation + optional REPO=,
    mirroring `push-remote`), add to `.PHONY` + the help block, then run
    `make generate-skills` (regenerates the `/make_merge-pr` slash command).
11. **Add `.agent/scripts/tests/test_merge_pr.sh`** (mirrors downstream's
    `test_merge_pr_root_resolution.sh`): root resolution from inside a worktree;
    owner-resolution dispatch (workspace vs layer); field-mode-refusal path.
    Mock `gh`/`git` where needed; the CI-wait + real merge stay untested
    (downstream's pragmatic call — mocking `gh pr checks --watch` is drift-prone).

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/merge_pr.sh` | New — adapted merge+cleanup+sync for the layered model |
| `.agent/scripts/tests/test_merge_pr.sh` | New — resolution + field-mode-guard tests |
| `Makefile` | Add `merge-pr` target + `.PHONY` entry + help line |
| `.claude/skills/make_*` (generated) | Re-run `make generate-skills` → adds `/make_merge-pr` |
| `AGENTS.md` | Add `merge_pr.sh` to the Script Reference table |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| A change includes its consequences | Makefile target + `generate-skills` + AGENTS.md table all in the plan. |
| Test what breaks | Regression test for the resolution + field-mode logic (the bug-prone parts), per downstream's hard-won fixes. |
| Only what's needed | Reuse `worktree_remove.sh` + `make sync`; don't reimplement. Adapt, don't gold-plate. |
| Human control & transparency | Invoked deliberately after review; CI-wait gate; does **not** auto-decide `Closes` vs umbrella semantics. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0011 — field mode | Yes | Field-mode repos have no GitHub PR; step 3 detects via `is_field_mode` and refuses with a clear message. |
| 0002 — worktree isolation | Yes | `cd` to main tree before `worktree_remove`; never operate inside the worktree being removed. |
| 0007 — retain Make | Yes | New `.PHONY` target follows Makefile conventions (PR=/REPO= validation like `push-remote`). |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Add a script (`merge_pr.sh`) | AGENTS.md Script Reference table | Yes (step + files table) |
| Add a `.PHONY` Makefile target | `make generate-skills` → `/make_merge-pr` slash command | Yes (step 10) |
| Worktree-adjacent tooling | Brief mention in WORKTREE_GUIDE.md (optional) | No — follow-up if useful |

## Open Questions

- **Project-repo PR resolution UX**: default to workspace, then worktree-narrowed
  scan of layer repos, with `--repo-slug`/`REPO=` override (proposed). Acceptable,
  or require `REPO=` explicitly for any non-workspace PR (simpler, less magic)?
- **Should `merge-pr` write a terminal `progress.md` entry?** Kept out of scope
  here (mechanical tool); the #481-C "address findings" / `## Implementation`
  marker owns lifecycle entries. Confirm that boundary.

## Estimated Scope

Single PR. One new bash script + its shell test, a Makefile target (+ regenerated
slash command), and a one-line AGENTS.md table addition. Independent of phase C
(#481) — it doesn't depend on the dispatch work and can land standalone.
