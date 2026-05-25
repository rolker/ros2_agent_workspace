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

**Key on the worktree/issue, not the global PR number.** PR numbers are
per-repo, so a bare `PR=<N>` is ambiguous across ~30 project repos — and in a
mature workspace most PRs are *project* PRs. The workspace is already
issue-keyed (`feature/issue-<N>`, `issue-<slug>-<N>`, `worktree_remove.sh
--issue/--repo-slug`); merge_pr keys the same way. The PR number is *derived*
from (repo, branch), never an ambiguous input.

1. **Add `.agent/scripts/merge_pr.sh`** with resolution ordered most-natural first:
   - **cwd (zero-arg, dominant case)** — `make merge-pr` from inside a worktree:
     repo = `git -C $PWD remote get-url origin`, branch = `git branch
     --show-current`, PR = `gh pr view "$branch" -R <slug>`. The worktree you're
     standing in *is* the answer — no number, no slug, no ambiguity. (You're
     normally here right after review-code.)
   - **`ISSUE=<N> [REPO=<slug>]`** — look the worktree up by issue number via
     `worktree_list.sh` (`issue-<slug>-<N>` / `issue-workspace-<N>`); `REPO=`
     disambiguates the rare cross-repo issue-number collision. Mirrors
     `worktree_remove.sh`'s interface (which merge_pr calls anyway).
   - **`PR=<N> REPO=<slug>`** — explicit escape hatch for headless/no-worktree
     callers (e.g. the #481-D orchestrator, or a main-tree invocation).
   - Keep downstream's hardened root resolution (`git worktree list --porcelain
     | head -1`, their #146) and `-R <slug>` on every `gh` call.
2. **Field-mode guard FIRST** — resolve the repo *locally* (its origin host),
   and if `is_field_mode <repo>` (source `field_mode.sh`), error **before any
   `gh` call**: field-mode repos have no GitHub PR (they push without PRs) — use
   the field workflow, not `merge-pr`. Ordering matters: a `gh pr view` against
   a gitcloud origin fails with a confusing "not found" before the clear guard
   message. (ADR-0011.)
3. **Wait for CI** — `gh pr checks <branch|N> -R <repo> --watch --fail-fast`;
   `--no-wait` to skip when CI is known green (their #186 race fix).
4. **Merge** — `gh pr merge <N> -R <repo> --merge` (workspace merge-commit convention).
5. **Remove worktree** — `cd "$ROOT_DIR"` in the script's own shell first
   (`worktree_remove.sh` refuses if `$PWD` is inside the target; the cwd is the
   worktree in the dominant case), then `worktree_remove.sh --issue <N>
   [--repo-slug <slug>]`.
6. **Delete branches** — `merge_pr.sh` owns this: `worktree_remove.sh` does NOT
   delete layer branches (it only prunes refs + prints a note). Delete local +
   remote `feature/issue-<N>` on the **owning repo** — the workspace repo, or
   the project repo under `layers/main/.../src/<slug>` (`git -C <repo> branch -d`
   + `push origin --delete`).
7. **Sync** — `cd "$ROOT_DIR"` (cwd may be the just-deleted worktree), then
   `make sync` (`sync_repos.py`, all repos + git-bug) — replaces downstream's
   hand-rolled two-repo `git pull`.
8. **Don't carry over** downstream's roadmap-update step or its `source
   _issue_helpers.sh` / `update_roadmap.sh` calls — neither exists here (we track
   state in `progress.md`/`plan.md`). Inline only what's needed.
9. **Makefile target `merge-pr`** — accepts optional `ISSUE=` / `REPO=` / `PR=`
   (no required arg; bare invocation uses cwd). Add to `.PHONY` + the help block.
10. **`generate_make_skills.sh`** — the default generator emits a bare
    `make <target>` with no argument substitution, so `/make_merge-pr` would run
    argless from the *main tree* (no worktree context) and fail. Add `merge-pr`
    to the generator's parameterized case so `/make_merge-pr <N>` → `make
    merge-pr ISSUE=<N>`, **preserving `disable-model-invocation: true`**
    (human-only; a merge+branch-delete shouldn't be model-auto-invocable). Update
    `test_generate_make_skills.sh`. Then run `make generate-skills`.
11. **Add `.agent/scripts/tests/test_merge_pr.sh`** (shell test, mirrors
    `test_field_mode.sh` / downstream's `test_merge_pr_root_resolution.sh`):
    root resolution from inside a worktree; cwd resolution; `ISSUE=` worktree
    lookup; field-mode-refusal path. Mock `gh`/`git` where needed; the CI-wait +
    real merge stay untested (mocking `gh pr checks --watch` is drift-prone —
    downstream's pragmatic call).

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/merge_pr.sh` | New — adapted merge+cleanup+sync, worktree/issue-keyed for the layered model |
| `.agent/scripts/tests/test_merge_pr.sh` | New — cwd/`ISSUE=` resolution + field-mode-guard + root-resolution tests |
| `.agent/scripts/generate_make_skills.sh` | Add `merge-pr` to the parameterized case (`ISSUE=$ARGUMENTS`, keep `disable-model-invocation`) |
| `.agent/scripts/tests/test_generate_make_skills.sh` | Cover the new `merge-pr` parameterized case |
| `Makefile` | Add `merge-pr` target (optional `ISSUE=`/`REPO=`/`PR=`) + `.PHONY` entry + help line |
| `.claude/skills/make_*` (generated) | Re-run `make generate-skills` → adds `/make_merge-pr <N>` |
| `AGENTS.md` | Add `merge_pr.sh` to the Script Reference table |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| A change includes its consequences | Makefile target + `generate-skills` + AGENTS.md table all in the plan. |
| Test what breaks | Regression test for the resolution + field-mode logic (the bug-prone parts), per downstream's hard-won fixes. |
| Only what's needed | Reuse `worktree_remove.sh` + `make sync`; don't reimplement. Adapt, don't gold-plate. |
| Human control & transparency | Invoked deliberately after review; CI-wait gate; does **not** auto-decide `Closes` vs umbrella semantics. The `/make_merge-pr` slash command keeps `disable-model-invocation: true` (human-only) — a merge+branch-delete is not model-auto-invocable. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0011 — field mode | Yes | Field-mode repos have no GitHub PR; step 2 detects via `is_field_mode` and refuses **before any `gh` call**, so the clear message wins over a confusing gh "not found". |
| 0002 — worktree isolation | Yes | `cd` to main tree before `worktree_remove`; never operate inside the worktree being removed. |
| 0007 — retain Make | Yes | New `.PHONY` target follows Makefile conventions (PR=/REPO= validation like `push-remote`). |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Add a script (`merge_pr.sh`) | AGENTS.md Script Reference table | Yes (step + files table) |
| Add a `.PHONY` Makefile target | `generate_make_skills.sh` parameterized case + its test; `make generate-skills` → `/make_merge-pr <N>` | Yes (steps 9–10) |
| Worktree-adjacent tooling | Brief mention in WORKTREE_GUIDE.md (optional) | No — follow-up if useful |

## Resolved Decisions

- **Resolution keyed on worktree/issue, not the global PR number** (decided
  2026-05-25). cwd (zero-arg) → `ISSUE=`/`REPO=` → `PR=` escape hatch. Rationale:
  in a mature workspace most PRs are project PRs, so a PR-number-keyed interface
  is both ambiguous (per-repo numbering across ~30 repos) and verbose in the
  common case; the worktree you're standing in disambiguates with zero input.
  Dissolves the PR#-vs-issue# conflation the plan review flagged.
- **`merge-pr` stays mechanical — no `progress.md` entry** (decided 2026-05-25).
  Lifecycle entries (`## Implementation`, etc.) are owned by the skills / #481-C
  "address findings"; coupling a mechanical merge tool to ADR-0013 vocabulary
  isn't warranted.

## Estimated Scope

Single PR. One new bash script + its shell test, a small `generate_make_skills.sh`
change (+ its test), a Makefile target, and a one-line AGENTS.md table addition.
Independent of phase C (#481) — it doesn't depend on the dispatch work and can
land standalone.
