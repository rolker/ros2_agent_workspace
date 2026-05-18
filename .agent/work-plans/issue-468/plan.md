# Plan: Sub-agent commits drop agent git identity

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/468

## Context

Sub-agent commits in this workspace can land with the human user's git
identity (`Roland Arsenault <roland@ccom.unh.edu>`) instead of the agent
identity (`Claude Code Agent <roland+claude-code@ccom.unh.edu>`).
Worked example: 4 of 6 commits on PR #464's branch before rebase.

Root cause: Bash tool calls don't persist shell state between
invocations. Sourcing `set_git_identity_env.sh` in one call exports
env vars that the next subshell doesn't see, so `git commit` falls back
to `.git/config`'s `user.email` (the human pattern).

The existing `.agent/hooks/check-commit-identity.py` pre-commit hook
*does* run on every commit, but its `ACCEPTED_PATTERNS` includes the
human emails alongside agent emails — so it passes the wrong-author
commits silently. Surfaced by the review-issue comment on #468.

User chose to bundle three mitigations into one PR (acceptance criteria
A + B + C below). Belt-and-suspenders from the start.

## Approach

1. **Mechanism A — tighten `.agent/hooks/check-commit-identity.py`
   to be branch-and-env aware.** When `$AGENT_NAME` is set AND the
   current branch matches the agent-branch convention (defined in
   shared module; see step 1a), require an agent-pattern email
   (`roland+*@ccom.unh.edu`) and reject the human patterns. Otherwise
   preserve current permissive behavior so the human can edit
   interactively and field-mode hotfixes still work. Branch detection
   uses `git symbolic-ref --short HEAD` (fail-safe to permissive on
   detached HEAD).

   **Honest acknowledgement**: Mechanism A is bypassed by the same
   root cause #468 describes — an unset `$AGENT_NAME` in a subshell
   silently triggers permissive mode. The hook helps when agents
   *do* set the env var; it can't defend against agents that don't.
   Mechanism C is therefore **load-bearing**, not optional
   belt-and-suspenders — it's the only layer immune to the
   subshell-env-var failure mode.

1a. **Extract shared identity patterns and agent-branch regex to a new
    module** `.agent/hooks/identity_patterns.py`. Both Mechanism A's
    hook and Mechanism C's CI script (and the existing
    `verify-issue-branch.py`) consume it. Contents:
    - `ACCEPTED_AGENT_PATTERNS = ["roland+*@ccom.unh.edu"]`
    - `HUMAN_PATTERNS = ["roland@ccom.unh.edu", "roland@rolker.net"]`
    - `PERMISSIVE_ACCEPTED_PATTERNS = ACCEPTED_AGENT_PATTERNS + HUMAN_PATTERNS`
    - Agent-branch regex reused from `verify-issue-branch.py:27`:
      `^feature/[iI][sS][sS][uU][eE]-(\d+)`, plus
      `^skill/[^/]+-\d{8}-\d{6}-\d{9}$` for the skill-worktree
      convention from AGENTS.md.
    - `is_agent_branch(name: str) -> bool` helper.
    Single source of truth eliminates DRY drift between A and C.
    `verify-issue-branch.py` refactored to import from this module
    on the same PR to ensure the regex stays consistent.

2. **Mechanism A regression test —**
   `.agent/scripts/test_check_commit_identity.sh` mirroring the
   pattern of the existing `test_identity_introspection.sh`. Covers:
   - Agent branch + `AGENT_NAME` set + human email → hook fails (exit 1)
   - Agent branch + `AGENT_NAME` set + agent email → hook passes (exit 0)
   - Agent branch + `AGENT_NAME` unset → permissive (passes either email)
   - Non-agent branch + `AGENT_NAME` set → permissive (passes either email)
   - Detached HEAD → permissive (no false-positive lockout)
   Hooked into `Makefile` `test` target if there's one for shell tests;
   otherwise standalone-runnable.

3. **Mechanism B — update `AGENTS.md` to recommend the per-commit
   `git -c` pattern.** New short section after "AI Signature" titled
   "Agent Commit Identity", documenting:
   - The canonical pattern:
     `git -c user.name="$AGENT_NAME" -c user.email="$AGENT_EMAIL" commit -m "…"`
   - Rationale: Bash tool calls don't persist shell state across
     invocations; the env-var script remains correct for human use
     and as a fallback, but per-commit `-c` is the robust pattern for
     agents that may commit across multiple Bash calls (which is
     all agents that touch this workspace).
   - Cross-reference to `set_git_identity_env.sh` which still exports
     `$AGENT_NAME` / `$AGENT_EMAIL` for the `-c` flags to reference.

4. **Mechanism B cascade —** verify `$AGENT_EMAIL` is exported by
   `set_git_identity_env.sh` (it is, at line 184 — no edit needed).
   Cascade the new "Agent Commit Identity" section to the three
   framework adapters (`.github/copilot-instructions.md`,
   `.agent/instructions/gemini-cli.instructions.md`,
   `.agent/AGENT_ONBOARDING.md`) per ADR-0006 and the consequences map.
   Also update `.agent/AI_IDENTITY_STRATEGY.md` (the canonical
   multi-framework identity doc — references the env-var pattern and
   should mention the `git -c` pattern alongside).

5. **Mechanism C — CI step.** Add a new CI-callable Python script
   `.agent/hooks/check_pr_authors.py` that takes a PR number, fetches
   commits via `gh api`, and exits non-zero if any commit on an
   agent-convention branch (per `identity_patterns.is_agent_branch`)
   has an author email matching `HUMAN_PATTERNS`. The script imports
   from `identity_patterns.py` so A and C share the source of truth.
   Then add a new `validate-commit-identity` job to
   `.github/workflows/validate.yml` that invokes the script on
   `pull_request` events. Independent of `$AGENT_NAME` env-var context
   (CI runners don't have it) — operates on git author email alone.

6. **Self-test on the fix-pass commit.** Before merging this PR, run
   the regression test script + verify CI's new step catches a
   deliberately-mis-authored commit if pushed. Document in
   `progress.md` as Local Review entry.

## Files to Change

| File | Change |
|------|--------|
| `.agent/hooks/identity_patterns.py` | **NEW** shared module: `ACCEPTED_AGENT_PATTERNS`, `HUMAN_PATTERNS`, `PERMISSIVE_ACCEPTED_PATTERNS`, agent-branch regex (`feature/[iI][sS][sS][uU][eE]-N` + `skill/<name>-<timestamp>`), `is_agent_branch(name)` helper. Single source of truth for A and C. |
| `.agent/hooks/check-commit-identity.py` | Import from `identity_patterns`. Add branch-detection (`git symbolic-ref --short HEAD`) + `$AGENT_NAME` check. When both gates fire, restrict to `ACCEPTED_AGENT_PATTERNS`; otherwise use `PERMISSIVE_ACCEPTED_PATTERNS` (current behavior). |
| `.agent/hooks/verify-issue-branch.py` | Refactor to import the feature/issue regex from `identity_patterns` (same source of truth). Behavior preserved. |
| `.agent/hooks/check_pr_authors.py` | **NEW** CI-callable script. Takes a PR number, fetches commits via `gh api`, fails if any commit on an agent-convention branch has an author email matching `HUMAN_PATTERNS`. Uses `identity_patterns`. |
| `.agent/scripts/test_check_commit_identity.sh` | **NEW**. Five-case regression test (agent-branch × agent-env matrix + detached HEAD). |
| `.agent/scripts/set_git_identity_env.sh` | No change — `$AGENT_EMAIL` already exported at line 184. (Verified before plan-edit, not a TODO.) |
| `AGENTS.md` | New "Agent Commit Identity" subsection after "AI Signature" documenting the `git -c` pattern + Bash-tool-state rationale. Add `check_pr_authors.py`, `identity_patterns.py`, and `test_check_commit_identity.sh` to the Script Reference table. |
| `.agent/AI_IDENTITY_STRATEGY.md` | Add a section noting the `git -c` per-commit pattern alongside the existing env-var setup. Cross-reference AGENTS.md. |
| `.github/copilot-instructions.md` | Mirror new section. Per ADR-0006. |
| `.agent/instructions/gemini-cli.instructions.md` | Mirror new section. Per ADR-0006. |
| `.agent/AGENT_ONBOARDING.md` | Mirror new section. Per ADR-0006. |
| `.github/workflows/validate.yml` | New `validate-commit-identity` job: on `pull_request`, invokes `check_pr_authors.py` against the PR. |
| `Makefile` | Verify `make test` / `make lint` runs the new shell test (or add explicit target). Confirm during implementation. |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Hook fails loudly with actionable message ("commit author X on agent branch — set `$AGENT_NAME` correctly or use `git -c user.email=...`"). CI failure is visible on PR. |
| **Enforcement over documentation** | Core principle this issue addresses. AGENTS.md AI-signature rule is currently doc-only; Mechanisms A + C enforce it at two layers per ADR-0004. |
| Only what's needed | Tighten existing hook rather than build new one. Single CI job, not a separate workflow file. No new env-var script. |
| A change includes its consequences | AGENTS.md edit cascades to 3 framework adapters (per consequences map). Regression test added in same PR. Sub-agent dispatch prompts in skill bodies (plan-task SKILL.md, review-code SKILL.md) noted as informational — agents read these but they're prose, not configuration; no separate file to update. |
| Test what breaks | Five-case regression test covers the dual-gate semantics. The hook can no longer fail-open silently the way it did on PR #464. |
| Workspace vs project separation | Workspace-level enforcement; project repos can opt-in by adding the same hook. |
| Primary framework first, portability where free | Hook is framework-agnostic Python. CI is generic GitHub Actions. AGENTS.md cascade preserves framework-agnostic guidance. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0004 — Enforcement hierarchy | Yes | Three layers: instructions (B) + pre-commit hook (A) + CI (C). Matches the ADR's "no single layer is sufficient." |
| 0005 — Layered enforcement | Yes | CI/branch protection is authoritative (Mechanism C); pre-commit provides local feedback (Mechanism A); instructions express intent (Mechanism B). |
| 0006 — Shared AGENTS.md | Yes | AGENTS.md is the source of truth; three framework adapters mirror per the consequences map. |
| 0011 — Field mode | Yes | The dual gate (`$AGENT_NAME` set AND agent-branch name) preserves field-mode hotfix workflow on the default branch — those commits won't trigger the strict path. |
| 0002 — Worktree isolation | Yes | Plan + implementation land on `feature/issue-468` workspace worktree. |
| 0003 — Workspace infra is project-agnostic | Yes | Hook, test, workflow, and instructions are generic. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `check-commit-identity.py` | `.agent/hooks/README.md` (if it documents accepted patterns) | Verify during implementation; doc fix folds into the hook commit if needed |
| `AGENTS.md` (new section) | `.github/copilot-instructions.md`, `.agent/instructions/gemini-cli.instructions.md`, `.agent/AGENT_ONBOARDING.md`, `.agent/AI_IDENTITY_STRATEGY.md` | Yes — all four listed in Files to Change |
| `AGENTS.md` Script Reference table | New scripts added there (`check_pr_authors.py`, `identity_patterns.py`, `test_check_commit_identity.sh`) | Yes — included in the AGENTS.md row above |
| New shell test (`test_check_commit_identity.sh`) | `Makefile` — verify it's picked up by `make test` / `make lint` | Yes — Makefile in Files to Change with verify-during-implementation note |
| `.github/workflows/validate.yml` (new job) | Branch protection rules — if the new check should be required to merge, update branch protection separately (per AGENTS.md "Ask First") | Flagged as Open Question 1, not in this PR |
| Sub-agent dispatch prompt prose in skill bodies (`plan-task` SKILL.md, `review-code` SKILL.md) | Adopt the `git -c` pattern when documenting commit examples for sub-agents | Yes — verify via workspace grep during implementation |

## Open Questions

1. **Branch protection** — should the new CI check be a *required* status check for merging into `main`, or just informational? If required, that's a separate config change in the repo's GitHub settings (per AGENTS.md "Ask First" — modifying branch protection requires human approval). Flag for the user at PR open time, not now.

**Resolved during plan-revision (2026-05-18, after review-plan pass)**:

- **Hook escape hatch (Q2)**: **No.** A `GIT_IDENTITY_OVERRIDE=1` env-var bypass creates a foot-gun without solving a real workflow problem — a maintainer who needs to commit as themselves on an agent branch can either rebase off the branch, cherry-pick to a non-agent branch, or `unset AGENT_NAME` in the current shell (which already triggers permissive mode). Decision: no escape hatch.

## Estimated Scope

Single PR, ~12 file edits + 3 new files (`identity_patterns.py`,
`check_pr_authors.py`, `test_check_commit_identity.sh`). Atomic
commits suggested grouping:

1. Shared module + hook tighten + regression test (new
   `identity_patterns.py`, modified `check-commit-identity.py`,
   refactored `verify-issue-branch.py`, new
   `test_check_commit_identity.sh`, Makefile if needed)
2. AGENTS.md "Agent Commit Identity" section + 4-doc cascade
   (3 framework adapters + `AI_IDENTITY_STRATEGY.md`) +
   AGENTS.md Script Reference table updates
3. CI: new `check_pr_authors.py` + new `validate-commit-identity`
   job in `validate.yml`

Three commits, each independently revertible.

## Implementation Notes

- **Shared `identity_patterns.py` module (Step 1a)** — added after the
  review-plan pass surfaced a DRY concern: the human-email pattern
  list would otherwise be hard-coded in both `check-commit-identity.py`
  (Mechanism A) and `check_pr_authors.py` (Mechanism C), and future
  updates would silently desync. The shared module also absorbs the
  agent-branch regex (`^feature/[iI][sS][sS][uU][eE]-(\d+)` from
  `verify-issue-branch.py` plus the skill-worktree pattern from
  AGENTS.md). User explicitly chose "extract to shared config" over
  the alternatives (hard-code-with-comments / defer-to-follow-up).

- **Honest acknowledgement of Mechanism A's bypass surface (Step 1)** —
  the carve-out for permissive mode (`AGENT_NAME` unset → permissive)
  is bypassed by the exact root cause #468 describes. This is not a
  flaw in the plan — it's a fundamental limit of pre-commit hooks that
  read env vars in the commit-time shell. Mechanism C (CI) is the only
  layer immune to this bypass. The rationale shift: C is not optional
  belt-and-suspenders; it's load-bearing for the principal failure
  mode this issue addresses. Plan now states this explicitly.

- **Resolved Open Question 2 (escape hatch)** during plan revision
  rather than deferring to implementation. The plan's own rationale
  already implied "no" — explicit resolution avoids scope churn during
  implementation.
