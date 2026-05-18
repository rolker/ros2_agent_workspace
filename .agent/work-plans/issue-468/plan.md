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
   current branch matches the agent-branch convention
   (`feature/issue-*` or `skill/<name>-<timestamp>`), require an
   agent-pattern email (`roland+*@ccom.unh.edu`) and reject the human
   patterns. Otherwise preserve current permissive behavior so the
   human can edit interactively and field-mode hotfixes still work.
   Branch detection uses `git symbolic-ref --short HEAD` (fail-safe to
   permissive on detached HEAD).

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

4. **Mechanism B cascade —** add `$AGENT_EMAIL` export to
   `set_git_identity_env.sh` if not already present (so step 3's
   recommended invocation works without the agent having to know
   their own email). Verify the three framework adapters
   (`.github/copilot-instructions.md`,
   `.agent/instructions/gemini-cli.instructions.md`,
   `.agent/AGENT_ONBOARDING.md`) mirror the new section per ADR-0006
   and the consequences map.

5. **Mechanism C — CI step.** Add a job to
   `.github/workflows/validate.yml` that fetches the PR commits via
   `gh api` and fails if any author email matches the human patterns
   (`roland@ccom.unh.edu`, `roland@rolker.net`) on a branch matching
   the agent convention. Runs on `pull_request` only (the existing
   workflow already filters there). Independent of `$AGENT_NAME`
   env-var context, which CI doesn't have.

6. **Self-test on the fix-pass commit.** Before merging this PR, run
   the regression test script + verify CI's new step catches a
   deliberately-mis-authored commit if pushed. Document in
   `progress.md` as Local Review entry.

## Files to Change

| File | Change |
|------|--------|
| `.agent/hooks/check-commit-identity.py` | Add branch-detection (`git symbolic-ref --short HEAD`) + `$AGENT_NAME` check. When both gates fire, swap `ACCEPTED_PATTERNS` to agent-only (`roland+*@ccom.unh.edu`). |
| `.agent/scripts/test_check_commit_identity.sh` | NEW. Five-case regression test (agent-branch×agent-env matrix + detached HEAD). |
| `.agent/scripts/set_git_identity_env.sh` | Verify/add `AGENT_EMAIL` export so `git -c user.email="$AGENT_EMAIL"` works downstream. |
| `AGENTS.md` | New "Agent Commit Identity" subsection after "AI Signature" documenting the `git -c` pattern + rationale. |
| `.github/copilot-instructions.md` | Mirror new section. Per ADR-0006. |
| `.agent/instructions/gemini-cli.instructions.md` | Mirror new section. Per ADR-0006. |
| `.agent/AGENT_ONBOARDING.md` | Mirror new section. Per ADR-0006. |
| `.github/workflows/validate.yml` | New `validate-commit-identity` job: on `pull_request`, iterate PR commits, fail if author email matches human pattern on agent-convention branch. |

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
| `check-commit-identity.py` | `.agent/hooks/README.md` (if it documents the hook's accepted patterns) | Verify during implementation; doc fix folds into the hook commit if needed |
| `AGENTS.md` (new section) | `.github/copilot-instructions.md`, `.agent/instructions/gemini-cli.instructions.md`, `.agent/AGENT_ONBOARDING.md` | Yes — listed in Files to Change |
| `set_git_identity_env.sh` (add AGENT_EMAIL export) | None known; the new var is additive | Verify with `rg -F '$AGENT_EMAIL'` before commit |
| `.github/workflows/validate.yml` (new job) | Branch protection rules (which checks block merge) — if the new check should be required to merge, update branch protection separately | Flag as follow-up if user wants the new check required for merge |
| Sub-agent dispatch prompt prose in skill bodies (`plan-task` SKILL.md, `review-code` SKILL.md) | Adopt the `git -c` pattern when documenting commit examples for sub-agents | Yes — verify via workspace grep during implementation |

## Open Questions

1. **Branch protection** — should the new CI check be a *required* status check for merging into `main`, or just informational? If required, that's a separate config change in the repo's GitHub settings (per AGENTS.md "Ask First" — modifying branch protection requires human approval). Flag for the user at PR open time, not now.
2. **Hook escape hatch** — should there be an explicit `GIT_IDENTITY_OVERRIDE=1` env var that lets a maintainer commit as themselves on an agent branch (e.g., to fix something the agent broke in-place)? Open question, default: no — the user can always cd out of the agent branch or rebase, and the override creates a foot-gun. Decide based on whether the user wants it.

## Estimated Scope

Single PR, ~8 file edits + 1 new test script. Atomic commits suggested
grouping: (1) hook tighten + test, (2) AGENTS.md + 3 adapter cascade
+ AGENT_EMAIL export, (3) CI job. Three commits, each independently
revertable.
