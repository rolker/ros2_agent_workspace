# Plan: Port review-skill improvements from agent_workspace + encourage use

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/452

## Context

PR #448 surfaced a systemic failure mode in the current review tooling: 13
review rounds, ~45 Copilot findings, many of which were **consequences of
prior fixes** (a round-7 rename broke inbound anchors caught in round 11;
round-6 path arithmetic flagged in round 9). The shared context of the
existing specialist reviewers causes them to carry forward assumptions from
previous rounds — they don't re-read the whole diff cold.

The fork `rolker/agent_workspace` has evolved four pieces that directly
address this: a fresh-context **Adversarial Specialist**, **depth tiers**
that auto-route to deeper review on risky diffs, **`progress.md`
persistence** across sessions, and **flexible `review-plan` inputs**
(file/issue, not only PR number). The `inspiration_agent_workspace_digest.md`
was last refreshed 2026-03-23 (a month stale); the plan includes a
refresh step before copying so we track what actually exists in the fork
today, not a month-old snapshot.

The scope also includes light-touch governance changes to make agents
actually **run** `/review-code` before pushing — docs-only enforcement
for now, composite `/ship` skill deferred to a follow-up issue per the
review-issue findings.

## Approach

One PR, three logical commit groups. Review-issue recommended this staging
and warned scope is borderline — if group 1 alone runs long, we split.

**Commit group A — review-code depth dispatch + Adversarial Specialist + persistence**

1. **Refresh the inspiration digest** — re-check the fork at current HEAD
   and update `inspiration_agent_workspace_digest.md` before copying, so we
   port what exists today rather than a 2026-03-23 snapshot.
2. **Add `.agent/knowledge/review_depth_classification.md`** — in
   ADR-style (Context / Decision / Consequences), documenting the signal
   table (lines changed, file count, governance-touching paths like
   `AGENTS.md` / `docs/decisions/` / `.claude/skills/`) and the thresholds
   that trigger Light / Standard / Deep. Capture the *why* for the
   thresholds, not just the *what* (per review-issue recommendation 3 and
   the "Capture decisions" principle).
3. **Update `review-code/SKILL.md`** to: (a) accept a depth argument
   `--depth=light|standard|deep` and a pre-push mode selector, (b)
   dispatch specialists by tier, (c) add the **Adversarial Specialist**
   sub-section — a fresh-context subagent launched only at Deep tier with
   no context from other specialists (via `Agent` with a dedicated
   prompt), and (d) add the `progress.md` append step.
4. **Dual-mode detection in `review-code`**: argument pattern determines
   mode — `<number>` or URL → post-PR mode (diff against the PR base);
   no arg or a branch name → pre-push mode (diff against
   `origin/<base-branch>`). Document both modes and the detection logic
   in the SKILL.

**Commit group B — triage-reviews persistence + review-plan input flex**

5. **Update `triage-reviews/SKILL.md`** to append classification results
   to the same per-issue `progress.md` (step placement: after step 6 /
   "Classify and present plan"). Preserve the existing "Justify every
   false positive" guidance — do not regress it.
6. **Update `review-plan/SKILL.md`** to accept three input forms: PR
   number (current), `--issue <N>` (find plan at
   `.agent/work-plans/PLAN_ISSUE-<N>.md` in the current repo), or a file
   path. Keep the PR-number path as the default.

**Commit group C — governance and adapter propagation**

7. **Update `AGENTS.md`** Post-Task Verification section to add a "Before
   opening a PR" bullet: *run `/review-code` against your diff; address
   findings before pushing*.
8. **Update `skill_workflows.md`** — insert `review-code` between
   *implement* and *push/open-PR* in the per-issue lifecycle (today it's
   positioned "After implementation" which conflates self-review with
   post-PR review; the new framing supports both modes from step 4).
9. **Update `plan-task/SKILL.md`** closing "Report to user" to mention
   `/review-code` as the next step after implementation.
10. **Propagate to framework adapters** (ADR-0006 + consequences map):
    mirror the "run `/review-code` before pushing" expectation in
    `.github/copilot-instructions.md`, `.agent/instructions/gemini-cli.instructions.md`,
    and `.agent/AGENT_ONBOARDING.md`. If any adapter can't invoke the
    skill the same way (e.g., Copilot review mode), document the
    deliberate omission instead of silently skipping.
11. **Mark ported pieces in `inspiration_agent_workspace_digest.md`** —
    move the four pieces from tracked/pending into a "Ported" section
    with the PR link, so `inspiration-tracker` doesn't re-flag them.

## Files to Change

| File | Change |
|------|--------|
| `.agent/knowledge/inspiration_agent_workspace_digest.md` | Refresh to current fork HEAD (step 1); add Ported section (step 11) |
| `.agent/knowledge/review_depth_classification.md` | **New** — ADR-style depth-tier design + signal table |
| `.claude/skills/review-code/SKILL.md` | Depth arg, dual-mode detection, Adversarial Specialist, progress persistence |
| `.claude/skills/triage-reviews/SKILL.md` | Progress persistence step; keep "Justify every false positive" |
| `.claude/skills/review-plan/SKILL.md` | Accept PR number / `--issue <N>` / file path |
| `.claude/skills/plan-task/SKILL.md` | Closing hint to run `/review-code` before push |
| `AGENTS.md` | Post-Task Verification: "Before opening a PR" bullet |
| `.agent/knowledge/skill_workflows.md` | Lifecycle: review-code between implement and push |
| `.github/copilot-instructions.md` | Mirror AGENTS.md pre-push expectation (or documented omission) |
| `.agent/instructions/gemini-cli.instructions.md` | Same |
| `.agent/AGENT_ONBOARDING.md` | Same |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Only what's needed | Scope is explicit about what NOT to port (cross-model adversarial, git-bug fallback, plan-path migration, de-ROS config). Adversarial = highest-leverage subset. |
| A change includes its consequences | Framework adapter propagation (ADR-0006) is in-scope (step 10). Inspiration digest update (step 11) prevents re-flagging. `skill_workflows.md` + `plan-task` closing hint propagate the lifecycle change. |
| Enforcement over documentation | **Soft nudge only** — docs-only enforcement for the pre-push expectation. Acceptable incremental step; composite `/ship` skill is explicitly deferred to a follow-up issue. Flag in PR body as known deferral. |
| Capture decisions, not just implementations | `review_depth_classification.md` structured as Context/Decision/Consequences so the *why* behind thresholds survives (review-issue rec 3). Full ADR-0010-style record deferred unless user prefers. |
| Improve incrementally | Single scoped slice of the fork's review tooling, not the whole thing. |
| Primary framework first | Adversarial Specialist uses Claude Code subagent dispatch. Adapters (Copilot/Gemini) get the lifecycle expectation, not the subagent mechanics. |
| Test what breaks | Prose-heavy Markdown skills have no unit-testable surface. Validation is a smoke test: run `/review-code --depth=deep` on a recent PR and confirm Adversarial fires and `progress.md` is written. Not CI. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0001 Adopt ADRs | Yes | `review_depth_classification.md` uses Context/Decision/Consequences structure. A separate short ADR is an alternative — see Open Questions. |
| 0002 Worktree isolation | Yes | Plan is being committed from `.workspace-worktrees/issue-workspace-452`. |
| 0003 Project-agnostic | Yes | Adversarial Specialist prompt stays generic — no references to specific layers, packages, or domain concepts. |
| 0004/0005 Enforcement hierarchy | Watch | Docs-only enforcement is the explicit Phase-1 choice. PR body flags this as a deferral; `/ship` composite is the planned next layer. |
| 0006 Shared AGENTS.md | Yes | Step 10 propagates the AGENTS.md change to Copilot, Gemini, and AGENT_ONBOARDING adapters. |
| 0008 ROS 2 conventions | N/A | No ROS package code touched. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `AGENTS.md` | Framework adapters (copilot, gemini-cli, AGENT_ONBOARDING) | **Yes** — step 10 |
| A framework skill (`.claude/skills/`) | That framework's adapter file | **Yes** — adapters list review-code/triage-reviews/review-plan already; no name changes so no skill-list edit needed, but verify during step 10 |
| `plan-task` / `review-code` / `triage-reviews` / `review-plan` | `skill_workflows.md` if lifecycle position changes | **Yes** — step 8 |
| `.agent/knowledge/inspiration_agent_workspace_digest.md` | None — the digest is tracked only by inspiration-tracker skill runs | **Yes** — steps 1 and 11 |

## Open Questions

1. **`progress.md` placement — flat or nested?** Review-issue recommended
   flat (`.agent/work-plans/PROGRESS_ISSUE-<N>.md`) to stay consistent
   with the deferred plan-path migration (`PLAN_ISSUE-<N>.md` → nested
   is tracked separately and out of scope). Plan defaults to flat unless
   user prefers nested. **Recommend: flat.**
2. **Depth-tier override keyword syntax.** Options:
   (a) `/review-code <N> --depth=deep` — extends existing
   `<pr-number-or-url>` signature cleanly; `--depth=light|standard|deep`.
   (b) inline magic keyword. **Recommend: (a)** for discoverability.
3. **Is depth-tier design significant enough for its own ADR?**
   Review-issue flagged this as a real decision. Alternatives: (i)
   Context/Decision/Consequences inline in
   `review_depth_classification.md` (lighter, plan default); (ii) new
   ADR-0011 that references the knowledge doc for the signal table. If
   you want ADR-level permanence, we add (ii). **Recommend: (i)** —
   keep the knowledge doc as the decision record; promote to ADR only
   if thresholds become contentious.
4. **Pre-push mode diff base detection.** In post-PR mode the base is
   the PR base branch. In pre-push mode, we infer base from:
   `gh repo view --json defaultBranchRef` → fall back to `main`. Is
   this acceptable, or should pre-push mode require an explicit
   `--base <branch>` argument? **Recommend: inferred with override
   flag available.**
5. **AGENT_ONBOARDING / Copilot propagation depth.** Should the adapter
   files each get the full pre-push expectation, or a cross-link to
   AGENTS.md? ADR-0006 says adapters are thin wrappers — leaning
   toward a brief one-line mention + cross-link rather than full
   duplication. **Recommend: one-line + cross-link.**

## Estimated Scope

**Single PR, staged commits** (A / B / C groups above). Borderline per
review-issue — if commit group A alone approaches review-fatigue size,
split B + C off as a follow-up PR against a base branch that includes
A. No project-repo code touched. Entirely workspace-level.
