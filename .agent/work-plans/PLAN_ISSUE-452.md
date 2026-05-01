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
address this: a fresh-context **Adversarial Specialist** (Claude-only —
relies on subagent dispatch), **depth tiers** that auto-route to deeper
review on risky diffs, **`progress.md` persistence** across sessions, and
**flexible `review-plan` inputs** (file/issue, not only PR number). The
`inspiration_agent_workspace_digest.md` was last refreshed 2026-03-23 (a
month stale) and the fork landed these recently — the plan includes a
refresh step before copying, and treats the design as experimental.

The scope also includes light-touch governance changes (docs-only
enforcement) to make agents actually **run** `/review-code` before
pushing. Composite `/ship` skill deferred to a follow-up issue per the
review-issue findings. **This is explicitly a soft nudge against the
"Enforcement over documentation" principle; the PR body will flag it
as a known deferral.**

A per-issue directory convention (`.agent/work-plans/issue-<N>/`) is
introduced for new artifacts — plan, progress, review output,
adversarial findings — without disturbing the 43 legacy flat plans.
Symlinks bridge legacy plans into the new layout so consumer skills
only look at one path.

## Approach

One PR, three logical commit groups. Scope is borderline — if group A
alone approaches review-fatigue size, split B+C off as a follow-up.

**Commit group A — directory convention, depth dispatch, Adversarial Specialist, persistence**

1. **Refresh the inspiration digest** — re-check the fork at current
   HEAD and update `inspiration_agent_workspace_digest.md` so we port
   what exists today, not a 2026-03-23 snapshot.
2. **Introduce per-issue directory convention.**
   - New plans write to `.agent/work-plans/issue-<N>/plan.md`.
   - Legacy flat plans (`PLAN_ISSUE-<N>.md`) stay where they are.
   - Eager migration: create `.agent/work-plans/issue-<N>/plan.md` as
     a symlink to `../PLAN_ISSUE-<N>.md` for every existing flat plan
     (~43 files). Single mechanical commit.
   - After this lands, every issue's plan is reachable at
     `issue-<N>/plan.md` — skills never branch on legacy vs. new.
   - Update `plan-task/SKILL.md` to write new plans nested.
3. **Add `.agent/knowledge/review_depth_classification.md`** — structured
   as Context / Current thinking / Consequences, leading with
   **Status: experimental**. Names what we don't yet know (whether
   thresholds produce the right tier on real PRs). Captures the signal
   table in two columns: workspace-repo triggers (`AGENTS.md`,
   `docs/decisions/`, `.claude/skills/`, `.repos`, `setup_layers.sh`,
   `Makefile`, `framework_config.sh`) and project-repo triggers
   (`.agents/README.md`, project `PRINCIPLES.md`, project `docs/decisions/`).
   No ADR — fork landed this recently; we lack experience to commit to
   ADR-level permanence.
4. **Update `review-code/SKILL.md`:**
   - Accept positional `[light|standard|deep]` depth override (matches
     the fork verbatim); auto-classify when omitted using the signal
     table from step 3.
   - Accept pre-push mode: `review-code` (no arg) diffs against the
     current repo's default branch (`gh repo view --json defaultBranchRef`,
     fall back to `main`); `--base <branch>` overrides. `review-code <N>`
     or `<url>` is post-PR mode.
   - Dispatch specialists by tier; **Adversarial Specialist** activates
     at **Standard + Deep** (Standard prompt covers edge cases /
     assumptions / subtle bugs / logic; Deep prompt adds security /
     concurrency / cross-cutting effects), launched via `Agent` as a
     fresh-context subagent with no context from other specialists.
     Document that Adversarial is **Claude-only** — structural
     limitation; other frameworks still get depth dispatch +
     persistence.
   - Document Adversarial's cross-repo limitation: fresh-context reviewer
     reads only the diff, so it cannot catch cross-repo consequences in
     the layered workspace. The Governance Specialist carries that load
     via the consequences map.
   - Append findings to `.agent/work-plans/issue-<N>/progress.md` in
     the owning repo.
5. **Repo-awareness pass.** The fork is single-repo; its skill text and
   examples assume one `.agent/work-plans/`, one governance layer. Pass
   over the ported text (review-code, triage-reviews, review-plan) and
   generalize: "in the owning repo," repo-aware lookups, per-repo
   governance paths.

**Commit group B — triage-reviews persistence + review-plan input flex**

6. **Update `triage-reviews/SKILL.md`** to append classification results
   to `.agent/work-plans/issue-<N>/progress.md` in the owning repo.
   **Preserve "Justify every false positive"** — do not regress it when
   reconciling with the fork's version.
7. **Update `review-plan/SKILL.md`** to accept three input forms:
   PR number (current), `--issue <N>` (find plan at the nested path in
   the current repo; require `--repo <owner/repo>` for cross-repo),
   or a file path. PR-number path stays default.

**Commit group C — governance and adapter propagation**

8. **Update `AGENTS.md`** Post-Task Verification: add "Before opening a
   PR" bullet — *run `/review-code` against your diff; address findings
   before pushing*.
9. **Update `skill_workflows.md`** — insert `review-code` between
   *implement* and *push/open-PR* in the per-issue lifecycle.
10. **Update `plan-task/SKILL.md`** closing "Report to user" to mention
    `/review-code` as the next step after implementation.
11. **Propagate to framework adapters** per ADR-0006 — one-line mirror +
    cross-link to AGENTS.md in `.github/copilot-instructions.md`,
    `.agent/instructions/gemini-cli.instructions.md`, and
    `.agent/AGENT_ONBOARDING.md`. Note Adversarial is Claude-only.
12. **Mark ported pieces in `inspiration_agent_workspace_digest.md`** —
    move the four pieces into a "Ported" section with PR link.

**Validation (smoke test, not CI):** run `/review-code 448 --depth=deep`
and confirm (a) Adversarial fires, (b) progress.md is written at
`.agent/work-plans/issue-448/progress.md`, (c) depth auto-classification
on a medium PR lands on Standard.

## Files to Change

| File | Change |
|------|--------|
| `.agent/knowledge/inspiration_agent_workspace_digest.md` | Refresh at start (step 1); mark ported (step 12) |
| `.agent/work-plans/issue-<N>/plan.md` × ~43 | **New** symlinks to legacy flat plans (step 2) |
| `.agent/knowledge/review_depth_classification.md` | **New** — experimental, repo-aware signal table |
| `.claude/skills/review-code/SKILL.md` | Depth arg, dual-mode detection, Adversarial (Claude-only), progress persistence |
| `.claude/skills/triage-reviews/SKILL.md` | Progress persistence; preserve "Justify every false positive" |
| `.claude/skills/review-plan/SKILL.md` | Accept PR number / `--issue <N>` / file path |
| `.claude/skills/plan-task/SKILL.md` | Write nested; closing hint to run `/review-code` |
| `AGENTS.md` | Post-Task Verification: "Before opening a PR" bullet |
| `.agent/knowledge/skill_workflows.md` | Lifecycle: review-code between implement and push |
| `.github/copilot-instructions.md` | Mirror AGENTS.md pre-push expectation + Adversarial-Claude-only note |
| `.agent/instructions/gemini-cli.instructions.md` | Same |
| `.agent/AGENT_ONBOARDING.md` | Same |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Only what's needed | Explicit defer list respected (cross-model adversarial, git-bug fallback, de-ROS config). Adversarial = highest-leverage subset. |
| A change includes its consequences | Adapter propagation (ADR-0006) in-scope (step 11). Digest refresh (step 1) and mark-ported (step 12) keep inspiration-tracker from re-flagging. Lifecycle update propagated through `skill_workflows.md` and `plan-task` closing hint. Symlink migration keeps legacy paths functional (no broken references). |
| Enforcement over documentation | **Soft nudge only — flagged as known deferral.** Docs-only enforcement of pre-push `/review-code`. Composite `/ship` is the planned next enforcement layer; PR body calls this out explicitly so the deferral is visible, not silent. |
| Capture decisions, not just implementations | `review_depth_classification.md` documents the *why* for thresholds. Framed experimental — thresholds expected to churn until we have data. Promotion to ADR reserved for when the structure stabilizes. |
| Improve incrementally | Single scoped slice of the fork's review tooling. Directory convention introduced without disturbing legacy plans. |
| Primary framework first | Adversarial Specialist uses Claude Code subagent dispatch — full primary-framework capability. Adapters document the limitation rather than hobble the primary tool. |
| Test what breaks | Prose skills; validation is smoke-test on PR #448 (the motivating case). No CI surface. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0001 Adopt ADRs | No | Depth-tier design intentionally not promoted to ADR while experimental. Revisit after real PR data. |
| 0002 Worktree isolation | Yes | Plan committed from `.workspace-worktrees/issue-workspace-452`. |
| 0003 Project-agnostic | Yes | Adversarial prompt stays generic; no layer-specific or package-specific references. Repo-awareness pass (step 5) ensures skill text generalizes. |
| 0004/0005 Enforcement hierarchy | Watch | Docs-only enforcement for pre-push expectation — explicit Phase-1 deferral, flagged in PR body. |
| 0006 Shared AGENTS.md | Yes | Step 11 propagates to adapters. |
| 0007 Retain Make | No | No Makefile changes. |
| 0008 ROS 2 conventions | N/A | No ROS package code touched. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `AGENTS.md` | Framework adapters (copilot, gemini-cli, AGENT_ONBOARDING) | **Yes** — step 11 |
| `plan-task` / `review-code` / `triage-reviews` / `review-plan` | `skill_workflows.md` if lifecycle position changes | **Yes** — step 9 |
| Plan file layout | Consumer skills that look up plans | **Yes** — symlink migration (step 2) keeps all plans reachable at one path; skills updated in steps 4, 6, 7 |
| `.agent/knowledge/inspiration_agent_workspace_digest.md` | — (digest is tracked only by inspiration-tracker) | **Yes** — steps 1 and 12 |

## Open Questions

All resolved during discussion:

1. **Directory convention** → legacy flat plans stay as-is; all new
   artifacts nest under `.agent/work-plans/issue-<N>/`; eager symlink
   migration bridges legacy plans so skills only check one path.
2. **Depth override syntax** → `/review-code <N> --depth=light|standard|deep`.
3. **ADR vs knowledge doc** → knowledge doc, experimental framing. No
   ADR until thresholds stabilize with real-PR data.
4. **Pre-push mode base detection** → inferred from default branch
   with `--base <branch>` override.
5. **Adapter propagation depth** → one-line mirror + cross-link to
   AGENTS.md; adapters note Adversarial is Claude-only.

## Estimated Scope

**Single PR, staged commits** (A / B / C). Borderline per review-issue.
If commit group A alone runs long, split B+C off as a follow-up PR
against a base branch that includes A. No project-repo code touched.
Entirely workspace-level.

## Implementation Notes

- **Branch caught up to main before implementation.** Branch was 32
  commits behind main when implementation began (had been sitting
  since plan revision on 2026-04-24). Merged `origin/main` in cleanly
  with no conflicts before starting commit group A.
- **Steps 1 (digest refresh) and 11 (mark ported) bundled** into a
  single commit since both touch
  `inspiration_agent_workspace_digest.md` and the natural rhythm is
  to write the file once. The PR taken as a whole leaves the digest
  consistent with merged reality at merge time.
- **Step 5 (repo-awareness pass) folded inline** into the SKILL.md
  updates for steps 4 / 6 / 7 rather than a separate sweep, since
  generalizing single-repo language is most cleanly done while
  porting each section. No standalone commit for step 5.
- **Smoke-test validation deferred to post-merge.** The plan's
  validation checklist (`/review-code 448 --depth=deep` confirming
  Adversarial fires, progress.md is written, medium PR lands on
  Standard) can only be run after this PR merges and the new SKILL
  is loadable. Will be exercised as part of the next real PR review.
