# Plan: Workflow skills as a composable timeline

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/470

## Context

Umbrella consolidating the workflow-skill composability vision. The
issue body has the full design; this plan adds the implementation
decisions on top of the umbrella and lands phase A in one PR, with
phases B and C as stacked sub-issues filed during implementation.

Three decisions locked in before planning (do not revisit):

1. **Sub-agent output = `progress.md` only.** Sub-agents are read-only
   against gh; the parent / downstream skills publish.
2. **ADR for the entry-type vocabulary lands as first commit of
   phase A.** New `docs/decisions/0013-progress-md-entry-type-vocabulary.md`.
3. **Dispatch wrapper + prompt boilerplate (both).** Phase C adds
   `.agent/scripts/dispatch_subagent.sh` AND updates each skill's
   handoff prompt to include the `git -c` pattern from PR #471.

Sub-agent `review-issue` pass (comment posted 2026-05-18) confirmed
umbrella scope is right; phasing A → B → C; ADR is action-needed.

## Approach

### Phase A (this PR — `feature/issue-470`)

1. **Commit 1**: New ADR `docs/decisions/0013-progress-md-entry-type-vocabulary.md`.
   Captures the canonical entry types (`## Issue Review`, `## Plan
   Authored`, `## Plan Review`, `## Local Review` /
   `## Local Review (Pre-Push)`, `## Integrated Review`,
   `## Implementation`), the schema (status/when/by/sources fields),
   and the consume-by-entry-type-filter rule. References from
   `principles_review_guide.md` ADR table.

   **Predecessor recognition**: ADR-0013 explicitly states that
   `## External Review` (the pre-phase-B triage-reviews entry name)
   is the recognized predecessor of `## Integrated Review`. Existing
   `## External Review` entries on closed PRs remain as historical
   artifacts and are NOT migrated. The semantic shift (triage-reviews
   as integrator, not just external-review classifier) happens in
   phase B; until then, triage-reviews continues to write
   `## External Review` and the ADR acknowledges this transitional
   state. Phase B's redesign retires the predecessor name for new
   entries.

2. **Commit 2**: Add `## 8. Persist to progress.md` final step to
   `review-issue`, `plan-task`, and `review-plan` SKILL.md files
   (mirroring `review-code` step 8). Each gets its own entry type
   per the ADR. Update `principles_review_guide.md` to reference
   the ADR and to add a "If you add a workflow skill → also persist
   to progress.md per ADR-0013" consequences-map row.

### Phase B (file as sub-issue after phase A merges)

`triage-reviews` becomes integrator. Step 3 extended to read all
prior `progress.md` entries by entry type. Step 5/6 produce
findings table with `sources` column (cross-source confirmations
flagged). Step 7's entry renamed `## Integrated Review` per the
ADR. Optional `.agent/scripts/progress_read.sh` helper.

**Reference output format**: `.agent/work-plans/issue-468/progress.md`
contains a hand-written `## Integrated Review` entry (from the manual
triage-reviews-as-integrator emulation that motivated this umbrella).
Phase B's redesigned skill should produce entries matching that shape:
findings table with a `sources` column, cross-source confirmations
flagged as the strongest signal, false-positive dismissals with
justification listed separately.

### Phase C (file as sub-issue after phase B)

`.agent/scripts/dispatch_subagent.sh` wrapper. Each workflow skill's
final step gains a "Next step" handoff prompt with `git -c` boilerplate.
`--auto-next` flag scope TBD (Open Question 1).

## Files to Change (phase A only)

| Phase | File | Change |
|------|------|--------|
| A.1 | `docs/decisions/0013-progress-md-entry-type-vocabulary.md` | **NEW** ADR. Context: composable-timeline vision (#470 + #469's reframe). Decision: canonical entry types + schema + consume-by-filter rule. Consequences: workflow skills must include a progress.md persist step. |
| A.1 | `.agent/knowledge/principles_review_guide.md` | Add ADR-0013 to applicability table. Add consequences-map row: "workflow skill added → also persist to progress.md per ADR-0013." Add a short vocab reference block listing the entry types. |
| A.2 | `.claude/skills/review-issue/SKILL.md` | Add **new step 8: "Persist to progress.md"** (current step 7 "Post findings as a comment" stays; new step 8 added after). Persist `## Issue Review` entry to `.agent/work-plans/issue-<N>/progress.md` in the owning repo, commit. **Step 8 creates the worktree on demand if none exists** — review-issue is typically the first lifecycle skill, so no worktree exists yet, and progress.md commits cannot land on the protected default branch. Worktree creation uses `worktree_create.sh` without `--plan-file` (no draft PR — that comes when plan-task runs). |
| A.2 | `.claude/skills/plan-task/SKILL.md` | **Insert new step 8 "Persist to progress.md"; renumber existing step 8 "Report to user" → step 9.** New step persists `## Plan Authored` entry between PR creation (step 7) and reporting (step 9 / formerly 8), so the report can cite the progress.md commit SHA. Schema includes plan file path + initial-commit SHA. |
| A.2 | `.claude/skills/review-plan/SKILL.md` | **Add new step 6: "Persist to progress.md"** (current ends at step 5 "Produce the report"). Persist `## Plan Review` entry with verdict + findings checklist. |

No changes to `review-code` (already writes `## Local Review`) or
`triage-reviews` (step-7 entry will be renamed in phase B, not now).

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Capture decisions, not just implementations | **Core.** ADR-0013 is the first commit — vocabulary lives in `docs/decisions/`, not just five skill files. Review-issue pass flagged this as action-needed; phase A.1 addresses it. |
| A change includes its consequences | Phase A's two consequences (skill cascade + principles-guide cross-reference) both in the PR. Phases B and C are sub-issues; not part of this PR's consequence surface. |
| Only what's needed | Phase A is a minimal first step — three skills gain one section each. ADR is short. Defers `progress_read.sh` helper (phase B), `dispatch_subagent.sh` (phase C), and `--auto-next` (open question). |
| Improve incrementally | A → B → C as stacked PRs; phase A independently merge-able and useful (more skills writing to progress.md improves the timeline before B/C exist to consume it). |
| Workspace vs project separation | All workspace tooling; project repos consume via the skills. |
| Workspace improvements cascade to projects | The ADR + skill changes propagate to projects automatically since skills are workspace-resident. Adapter docs (`.github/copilot-instructions.md` etc.) untouched in phase A — vocabulary is internal to skill bodies. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0001 — Adopt ADRs | Yes | Phase A.1 adds ADR-0013 capturing the new vocabulary as a workspace decision rather than scattered skill-level prose. |
| 0002 — Worktree isolation | Yes | Plan + implementation on `feature/issue-470` workspace worktree. |
| 0003 — Workspace infra project-agnostic | Yes | Vocabulary + skills are framework-agnostic; not coupled to any project. |
| 0006 — Shared AGENTS.md | Watch (phase C, not A) | Phase C's handoff-prompt boilerplate will cascade to framework adapters per consequences map. Phase A doesn't touch AGENTS.md. |
| 0010 — git-bug | Watch | Sub-agent local-first publishing model (decision #1) is consistent with ADR-0010's direction; sub-agents could optionally file git-bug issues for newly discovered work, but that's parent-discretion not in-skill — flag in phase C. |
| 0012 — Cross-reference addendums | Yes (mechanism) | If ADR-0013's relationship to ADR-0010 / ADR-0006 matters later, use cross-reference addendums per ADR-0012's mechanism rather than rewriting. |

## Consequences

| If we change… | Also update… | Included in plan? |
|---|---|---|
| Add ADR-0013 | `principles_review_guide.md` ADR table | Yes — A.1 |
| Skill SKILL.md files add progress.md persistence | `principles_review_guide.md` consequences-map row for "new workflow skill" | Yes — A.1 |
| New entry-type vocabulary | Existing progress.md entries on closed PRs. Verification: `## Local Review`, `## Local Review (Pre-Push)` are already canonical (used by review-code). `## External Review` from issue-452/460/461 progress.md is the **predecessor name for `## Integrated Review`** — ADR-0013's predecessor-recognition clause covers it; no migration of historical files. New `## External Review` writes continue from the pre-phase-B triage-reviews until phase B retires the name. | Yes — explicit in ADR-0013 predecessor recognition; no migration commit needed |
| Sub-agent dispatch model (phase C) | Adapter docs (3 framework files) | Phase C — out of scope for this PR |
| Workflow skill list grows | Skill index in adapter docs (`copilot-instructions.md`, `gemini-cli.instructions.md`, `AGENT_ONBOARDING.md`) | Verify no new skill in phase A — only edits to existing skills, so no skill-list cascade. Phase B/C may need cascade. |

## Open Questions

1. **`--auto-next` flag (phase C)**: include in initial phase C scope, or defer until handoff-prompt UX is field-tested? Plan recommends defer; user can override at phase C planning time.

2. **Enforcement of "skill writes progress.md" rule**: doc-only (ADR-0013 prose + skill spec) or add a pre-commit / CI check? Plan recommends doc-only for now per "improve incrementally" — escalate to enforcement only if drift is observed in practice (matches the dynamic from PR #471 review).

3. **Sub-issue / stacked-PR progress.md location**: sensible default the plan adopts — each sub-issue has its own `.agent/work-plans/issue-<N>/progress.md`; umbrella's progress.md aggregates only when a cross-phase summary is meaningful. Confirm during phase B/C planning.

4. **Field-mode interaction**: sensible default the plan adopts — progress.md lives at the usual path in the owning repo; in field-mode repos it commits via field-mode workflow (commit-to-default-branch, push-to-gitcloud, no PR). No special-case logic needed in skill bodies. Confirm during phase B/C planning.

## Estimated Scope

**Phase A (this PR)**: 5 file edits in 2 atomic commits, ~150–200
lines of additions total (ADR is ~50 lines; each skill SKILL.md gets
~30 lines for its persist step + a few lines in
`principles_review_guide.md`). Markdown-only.

**Phase B + C**: separate stacked PRs per umbrella's "Sub-issues
worth splitting" section. Sub-issues filed during phase A
implementation so the umbrella doesn't stall behind issue-filing
ceremony.

## Implementation Notes

- **review-issue now creates the worktree on demand.** During phase A
  implementation the user redirected: rather than gracefully skipping
  persistence when no worktree exists, `review-issue` should be the
  first lifecycle skill to *create* the worktree. Rationale: progress.md
  is canonical per ADR-0013; commits to it cannot land on the protected
  default branch; the "skip persistence" carve-out would have meant
  review-issue's findings live only as a GitHub comment until plan-task
  runs (asymmetric with the other skills' write semantics). The cost is
  the user may abandon a review-only worktree — handled by
  `worktree_remove.sh --issue <N>`. plan-task step 4's existing "if a
  worktree exists, enter it" branch handles the hand-off cleanly.
