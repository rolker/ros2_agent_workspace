# Plan: Document that work plans live in the repo that owns the issue

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/381

## Context

Work plans (`.agent/work-plans/PLAN_ISSUE-<N>.md`) exist in both the workspace
repo and project repos. Project repos like `unh_marine_autonomy`,
`ben_description`, and `unh_marine_simulation` already have their own
`.agent/work-plans/` directories with committed plans. However, workspace
documentation only describes the workspace copy, which caused Copilot to
incorrectly flag layer worktree instructions in PR #374.

The convention — plans follow the repo that owns the issue — needs to be
made explicit in the docs that currently assume workspace-only.

## Approach

1. **Update `.agent/work-plans/README.md`** — Add a section explaining that
   this directory pattern is replicated in project repos (under `.agents/work-plans/`).
   Note that the workspace copy documents workspace issues; project repos
   document their own issues.

2. **Update `.github/copilot-instructions.md`** — Change the line about
   "`.agent/work-plans/PLAN_ISSUE-<N>.md`" to clarify it applies to both
   workspace and project repos (plan lives in the repo being changed).

3. **Update `.github/instructions/work-plans.instructions.md`** — Broaden
   the `applies_to` scope description and add a note that project repos
   have the same pattern under `.agents/work-plans/`.

4. **Update `.claude/skills/plan-task/SKILL.md`** — In step 5, clarify that
   the plan file path is relative to the current repo. For project repo
   issues, the plan is committed in the project repo's `.agents/work-plans/`,
   not the workspace's.

5. **Update `.claude/skills/review-pr/SKILL.md`** — Line 41 references
   `.agent/work-plans/PLAN_ISSUE-*.md` without noting it's repo-relative.
   Add brief clarification.

## Files to Change

| File | Change |
|------|--------|
| `.agent/work-plans/README.md` | Add "Project repos" section explaining the pattern is shared |
| `.github/copilot-instructions.md` | Clarify plan location is repo-relative, not workspace-only |
| `.github/instructions/work-plans.instructions.md` | Broaden scope note to include project repos |
| `.claude/skills/plan-task/SKILL.md` | Clarify step 5 path is relative to current repo |
| `.claude/skills/review-pr/SKILL.md` | Clarify line 41 reference is repo-relative |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| A change includes its consequences | Review-issue comment identified `review-pr/SKILL.md` as an additional file — included in plan |
| Workspace improvements cascade to projects | The `.agents/work-plans/` convention in project repos already works; this just documents it |
| Only what's needed | Documentation-only, no code or script changes |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0006 — Shared AGENTS.md | Watch | `AGENTS.md` itself doesn't reference work-plans; no change needed there. Changes are to framework adapters and skills. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| A framework skill (`plan-task`, `review-pr`) | Framework adapter if affected | Yes — copilot-instructions.md is included |
| Framework adapter (`.github/copilot-instructions.md`) | Other adapters if affected | Checked — Gemini CLI and onboarding docs don't reference work-plans |

## Open Questions

None.

## Estimated Scope

Single PR, single commit.
