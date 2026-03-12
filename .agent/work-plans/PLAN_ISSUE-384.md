# Plan: triage-reviews: add guidance for plan-first workflow PRs

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/384

## Context

The triage-reviews skill handles plan-first PRs but lacks explicit guidance
for two patterns: (1) comments on plan files that are already addressed in
implementation, and (2) plan-phase reviews that become stale after
implementation commits land. This causes agents to spend time evaluating
plan-document wording instead of focusing on implementation correctness.

## Approach

1. **Add plan-file recognition to step 5** — In step 5d (assess the comment),
   add a check: if the comment targets a `.agent/work-plans/PLAN_ISSUE-*.md`
   file, note it as a planning artifact and check whether the concern is
   addressed in the implementation files in the same PR.

2. **Add a "Plan-first workflow" subsection to Guidelines** — After the
   existing guidelines, add a section covering:
   - Plan files (`PLAN_ISSUE-*.md`) are planning artifacts, not implementation
   - Comments on plan wording are low-priority if implementation is correct
   - Plan-phase reviews (all reviews against the plan-only commit) should be
     evaluated against current implementation, not plan text

## Files to Change

| File | Change |
|------|--------|
| `.claude/skills/triage-reviews/SKILL.md` | Add plan-file check in step 5d; add plan-first workflow guideline |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Only what's needed | Two small additions to existing skill, no new files. |
| Improve incrementally | Targeted enhancement from real observation (#374). |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0006 — Shared AGENTS.md | No | Framework-specific skill, not shared instructions. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `.claude/skills/triage-reviews/SKILL.md` | Framework adapter / regenerate skills | Not needed — interface unchanged |

## Estimated Scope

Single PR, one file, two localized additions.
