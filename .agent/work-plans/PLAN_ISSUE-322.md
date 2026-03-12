# Plan: gather-project-knowledge: extend validation to cover summary statistics

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/322

## Context

The `gather-project-knowledge` skill (`.claude/skills/gather-project-knowledge/SKILL.md`)
has a validation step (step 4) that checks package names in profiles against `package.xml`
`<name>` elements. However, summary statistics in `workspace_overview.md` (repo counts,
package counts per layer) are not validated, leading to incorrect numbers in the second
run (see [rolker/unh_marine_autonomy#96](https://github.com/rolker/unh_marine_autonomy/pull/96)). The root cause is the same as #320: values were copied
from intermediate agent output instead of being derived from the authoritative inventory
table.

## Approach

1. **Add count-derivation instruction to step 3** — In the "Generate summaries" section,
   under `workspace_overview.md`, add an explicit instruction that all numeric counts
   (repos per layer, packages per layer, totals) must be computed by counting entries in
   the structured inventory table from step 2. Never use pre-computed numbers from
   intermediate outputs or prior runs.

2. **Extend step 4 validation to cover counts** — After the existing package-name
   validation, add a new validation sub-step: for each layer in the `workspace_overview.md`
   Layer Structure table, count repos and packages from the Package Inventory table and
   compare against the claimed numbers. If any mismatch, fix the overview before
   proceeding.

## Files to Change

| File | Change |
|------|--------|
| `.claude/skills/gather-project-knowledge/SKILL.md` | Add count-derivation instruction to step 3 (`workspace_overview.md` section) |
| `.claude/skills/gather-project-knowledge/SKILL.md` | Extend step 4 validation with count-checking sub-step |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Enforcement over documentation | This adds a validation step (enforcement at skill runtime), not just a guideline |
| A change includes its consequences | The consequences map entry for skill changes requires checking the framework adapter — no adapter change needed here since the skill interface is unchanged |
| Only what's needed | Two targeted additions to an existing file — minimal change |
| Improve incrementally | Builds on the #320 fix with the same pattern |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0002 — Worktree isolation | Yes | Using workspace worktree `feature/issue-322` |
| Others | No | No new design decisions, packages, or tooling changes |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| A framework skill (`.claude/skills/`) | Framework adapter if interface changes | N/A — no interface change |
| A framework skill (`.claude/skills/`) | Regenerate skills if needed | N/A — no new/removed skills |

## Open Questions

None — the issue is well-specified and the fix is straightforward.

## Estimated Scope

Single PR, single file changed.
