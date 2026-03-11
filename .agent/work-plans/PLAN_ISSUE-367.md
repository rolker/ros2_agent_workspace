# Plan: plan-task skill should create worktree before committing plan

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/367

## Context

The `plan-task` skill writes a plan file, commits, pushes, and creates a
draft PR — but never ensures the agent is in the correct worktree first.
If invoked from a different worktree, the plan gets committed to the wrong
branch. This actually happened: `/plan-task 366` from issue-364's worktree
committed the plan to `feature/issue-364`.

The fix is to restructure the skill steps so the worktree is created/entered
before the plan file is written, ensuring it lands in the correct branch
from the start.

## Approach

1. **Add a new step 4.5 "Ensure correct worktree"** between "Explore the
   codebase" (step 3) and "Generate the plan" (step 4). This step:
   - Checks if already in a worktree for the target issue (compare
     `$WORKTREE_ISSUE` or current branch against `feature/issue-<N>`)
   - If not, creates one: `worktree_create.sh --issue <N> --type workspace`
   - Enters it: `source worktree_enter.sh --issue <N>`
   - Renumber subsequent steps (old 4→5, 5→6, 6→7, 7→8)

2. **Keep steps 5-7 (commit/push/PR) as-is** — `worktree_create.sh
   --plan-file` handles PR creation but doesn't commit the plan into the
   repo. The existing commit+push+PR flow is still needed.

## Files to Change

| File | Change |
|------|--------|
| `.claude/skills/plan-task/SKILL.md` | Add worktree step between steps 3 and 4, renumber subsequent steps |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Enforcement over documentation | The skill becomes self-correcting — agents can't commit to the wrong branch even if invoked from the wrong worktree |
| A change includes its consequences | Skill content change only; no regeneration or adapter updates needed |
| Only what's needed | Minimal fix — one new step, no changes to worktree scripts |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0002 — Worktree isolation | Yes | This fix enforces ADR-0002 within the skill itself |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Framework skill content | Adapter file if affected | Not needed — content change only |

## Open Questions

None.

## Estimated Scope

Single PR, single commit.
