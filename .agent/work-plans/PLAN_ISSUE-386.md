# Plan: worktree_remove.sh flags generated convenience scripts as unexpected content

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/386

## Context

`worktree_create.sh` generates convenience scripts (`build.sh`, `test.sh`,
`colcon/`) inside each layer workspace directory (#356). `worktree_remove.sh`
was not updated to recognize these, so it flags them as unexpected content and
requires `--force`.

Additionally, during planning we discovered a regression from #315: the
PR-vs-issue guard uses `gh pr view --json number --jq '.number'`, which
unreliably echoes the argument even for non-PR issue numbers, blocking
worktree creation for valid issues. This must be fixed in the same PR.

## Approach

1. **Update `KNOWN_INFRA` in `worktree_remove.sh`** (line 323) — Add the
   generated file names to the pattern. The complete set generated per-layer
   is: `build.sh`, `test.sh`, `colcon` (directory). `setup.bash` is at the
   worktree root, outside the layer workspace scan, so it's not affected.

   ```bash
   KNOWN_INFRA="^(src|build|install|log|build\.sh|test\.sh|colcon)$"
   ```

   Add a comment noting the coupling with `worktree_create.sh`.

2. **Fix PR-vs-issue guard in `worktree_create.sh`** (lines 569-571) —
   Change `--json number --jq '.number'` to `--json state --jq '.state'`.
   The `state` field correctly returns empty/error for non-PR issues while
   `number` unreliably echoes the argument.

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/worktree_remove.sh` | Update `KNOWN_INFRA` regex (line 323) |
| `.agent/scripts/worktree_create.sh` | Fix PR guard field from `number` to `state` (lines 569-571) |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| A change includes its consequences | This issue IS the missing consequence from #356. The PR guard fix is a consequence of #315. Both addressed here. |
| Enforcement over documentation | `KNOWN_INFRA` is enforcement — keeps worktree removal safe. PR guard is enforcement — prevents wrong issue numbers. |
| Only what's needed | Two targeted fixes, no new abstractions. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0002 — Worktree isolation | Yes | Fixes worktree lifecycle (clean create and remove). |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `.agent/scripts/worktree_remove.sh` | `.agent/WORKTREE_GUIDE.md` | Not needed — no interface change |
| `.agent/scripts/worktree_create.sh` | Already-merged #315 PR | N/A — this is the bugfix |

## Estimated Scope

Single PR, two files, two small edits.
