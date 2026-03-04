# Plan: Fix setup.bash layer sourcing in layer worktrees

**Issue**: [#349](https://github.com/rolker/ros2_agent_workspace/issues/349)
**Status**: In Progress

## Context

`setup.bash` is worktree-aware in design (lines 30-41 detect worktree paths) but the
detection never fires when called from `worktree_enter.sh`. The root cause:
`worktree_enter.sh` sources `setup.bash` via absolute path to the main tree, so
`BASH_SOURCE[0]` resolves to the main tree and `ROOT_DIR` never matches the worktree
detection pattern. Result: `LAYERS_BASE` always defaults to `layers/main/`, and
worktree-local builds are never sourced.

## Approach

1. **Have `setup.bash` respect a pre-set `ROS2_LAYERS_BASE`** — if the variable is
   already set when `setup.bash` runs, use it instead of the default. The existing
   detection logic (lines 31-41) remains as a fallback for direct invocations.

2. **Have `worktree_enter.sh` export `ROS2_LAYERS_BASE`** — before sourcing
   `setup.bash`, set it to `$WORKTREE_DIR` for layer worktrees. This is the path
   that already contains the modified layer's `install/` plus symlinks to main for
   other layers.

3. **Verify no doc updates needed** — the WORKTREE_GUIDE already says "each worktree
   keeps its own build/install artifacts" (line 108). The fix makes reality match
   the docs, so no doc changes are needed.

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/setup.bash` | Line 27: use `${ROS2_LAYERS_BASE:-$ROOT_DIR/layers/main}` instead of hardcoded default |
| `.agent/scripts/worktree_enter.sh` | Before line 270: export `ROS2_LAYERS_BASE="$WORKTREE_DIR"` for layer worktrees |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Enforcement over documentation | Fix is mechanical — makes the existing worktree isolation actually work for builds |
| A change includes its consequences | WORKTREE_GUIDE already documents the intended behavior; no doc drift to fix |
| Only what's needed | Two-line fix; no redesign of the detection mechanism |
| Improve incrementally | Fixes existing broken mechanism rather than replacing it |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0002 — Worktree isolation | Yes | Restores intended build isolation — worktree-local `install/` is sourced |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Worktree scripts | `.agent/WORKTREE_GUIDE.md`; `AGENTS.md` worktree section | Not needed — docs already describe the intended behavior this fix restores |

## Open Questions

None — the fix is straightforward and the review comment confirmed the approach.

## Estimated Scope

Single PR, two files, minimal changes.
