# Plan: dispatch_subagent.sh: container exit-contract checks progress.md at the wrong path for layer worktrees (false FAILED)

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/550

## Context

`dispatch_subagent.sh` line 278 hard-codes the `progress.md` path as:

```bash
PROGRESS_FILE="$WORKTREE_PATH/.agent/work-plans/issue-$ISSUE/progress.md"
```

This is correct for **workspace worktrees** (where the workspace repo is at `$WORKTREE_PATH`), but wrong for **layer worktrees**, where the project repo is nested at `$WORKTREE_PATH/<layer>_ws/src/<project_repo>/`. The actual `progress.md` lives at the project-repo root, not the worktree root. As a result, `entry_count()` reads 0 pre-dispatch and 0 post-dispatch, causing a false `FAILED` exit even when the sub-agent succeeds and writes its entry correctly. Confirmed in real-world runs on `rqt_operator_tools` layer worktrees.

The regression test (`test_dispatch_worktree_resolution.sh`) creates its fake worktrees with `.agent/work-plans/` at the worktree root (the workspace convention), so it does not exercise the nested layer path and would not have caught this.

## Approach

1. **Fix `dispatch_subagent.sh` — dynamic `PROGRESS_FILE` resolution** — After `WORKTREE_PATH` is resolved (line 278), replace the hard-coded path with a `find`-based discovery that searches up to 6 levels deep:

   ```bash
   # Layer worktrees nest the project repo under <layer>_ws/src/<repo>/;
   # discover the file rather than constructing the path.
   PROGRESS_FILE="$(find "$WORKTREE_PATH" -maxdepth 6 \
       -path "*/.agent/work-plans/issue-$ISSUE/progress.md" \
       -print -quit 2>/dev/null)"
   # Fall back to the workspace-root convention (covers pre-creation case
   # and workspace worktrees where find may return empty before first write).
   [ -z "$PROGRESS_FILE" ] && \
       PROGRESS_FILE="$WORKTREE_PATH/.agent/work-plans/issue-$ISSUE/progress.md"
   ```

   Add a one-line comment above the block explaining the nesting.

2. **Extend `test_dispatch_worktree_resolution.sh` — layer-nested fixture** — Add a new test case (test #6) that:
   - Creates a fake layer worktree: `$WTBASE/issue-zzzlayer-$N/ui_ws/src/fake_repo/`
   - Places a pre-populated `progress.md` at the nested path: `.../fake_repo/.agent/work-plans/issue-$N/progress.md` with one typed entry
   - Dispatches with `--mode in-process --repo-slug zzzlayer` and asserts the script does NOT emit `FAILED` (i.e. the pre-dispatch count is read as ≥ 1, not 0)
   - Cleans up in the shared `trap` handler

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/dispatch_subagent.sh` | Replace hard-coded `PROGRESS_FILE` path (line 278) with `find`-based discovery + workspace-root fallback; add explanatory comment |
| `.agent/scripts/tests/test_dispatch_worktree_resolution.sh` | Add test #6: layer-nested `progress.md` fixture; assert `entry_count()` reads it correctly (non-zero pre-dispatch count) |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| A change includes its consequences | Test file updated alongside the fix; no other callers of `PROGRESS_FILE` exist |
| Test what breaks | New test case exercises exactly the layer-nested path that was broken |
| Only what's needed | Fix is two lines in the script; test is one new case in the existing file |
| Capture decisions, not just implementations | One-line comment explains the nesting convention |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0002 — Worktree isolation | Yes | Fix handles both workspace and layer worktree types; worktree creation itself is unchanged |
| ADR-0013 — progress.md vocabulary | Yes | Path resolution fix is upstream of the schema; no schema changes |
| ADR-0004/0005 — Enforcement hierarchy | Watch | Fix restores the intended convention check without changing enforcement posture |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `PROGRESS_FILE` resolution in `dispatch_subagent.sh` | Inline comment explaining layer nesting | Yes |
| Regression test fixtures | `trap cleanup` handler to remove the new `zzzlayer` dir | Yes |
| Script external API | AGENTS.md script table | No — external API unchanged, no update needed |

## Open Questions

- None — the fix strategy is clear from the issue and the review-issue analysis.

## Estimated Scope

Single PR — two files, minimal change.
