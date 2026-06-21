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

## Mechanism (verified against the code)

`PROGRESS_FILE` is resolved **once** at line 278, then used by `entry_count()`
for both the PRE count (line 369) and the POST count (line 402), and again for
the `LAST_ENTRY` display (line 420). `entry_count()` returns **0 for a missing
file** (line 357). The exit contract passes iff `POST_COUNT > PRE_COUNT`.

Two consequences that shape the fix:
- **Depth.** Relative to `WORKTREE_PATH`, a layer worktree's `progress.md` is at
  depth **7** (`<layer>_ws`/`src`/`<repo>`/`.agent`/`work-plans`/`issue-N`/`progress.md`).
  `find -maxdepth 6` never reaches it (confirmed: 6 → empty, 7 → found). The
  original plan's `-maxdepth 6` would fall through to the (nonexistent)
  workspace-root path and the false FAILED would persist. **Must be `-maxdepth 7`.**
- **Pre-creation (first phase).** On the *first* phase (`review-issue`) the file
  does **not exist pre-dispatch**, so a find-on-the-file cannot discover the
  nesting yet. A one-time resolve at line 278 therefore can't point POST at the
  nested file. The fix must **re-resolve on each `entry_count()` call**: PRE finds
  nothing → fallback root → absent → 0; POST finds the freshly-written nested
  file → 1; `0 → 1` passes. (This case the review-plan did not flag; it matters
  for every first-phase dispatch into a layer worktree.)

## Approach

1. **Add a `resolve_progress_file()` helper in `dispatch_subagent.sh`** that
   discovers the real `progress.md` for the worktree, with a root fallback:

   ```bash
   # Layer worktrees nest the project repo at <layer>_ws/src/<repo>/, so the
   # progress.md is up to 7 levels below the worktree root (workspace worktrees
   # keep it at depth 4). Discover it rather than constructing one fixed path.
   resolve_progress_file() {
       local wt="$1" issue="$2" f
       f="$(find "$wt" -maxdepth 7 -type f \
           -path "*/.agent/work-plans/issue-$issue/progress.md" \
           -print -quit 2>/dev/null)"
       if [ -n "$f" ]; then printf '%s\n' "$f"
       else printf '%s\n' "$wt/.agent/work-plans/issue-$issue/progress.md"; fi
   }
   ```

2. **Re-resolve per call.** Have `entry_count()` call
   `resolve_progress_file "$WORKTREE_PATH" "$ISSUE"` at the top of each
   invocation (replacing the global `$PROGRESS_FILE` read), and resolve the same
   way for the `LAST_ENTRY` display. Re-resolution is what fixes the first-phase
   pre-creation case. (`find -maxdepth 7` is depth-bounded and dispatch is
   infrequent, so the cost is negligible.)

3. **Make the resolver unit-testable.** Guard the script's main body so that
   `source`-ing it defines the functions **without** running the dispatch (e.g.
   `if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then …main… fi`, or the least
   invasive equivalent given the current top-down structure). The script's
   executable behavior must be unchanged when run directly.

4. **Rewrite the regression test to target the resolver directly**
   (`test_dispatch_worktree_resolution.sh`). The old design ran `--mode
   in-process`, which exits at `:339` *before* `PROGRESS_FILE` is ever read
   (it is consumed only in container mode at `:357-358`/`:420`) — so its
   "does not emit FAILED" assertion was vacuously true for both broken and fixed
   code. Instead, `source` the script and assert on `resolve_progress_file`:
   - **layer-nested fixture at depth 7** (`$WTBASE/issue-zzzlayer-$N/ui_ws/src/fake_repo/.agent/work-plans/issue-$N/progress.md`) → resolver returns the **nested** path (guards the maxdepth off-by-one).
   - **workspace fixture** (file at `$wt/.agent/work-plans/issue-$N/progress.md`) → resolver returns the **root** path.
   - **absent file** (repo dir exists, no progress.md yet) → resolver returns the **root fallback** path (the pre-creation contract: PRE count reads absent → 0).
   - Clean up the new fixtures in the shared `trap` handler.

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/dispatch_subagent.sh` | Add `resolve_progress_file()` (find `-maxdepth 7` + root fallback); call it per-invocation inside `entry_count()` and for `LAST_ENTRY` instead of the one-time line-278 `PROGRESS_FILE`; source-guard the main body so the helper is testable. Add explanatory comment about the layer nesting. |
| `.agent/scripts/tests/test_dispatch_worktree_resolution.sh` | Replace the vacuous in-process test with direct `resolve_progress_file` assertions: depth-7 layer fixture → nested path; workspace fixture → root; absent-file → root fallback. Extend the `trap` cleanup. |

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
| `progress.md` resolution in `dispatch_subagent.sh` | Inline comment explaining layer nesting + per-call re-resolve rationale | Yes |
| Source-guard the main body | Confirm direct-execution behavior is byte-for-byte unchanged (existing `test_dispatch_subagent.sh` suite must still pass) | Yes |
| Regression test fixtures | `trap cleanup` handler to remove the new `zzzlayer` dir | Yes |
| Script external API | AGENTS.md script table | No — external API unchanged, no update needed |

## Open Questions

- None. Two review-plan must-fixes resolved (maxdepth 6 → 7; vacuous in-process
  test → direct resolver assertions), plus the first-phase pre-creation case
  handled by re-resolving per `entry_count()` call.

## Estimated Scope

Single PR — two files, minimal change.
