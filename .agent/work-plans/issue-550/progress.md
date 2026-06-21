---
issue: 550
---

# Issue #550 — dispatch_subagent.sh: container exit-contract checks progress.md at the wrong path for layer worktrees (false FAILED)

## Issue Review
**Status**: partial
**When**: 2026-06-21 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #550
**Comment**: NOT POSTED — GitHub CLI has no auth token in this environment; see review text below for the full comment to post manually.
**Scope verdict**: well-scoped

### Review Text (to be posted as comment on #550)

---

## Review

### Scope Assessment

**Well-scoped?** Yes — the bug is isolated to a single line in `dispatch_subagent.sh` (line 278):
`PROGRESS_FILE="$WORKTREE_PATH/.agent/work-plans/issue-$ISSUE/progress.md"`.
This assumes `progress.md` lives at the worktree root, which is correct for workspace worktrees but wrong for layer worktrees, where the project repo is nested at `<WORKTREE_PATH>/<layer>_ws/src/<project_repo>/`. The issue provides three real-world examples from `rqt_operator_tools-79` confirming the blast radius. A single PR touching the path-resolution logic plus a regression test is sufficient.

**Right repo?** Yes — `dispatch_subagent.sh` is workspace infrastructure (`rolker/ros2_agent_workspace`).

**Dependencies**: None open. Issue references #526 (fail-loud on ambiguous worktree) which is already merged.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | OK | Fix removes misleading false-FAILED status that makes operators think phases failed when they succeeded. Improves operator trust in orchestration output. |
| Enforcement over documentation | OK | No rules being added/removed; fixing a mechanical check. |
| Capture decisions, not just implementations | Watch | If the resolution strategy (e.g. `find`-based vs. convention-based path walk) is non-obvious, a brief inline comment in the script explaining the why is sufficient — no ADR needed for a single-script bug fix. |
| A change includes its consequences | Action needed | The existing regression test `test_dispatch_worktree_resolution.sh` creates fake layer worktrees with `.agent/work-plans` at the root (not nested), so it does not exercise the layer-nested path. A new test case — or an update to the existing fixture — must place `progress.md` at the real nested path (`<WTBASE>/issue-<slug>-<N>/<layer>_ws/src/<repo>/.agent/work-plans/issue-<N>/progress.md`) and assert `entry_count()` finds it. |
| Only what's needed | OK | The fix should be minimal: resolve the real progress.md path for layer worktrees, nothing more. |
| Improve incrementally | OK | Targeted bug fix. |
| Test what breaks | Action needed | `test_dispatch_worktree_resolution.sh` must be extended: add a layer-nested fixture and assert the pre-dispatch count reads correctly. This is the specific regression that caused the bug in the first place. |
| Workspace vs. project separation | OK | Workspace script, workspace repo. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| ADR-0002 — Worktree isolation | Yes | The fix must correctly handle both worktree types. The fix must NOT change how worktrees are created or entered — only how the dispatcher locates `progress.md` inside them. |
| ADR-0013 — progress.md vocabulary | Yes | The `entry_count()` and exit-contract read logic both depend on finding the correct `progress.md`. The fix is on the path-resolution step upstream of those reads; the ADR schema is not changing. |
| ADR-0004/0005 — Enforcement hierarchy | Watch | The exit contract is "convention-only (no enforcement)" per the script's own comment (line 8). The bug means the convention check was silently broken for layer worktrees. The fix restores the intended behavior but doesn't change the enforcement posture — that is an explicit known gap from ADR-0004/0005, not in scope here. |

### Consequences

- `.agent/scripts/dispatch_subagent.sh` path resolution changes → update inline comment if the resolution strategy is non-trivial (e.g. `find -maxdepth 5` vs. explicit `<layer>_ws/src/<repo>` walk).
- `.agent/scripts/tests/test_dispatch_worktree_resolution.sh` must be updated (or a companion test added) to cover the layer-nested `progress.md` fixture. The current test creates `$WTBASE/issue-zzztesta-$N/.agent/work-plans/issue-$N/` at root — inconsistent with real layer worktrees.
- AGENTS.md script table does not need updating (external API unchanged).

### Recommendations

1. **Dynamic path resolution via `find`**: The cleanest fix is to locate `progress.md` with a bounded `find` after the worktree is identified:
   ```bash
   PROGRESS_FILE="$(find "$WORKTREE_PATH" -maxdepth 6 \
       -path "*/.agent/work-plans/issue-$ISSUE/progress.md" \
       -print -quit 2>/dev/null)"
   # Fall back to the workspace-root convention if not found (pre-creation case)
   [ -z "$PROGRESS_FILE" ] && \
       PROGRESS_FILE="$WORKTREE_PATH/.agent/work-plans/issue-$ISSUE/progress.md"
   ```
   This works for both workspace and layer worktrees and doesn't require knowing `<layer>` or `<project_repo>`. The `--quit` flag makes it O(1) once found.

2. **Extend the regression test**: Add a test case that mirrors the real layer structure — create the progress.md nested at `<WTBASE>/issue-<slug>-<N>/ui_ws/src/fake_repo/.agent/work-plans/issue-<N>/progress.md` — and verify that the dispatcher reads the pre-dispatch count correctly (non-zero if the file has a typed entry).

3. **Document the path convention**: Add a one-line comment above `PROGRESS_FILE=` explaining that layer worktrees nest the project repo under `<layer>_ws/src/<repo>/`, so the path must be discovered rather than constructed.

---
**Authored-By**: `Claude Code Agent`
**Model**: `Claude Sonnet`

### Actions
- [ ] Update `dispatch_subagent.sh` to resolve `PROGRESS_FILE` dynamically for layer worktrees (line 278)
- [ ] Update `test_dispatch_worktree_resolution.sh` to include a layer-nested progress.md fixture and assert `entry_count()` finds it
- [ ] NOTE: GitHub comment could not be posted (no GH auth in this environment). Host should post the review text above as a comment on issue #550 before proceeding to plan-task.

## Plan Authored
**Status**: complete
**When**: 2026-06-21 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-550/plan.md` at `34ee993`
**Branch**: feature/issue-550 at `34ee993`
**Phases**: single

### Open questions
- [ ] No open questions — plan is review-plan-ready.
