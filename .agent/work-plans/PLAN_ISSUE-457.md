# Plan: worktree_create.sh — query the right repo's git-bug, gate on bridge presence

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/457

## Context

`worktree_create.sh` resolves the issue title (and state) in two stages —
git-bug first, `gh` as fallback. The git-bug call at line 602 is hardcoded
to query `$ROOT_DIR` (the workspace) regardless of `WORKTREE_TYPE`:

```bash
_BUG_TITLE=$(gitbug_lookup "$ROOT_DIR" "$ISSUE_NUM" title 2>/dev/null || echo "")
```

For layer-type worktrees, the issue lives in a project repo, not the
workspace — so this returns the workspace repo's issue with the same
number when one exists, populating both `ISSUE_TITLE` (line 605) and
`ISSUE_STATE` (lines 606–608) from the wrong source. The `gh` fallback at
line 632 only engages when both fields are empty, so once git-bug returns
*anything* the wrong-repo data sticks.

The actual git work (branch, push, PR target) is unaffected — those paths
correctly use `GH_REPO_SLUG`. The defect is in the human-facing metadata
display only.

**Forward-compat constraint**: project repos are not currently git-bug-bridged,
but they will be eventually. The fix must point `gitbug_lookup` at the
*right* repo for the worktree type (not always the workspace) and must
fall through to `gh` based on whether *that* repo is bridged — not based on
whether it happens to be the workspace.

## Approach

1. **Resolve the git-bug target dir per worktree type.** Add a
   `BUG_QUERY_DIR` variable that points at the repo whose git-bug cache
   should be queried for this issue:
   - `workspace` and `skill` types → `$ROOT_DIR`
   - `layer` type with `TARGET_PACKAGES` → the first package's repo dir
     (`$ROOT_DIR/layers/main/${TARGET_LAYER}_ws/src/${FIRST_PKG}`, the
     same path the slug-resolution block already computes at lines 533
     and 577)

2. **Gate the `gitbug_lookup` call on bridge presence.** Use the existing
   `gitbug_has_bridge "$BUG_QUERY_DIR"` helper from `gitbug_helpers.sh`.
   If the target repo has no bridge, skip git-bug entirely and let the
   `gh` fallback handle it. This is forward-compatible: when project
   repos start getting bridged, the same code path picks them up
   automatically.

3. **Pass `BUG_QUERY_DIR` to `gitbug_lookup`** instead of the hardcoded
   `$ROOT_DIR`. Both the title call (line 602) and the status call
   (line 606) need this change.

4. **Add a short comment** at the top of the gating block explaining why
   the bridge check matters (collision scenario from #457). Per Option C,
   the constraint isn't being recorded in an ADR — the script comment +
   regression test + PR description carry the breadcrumb.

5. **Regression test.** Add a focused test to
   `.agent/scripts/tests/test_worktree_create.sh` covering:
   - Collision scenario: layer-type worktree, project repo unbridged,
     stub `gitbug_lookup` to return a wrong-repo title — assert the gh
     fallback path runs and the right title is reported.
   - Forward-compat scenario: layer-type worktree, project repo *is*
     bridged (mocked) — assert `gitbug_lookup` is called against the
     project repo's dir, not the workspace.

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/worktree_create.sh` | Add `BUG_QUERY_DIR` resolution; gate `gitbug_lookup` on `gitbug_has_bridge`; pass `BUG_QUERY_DIR` to `gitbug_lookup` for both title and status; add short explanatory comment. |
| `.agent/scripts/tests/test_worktree_create.sh` | Add collision-scenario regression test and forward-compat (bridged-project-repo) test. |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Banner stops misleading agents about which task they're on — direct improvement to a transparency surface. |
| Enforcement over documentation | The doc-only "verify issue matches before first commit" rule survives this fix; the supporting tooling no longer lies. Mechanical enforcement of the verification rule itself is tracked separately (out of scope). |
| Only what's needed | One new variable + one bridge check + correct argument to existing helpers; no new abstractions. Reuses `gitbug_has_bridge` which already exists. |
| Test what breaks | Two regression tests: one for the current collision scenario, one for the forward-compat case where project repos become bridged. Catches regressions in both directions. |
| A change includes its consequences | No doc updates required (no signature change, no ADR touch by design — see Option C decision below). PR description carries the architectural context. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0002 — Worktree isolation | Yes | Decision unchanged; only the metadata-display path of an existing enforcement script. |
| 0010 — git-bug for local issue tracking | Yes (no change required) | The "lookup priority" rule is preserved as-is. The fix queries the *right* repo and falls through to `gh` when no bridge is present — both consistent with the ADR's existing language about graceful degradation. ADR text not modified (Option C, decided in planning). |
| 0004 — Enforcement hierarchy | Watch (deferred) | The doc-only "verify issue matches" rule is a separate enforcement-layer gap. Out of scope here; see Open Questions for follow-up. |

**ADR change decision (Option C)**: The constraint *"git-bug lookups must
target the right repo and respect bridge presence"* lives in the script
code (with an explanatory comment), the regression tests, and the PR
description. No ADR addendum, no superseding ADR. Rationale: this is an
implementation invariant, not a workspace-level architectural decision.
The expectation that project repos may eventually be bridged is exactly
why the gate is bridge-presence-based rather than workspace-only —
the architecture already accommodates this without an ADR change.

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `.agent/scripts/worktree_create.sh` | AGENTS.md script reference table; Makefile target | Not needed — invocation signature and Makefile integration unchanged. |
| Worktree scripts | `.agent/WORKTREE_GUIDE.md`; AGENTS.md worktree section | Not needed — no behavior change visible to script users; only the displayed title becomes correct. |

## Open Questions

- **File a follow-up issue for the ADR-0004 enforcement-hierarchy gap?**
  The "verify issue matches before first commit" rule in AGENTS.md is
  currently doc-only. A follow-up could propose a mechanical check
  (e.g., a one-shot helper invoked at first-commit time). Open it now
  or after this PR lands?

## Estimated Scope

Single PR. Changes are localized to one script and one test file. No
ADR touches, no doc updates. Estimate: ~30 lines of script change
(`BUG_QUERY_DIR` resolution + gating + helper call updates), ~50–60
lines of test (two scenarios with mocking).
