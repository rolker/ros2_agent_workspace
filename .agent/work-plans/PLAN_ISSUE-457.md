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
`ISSUE_STATE` (lines 606–608) from the wrong source. The `gh` fallback
at line 632 engages only when at least one of those fields is empty
(`[ -z "$ISSUE_TITLE" ] || [ -z "$ISSUE_STATE" ]`); in the collision
case git-bug fills both fields with wrong-repo data, so neither is
empty and the fallback never runs.

The same defect exists in `worktree_enter.sh` (line 243): its
`gitbug_lookup "$ROOT_DIR" ...` call is hardcoded to the workspace,
and its `gh` fallback (lines 257–268) explicitly forces
`--repo "$_WS_SLUG"` so even when git-bug isn't bridged the layer
worktree's "Title:" line still shows workspace data. This PR fixes
both scripts.

The actual git work (branch, push, PR target) is unaffected — those paths
correctly use `GH_REPO_SLUG`. The defect is in the human-facing metadata
display only.

**Forward-compat constraint**: project repos are not currently git-bug-bridged,
but they will be eventually. The fix must point `gitbug_lookup` at the
*right* repo for the worktree type (not always the workspace) and must
fall through to `gh` based on whether *that* repo is bridged — not based on
whether it happens to be the workspace.

## Approach

1. **Resolve the git-bug target dir per worktree type, in both scripts.**
   Add a `BUG_QUERY_DIR` variable that points at the repo whose git-bug
   cache should be queried for this issue:
   - `workspace` (and `skill`, in `worktree_create.sh`) → `$ROOT_DIR`
   - `layer` worktrees → the first package's repo dir
     - In `worktree_create.sh`: derived from `--packages` (the same path
       the slug-resolution block already computes at lines 533 and 577).
     - In `worktree_enter.sh`: derived from the worktree itself via the
       new `wt_layer_pkg_dir` helper, which iterates `<wt>/*_ws/src/*`
       and returns the first inner git worktree.

2. **Gate the `gitbug_lookup` call on bridge presence.** Use the existing
   `gitbug_has_bridge "$BUG_QUERY_DIR"` helper from `gitbug_helpers.sh`.
   If the target repo has no bridge, skip git-bug entirely and let the
   `gh` fallback handle it. This is forward-compatible: when project
   repos start getting bridged, the same code path picks them up
   automatically.

3. **Pass `BUG_QUERY_DIR` to `gitbug_lookup`** instead of the hardcoded
   `$ROOT_DIR`. Covers both title and status fields.

4. **Fix the gh fallback in `worktree_enter.sh`.** The pre-fix code first
   tried `gh issue view` without `--repo` (uses cwd's git context), then
   on failure forced `--repo "$_WS_SLUG"` to the workspace — wrong for
   layer worktrees. Replace with: derive the right slug from
   `BUG_QUERY_DIR`'s origin once, pass it as `--repo` from the start. No
   workspace-fallback retry. (`worktree_create.sh`'s `gh` fallback was
   already correct — it uses `--repo "$GH_REPO_SLUG"` which is resolved
   per worktree type at lines 549/580/584.)

5. **Add a short comment** at the top of each gating block explaining
   why the bridge check matters (collision scenario from #457). Per
   Option C, the constraint isn't being recorded in an ADR — the script
   comments + regression tests + PR description carry the breadcrumb.

6. **Regression tests.** Add focused tests to
   `.agent/scripts/tests/test_worktree_create.sh` covering:
   - Collision scenario: layer-type worktree, project repo unbridged,
     fake `gitbug_lookup` returns a wrong-repo title — assert the gh
     fallback path runs and the right title is reported.
   - Forward-compat scenario: layer-type worktree, project repo *is*
     bridged (mocked) — assert `gitbug_lookup` is queried against the
     project repo's dir, not the workspace.
   - Sanity scenario: workspace-type worktree, workspace bridged —
     assert the original happy path still uses workspace git-bug data.
   - Unit tests for the new `wt_layer_pkg_dir` helper: finds the first
     inner git worktree, skips plain dirs and symlinks, fails when no
     inner worktree exists, skips symlinked layer dirs.

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/worktree_create.sh` | Add `BUG_QUERY_DIR` resolution; gate `gitbug_lookup` on `gitbug_has_bridge`; pass `BUG_QUERY_DIR` to `gitbug_lookup` for both title and status; add short explanatory comment. |
| `.agent/scripts/worktree_enter.sh` | Same fix pattern: resolve `_BUG_QUERY_DIR` per worktree type (uses new `wt_layer_pkg_dir` helper for layer worktrees), gate `gitbug_lookup` on `gitbug_has_bridge`, derive the right GitHub slug for `gh issue view --repo` (drop the workspace-fallback retry that was the gh-side bug). |
| `.agent/scripts/_worktree_helpers.sh` | Add `wt_layer_pkg_dir` helper — finds the first inner package git worktree in a layer worktree dir. Mirrors `wt_layer_branch`'s iteration pattern. Also moves `extract_gh_slug` here (from `worktree_create.sh`) and hardens it: now correctly handles HTTPS, SCP-form, SSH URL, and SSH-over-443 (`ssh.github.com:443`) remotes, and rejects substring/lookalike hosts (`mygithub.com`, `gist.github.com`, etc.). Both worktree scripts share this implementation. |
| `.agent/scripts/tests/test_worktree_create.sh` | Add three regression tests (collision, forward-compat, workspace sanity) for `worktree_create.sh` plus a fake-helpers installer; add unit tests for new `wt_layer_pkg_dir` helper (alongside existing `find_worktree_by_skill` tests, matching the file's existing convention of co-locating helper tests); also fix the pre-existing missing-manifest gap in `setup_mock_workspace` / `setup_mock_layer_workspace` (without it, no script-invoking test could run — see Implementation Notes). |

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

None outstanding. (Decisions taken during plan iteration: Option C for
the ADR question — no ADR touch — and skip the ADR-0004
enforcement-hierarchy follow-up issue. Decision taken during review
triage: fix `worktree_enter.sh` in this same PR rather than splitting
to a follow-up issue.)

## Estimated Scope

Single PR. Changes touch two scripts (`worktree_create.sh`,
`worktree_enter.sh`), one shared helpers file (`_worktree_helpers.sh`),
and one test file. No ADR touches, no doc updates.

## Implementation Notes

- **Test infrastructure fix**: `setup_mock_workspace` and
  `setup_mock_layer_workspace` did not create
  `configs/manifest/layers.txt`, which the script now requires (early
  fatal exit). Every script-invoking test in the file (16/20) was
  silently failing on `main`. Fixed by adding two lines to each setup
  function. Without this, the new #457 regression tests could not run.
  This is in scope under the Quality Standard's "fix it completely"
  guidance — the pre-existing test gap actively blocks the new tests
  this PR adds, and the fix is a one-line-per-function change.
- **Third regression test added**: a workspace-type sanity test was
  added beyond the two listed in the approach. Without it, the
  bridge-gating change had no positive assertion that the workspace
  happy path (the original ADR-0010 contract) still works — only
  negative assertions that wrong data doesn't leak. The added test
  costs ~15 lines and protects against silently disabling git-bug for
  the workspace.
- **`worktree_enter.sh` brought into scope after triage**: Copilot's
  bot review flagged the same defect class in `worktree_enter.sh`
  (line 243's hardcoded `gitbug_lookup "$ROOT_DIR" ...` plus a worse
  gh-side bug at line 265 that explicitly forces `--repo "$_WS_SLUG"`
  for the workspace, so even unbridged layer worktrees got workspace
  data). User decision: fix in same PR rather than splitting — fixing
  half would leave the post-create banner correct but the
  worktree_enter.sh "Title:" line still misleading. Required adding
  the `wt_layer_pkg_dir` helper to `_worktree_helpers.sh` so
  `worktree_enter.sh` can find the package's repo dir from the
  worktree at enter-time (where `--packages` is no longer in scope).
- **`extract_gh_slug` boundary tightened after round-4 review**:
  Round-4 Copilot review caught that the initial hardening's boundary
  class `(^|[@/])` still allowed `github.com` to appear inside a URL
  path (e.g. `https://example.com/github.com/owner/repo.git` would
  match the `/` before `github.com` and produce a false-positive
  slug). Tightened the boundary to `(^|@|://)` so the host must be at
  a true URL host position (start, after `@` auth section, or after
  `://` protocol delimiter). Added regression cases for the
  path-component scenarios.
- **`extract_gh_slug` hardening brought into scope after round-3
  review**: Copilot pointed out two latent bugs in the existing
  `extract_gh_slug` (defined in `worktree_create.sh`): (1) substring
  host matching — `mygithub.com` was incorrectly accepted; (2) the
  `sed`-based parser produced an invalid 3-segment slug for
  `ssh://git@ssh.github.com:443/OWNER/REPO.git` (an officially
  supported form per `AGENTS.md` and `field_mode.sh`'s allowlist),
  causing validation to fail and leaving `GH_REPO_SLUG` empty. With
  the new bridge gating, an empty slug means `gh issue view` runs
  without `--repo`, which can fall back to the workspace repo via
  cwd inference — re-introducing the wrong-repo bug. Fix: move
  `extract_gh_slug` to `_worktree_helpers.sh` (single source of truth
  for both worktree scripts) and replace the `sed` chain with a
  bash regex anchored on URL boundaries that handles all officially
  supported forms and rejects lookalike hosts. Quality Standard's
  "fix it completely" applies: this is the same code path being
  changed, the SSH-over-443 form is a documented workspace
  capability, and the test mocks already exercise non-GitHub URLs
  (so the regression risk is observable). 10 new unit tests cover
  HTTPS / SCP / SSH / SSH-over-443, dotted repo names,
  substring-host rejection, and malformed input.
