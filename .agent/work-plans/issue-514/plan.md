# Plan: merge_pr.sh resolves install/ or build/ artifacts instead of src/ project repo

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/514

## Context

`merge_pr.sh` uses `find ... -maxdepth 3 -type d -name "$slug"` in two places
to locate a project repo under `layers/main/`. After a `colcon build`, colcon
creates artifact directories at `<layer>_ws/install/<pkg>` and
`<layer>_ws/build/<pkg>` — same depth and name as the source at
`<layer>_ws/src/<pkg>`. Since `-quit` returns the first match, colcon artifacts
are found before the source repo (filesystem order, install/ typically first).

This causes `git remote get-url origin` to walk up to the workspace root,
resolving `GH_REPO` to the workspace repo instead of the package's own repo,
and the subsequent `gh pr view` call fails with "no PR found."

Both find invocations are affected:
- **Line 151** (`--pr` escape hatch): `REPO_PATH` derivation
- **Line 221** (`BRANCH_REPO` derivation): used by all resolution modes when
  `REPO_SLUG` is non-empty

## Approach

1. **Scope both `find` calls to `*/src/*`** — colcon artifacts always live
   under `*/install/*` or `*/build/*`, never under `*/src/*`. Adding
   `-path '*/src/*'` to both find calls excludes artifacts entirely.

2. **Add git-repo verification after each `find` (belt-and-suspenders)** —
   after finding a candidate path, verify it is a git repo root via
   `git -C "$path" rev-parse --show-toplevel 2>/dev/null`. If the result
   does not equal `$path`, emit a clear error (not a silent wrong-path
   fallback). This defends against future layout changes that `-path`
   alone can't catch.

3. **Add a regression test to `test_merge_pr.sh`** — fabricate a fake
   `layers/main/testlayer_ws/{install,build,src}/testpkg` structure where
   `src/testpkg` is a git repo with a gitcloud (field-mode) origin. Invoke
   `merge_pr.sh --pr 1 --repo-slug testpkg` and assert the field-mode guard
   fires (proving `src/testpkg` was resolved), not any other error
   (which would indicate `install/testpkg` or `build/testpkg` was found first).

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/merge_pr.sh` | Add `-path '*/src/*'` to the two `find` calls on lines 151 and 221; add git-repo verification after each |
| `.agent/scripts/tests/test_merge_pr.sh` | Add regression test: fake artifact dirs + src git repo, assert field-mode guard fires (src resolved) |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| A change includes its consequences | Regression test ships in same PR; AGENTS.md script reference table needs no update (interface unchanged) |
| Test what breaks | Test exercises the exact failure mode from the issue (artifact dir found before src/) |
| Human control and transparency | Fix makes script behave as documented; error messages remain clear |
| Enforcement over documentation | Fix is code-level (path filter + git verify), not a warning comment |
| Only what's needed | Two targeted find changes + one test; no refactor beyond the bug |
| Improve incrementally | Single focused PR; no scope creep |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0007 (Retain Make with dependency tracking) | No | `make merge-pr` delegates to `merge_pr.sh`; Makefile target interface unchanged |
| ADR-0001 (Adopt ADRs) | No | Bug fix, not a new design decision; no ADR needed |
| ADR-0013 (progress.md vocabulary) | Yes | Plan Authored entry written to `progress.md` per spec |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Both `find` calls in `merge_pr.sh` | Regression test in `test_merge_pr.sh` | Yes |
| `merge_pr.sh` behavior | AGENTS.md script reference table | No — interface unchanged, no update needed |
| `merge_pr.sh` behavior | `.agent/WORKTREE_GUIDE.md` | No — script behavior fix, not workflow change |

## Open Questions

- [ ] No open questions — plan is review-plan-ready.

## Estimated Scope

Single PR — two targeted `find` fixes + one regression test in an existing test file.
