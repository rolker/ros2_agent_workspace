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

1. **Scope both `find` calls to the layer-workspace `src/` with an *anchored*
   path: `-path '*_ws/src/*'`** — colcon artifacts live under
   `<layer>_ws/install/*` / `<layer>_ws/build/*`, the source under
   `<layer>_ws/src/*`. **Not** a bare `-path '*/src/*'`: if the whole workspace
   is checked out under a path that itself contains a `/src/` segment (e.g.
   `~/src/project11/…`), then `…/install/<pkg>` *also* matches `*/src/*` and the
   artifact dir is not excluded (review-plan finding). Anchoring on `_ws/src/`
   keys on the layer-workspace structure, which the artifact paths never share.

2. **Add git-repo verification after each `find` (belt-and-suspenders)** —
   after finding a candidate path, verify it is a git repo root via
   `git -C "$path" rev-parse --show-toplevel 2>/dev/null`. **Scope the new error
   to the *found-but-not-a-git-root* case only** — an empty `find` result is a
   legitimate signal that must keep flowing to the existing `REPO_PATH` fallback
   at `merge_pr.sh:222` (do not convert an empty result into a hard error, or the
   escape-hatch path regresses). Emit the clear error only when a non-empty
   candidate fails the git-root check.

3. **Add a regression test to `test_merge_pr.sh`** — fabricate a fake
   `layers/main/testlayer_ws/{install,build,src}/testpkg` structure. Make
   **`install/testpkg` a *github*-origin git repo** and **`src/testpkg` a
   *gitcloud* (field-mode) git repo**, so buggy vs fixed resolution produces a
   **positive/positive contrast** rather than relying on filesystem ordering or
   an error: invoke `merge_pr.sh --pr 1 --repo-slug testpkg` and assert the
   **field-mode guard fires** (proving `src/testpkg` was resolved). If resolution
   regressed to `install/testpkg`, the github-origin path would instead proceed
   past the field-mode guard — a clearly distinguishable outcome.

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/merge_pr.sh` | Add anchored `-path '*_ws/src/*'` to the two `find` calls (L151, L221); add git-repo verification scoped to the found-but-not-git-root case (preserve the empty→`REPO_PATH` fallback at L222) |
| `.agent/scripts/tests/test_merge_pr.sh` | Add regression test: github-origin `install/testpkg` + gitcloud-origin `src/testpkg`, assert the field-mode guard fires (positive/positive contrast, no filesystem-order reliance) |

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

- [x] None. Review-plan's 3 suggestions folded in: anchored `*_ws/src/*` (not
  bare `*/src/*`); git-verify error scoped to found-but-not-git-root (empty
  fallback preserved); contrast-based test (github `install/` vs gitcloud `src/`).

## Estimated Scope

Single PR — two targeted `find` fixes + one regression test in an existing test file.
