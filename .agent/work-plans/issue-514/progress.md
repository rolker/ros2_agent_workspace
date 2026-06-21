---
issue: 514
---

# Issue #514 — merge_pr.sh resolves install/ or build/ artifacts instead of src/ project repo

## Issue Review
**Status**: complete
**When**: 2026-06-21 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #514
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Scope Assessment

The issue identifies a concrete, reproducible bug in `.agent/scripts/merge_pr.sh`:
two `find` calls use `-maxdepth 3 -type d -name "$slug"` without path-scoping,
so a built layer package's `install/<pkg>` or `build/<pkg>` artifact directory
is found before `src/<pkg>`. The fix is narrow (two locations in one script),
the root cause is clear, and the suggested approaches are ready to implement.
Fits in a single PR. The issue lives in the workspace repo (infra script).

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| A change includes its consequences | Action needed | Issue requests a regression test (build a layer pkg, assert `merge_pr.sh` resolves `src/<pkg>`). Test must ship in the same PR. |
| Test what breaks | Action needed | No test currently guards this `find` scoping. The regression test described in the issue is concrete and achievable. |
| Human control and transparency | OK | Fix makes script behave as documented; no hidden side effects. |
| Enforcement over documentation | OK | The fix is a code-level guard, not just a warning comment. |
| Only what's needed | OK | Belt-and-suspenders option (path scope + `git rev-parse` verify) is appropriate given the bug severity; not over-engineering. |
| Improve incrementally | OK | Single focused bug fix; no scope creep. |
| Workspace vs. project separation | OK | Change is in workspace infra script, correct home. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| ADR-0007 (Retain Make with dependency tracking) | Watch | `make merge-pr` delegates to `merge_pr.sh`; fix is in the underlying script, Makefile target unchanged. No ADR action needed. |
| ADR-0001 (Adopt ADRs) | OK | Bug fix doesn't introduce a new design decision requiring an ADR. |

### Consequences

- **AGENTS.md script reference table**: `merge_pr.sh` is listed; behavior fix doesn't change the interface, no update needed.
- **Regression test**: the issue explicitly calls for one — this must be included in the PR, not left as follow-up.
- No `.agent/WORKTREE_GUIDE.md` update needed (script behavior fix, not workflow change).

### Recommendations

- Use the belt-and-suspenders approach: scope `find` to `*/src/*` (option 1) AND verify the result is a git repo root via `git -C <dir> rev-parse --show-toplevel` (option 2). Both are cheap; either alone is fragile if the layout changes.
- Consider also checking `--pr <N> --repo-slug <slug>` escape-hatch path — the issue notes it hits the same `find` for `REPO_PATH`.
- Regression test: create a minimal fixture (or use an existing built layer) to assert the resolved path ends in `src/<pkg>`, not `install/<pkg>` or `build/<pkg>`.

### Actions
- [ ] Scope both `find` calls in `merge_pr.sh` to `*/src/*` and/or verify result is a git repo root.
- [ ] Include a regression test: build a layer package, then assert `merge_pr.sh` resolves `src/<pkg>` (not `install/<pkg>`).
- [ ] Verify the `--pr <N> --repo-slug <slug>` escape-hatch `REPO_PATH` derivation also uses the scoped `find`.

## Plan Authored
**Status**: complete
**When**: 2026-06-21 10:30 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-514/plan.md` at `5f4a41c`
**Branch**: feature/issue-514 at `5f4a41c`
**Phases**: single

### Open questions
- [ ] No open questions — plan is review-plan-ready.

## Plan Review
**Status**: complete
**When**: 2026-06-21 14:57 +00:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-514/plan.md` at `5f4a41c`
**PR**: PR-less (container dispatch, no GitHub auth)
**Verdict**: approve-with-suggestions

### Findings
- [ ] (suggestion) `-path '*/src/*'` is defeated by a checkout path containing a `/src/` segment (e.g. `~/src/...`) — the install/ artifact path then still matches; degrades to a false hard error via git-verify, not a wrong merge. Use anchored `-path '*_ws/src/*'`. — `plan.md:28`
- [ ] (suggestion) Scope the new git-verify error to found-but-not-git-root; preserve the legit empty→`REPO_PATH` fallback at `merge_pr.sh:222`. — `plan.md:32`
- [ ] (suggestion) Stronger test discriminator: make `install/testpkg` a github-origin git repo so buggy vs fixed resolution gives a positive/positive contrast (no reliance on filesystem ordering). — `plan.md:39`

## Implementation
**Status**: complete
**When**: 2026-06-21 00:00 +00:00
**By**: Claude Opus

**Branch**: feature/issue-514 at `e0e1477`
**Commit**: `e0e1477` (fix + regression test, atomic)

### Changes
- `.agent/scripts/merge_pr.sh` — both slug-resolution `find` calls now anchor on
  `-path '*_ws/src/*'` (the `--pr` escape-hatch `REPO_PATH` derivation, and the
  `BRANCH_REPO` derivation), excluding colcon `install/<pkg>` and `build/<pkg>`
  artifacts that share the source's depth+name. Anchored on the layer-workspace
  structure rather than a bare `*/src/*`, per the review-plan finding (a workspace
  under a `/src/` parent dir would defeat the bare form).
- Added git-repo-root verification after each `find`
  (`git -C "$cand" rev-parse --show-toplevel` must equal the candidate). Scoped
  to the **found-but-not-git-root** case only: the `--pr` path errors after its
  own non-empty `find`; the `BRANCH_REPO` path guards the check behind
  `[[ -n "$BRANCH_REPO" && ... ]]` so an EMPTY result still flows to the existing
  `REPO_PATH` fallback (no regression of the escape-hatch path).
- `.agent/scripts/tests/test_merge_pr.sh` — new regression test: fabricates
  `layers/main/testlayer_ws/{install,build,src}/testpkg`, makes `install/testpkg`
  a github-origin repo and `src/testpkg` a gitcloud (field-mode) repo, invokes
  `merge_pr.sh --pr 1 --repo-slug testpkg`, and asserts the field-mode guard
  fires (proving `src/` resolved). Positive/positive contrast — a regression to
  `install/testpkg` (github origin) would proceed past the guard. `gh` is stubbed
  to a loud failure to prove no GitHub call escapes.

### Verification
- `bash .agent/scripts/tests/test_merge_pr.sh` → **16 passed, 0 failed** (incl. the new #514 case).
- `PYTHON=python3 ./.agent/scripts/tests/run_script_tests.sh` → `test_merge_pr.sh` green; all
  pytest (51) green. One pre-existing unrelated failure in
  `test_check_commit_identity.sh` ("no propagation rejection"), an `$AGENT_NAME`/git-config
  env-gate test that touches neither changed file (confirmed pre-existing).
- `bash -n .agent/scripts/merge_pr.sh` → clean.
- `shellcheck .agent/scripts/merge_pr.sh` → only pre-existing info/style notes
  (SC2317/SC1091/SC2001/SC2015), none in the edited regions; passes the pre-commit
  shellcheck hook.

### Actions
- [x] Scope both `find` calls (anchored `*_ws/src/*`) + git-repo-root verify.
- [x] Regression test (github `install/` vs gitcloud `src/`, field-mode-guard assertion).
- [x] `--pr` escape-hatch `REPO_PATH` derivation uses the scoped `find`.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-21 15:28 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-514 at `f30034d`
**Mode**: pre-push
**Depth**: Deep (reason: destructive tool that shells out with user input; 283-line diff)
**Must-fix**: 0 | **Suggestions**: 0
**Round**: 1 | **Ship**: recommended — fix correct, empirically verified in main tree, 16/16 tests pass, no Must-fix.

### Findings
- [ ] No issues found. LGTM. Both adversarial passes (Lens A logic, Lens B
  systemic/safety) independently confirmed the anchored `-path '*_ws/src/*'`
  excludes colcon artifacts and the git-root verify fails closed before any
  merge/delete. Verified end-to-end in the real main tree: `find` resolves
  `site_ws/src/ccomjhc_project11` and `git rev-parse --show-toplevel` matches.
  Lens A's theoretical symlink/realpath mismatch does not apply (`$ROOT_DIR/layers/main`
  is a real dir; were it symlinked, the failure mode is fail-closed, not a wrong merge).
  No plan drift; consequences addressed (regression test ships in same commit).
