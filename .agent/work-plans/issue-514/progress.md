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
