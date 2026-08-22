---
issue: 602
---

# Issue #602 — docker_run_agent.sh: shield worktree build artifacts from host contamination

## Issue Review
**Status**: complete
**When**: 2026-08-22 19:43 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #602
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Scope Assessment

The issue targets a single, well-identified gap in `docker_run_agent.sh`: the anonymous-volume "hole-punch" loop shields `layers/main/*_ws/{build,install,log}` but not the dispatched worktree's `*_ws` equivalents. The proposed fix is a small, targeted addition to the existing mount-assembly block. Fits in a single PR with no sub-issue splitting required.

**Right repo?** Yes — `docker_run_agent.sh` is workspace infrastructure under `.agent/scripts/`. No project-repo content involved.

**Dependencies**: None blocking. Issues referenced (#492, #532, #552, #570) are historical context only.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | OK | Fix is minimal and commented; the issue documents the mechanism and acceptance criteria clearly |
| Enforcement over documentation | OK | Existing comment stated the intent; this fix makes enforcement match the comment |
| Capture decisions, not just implementations | Watch | The `-L` symlink guard and the "only dispatch-target worktree, not all worktrees" rationale should appear as code comments; issue body is thorough but won't survive a diff |
| A change includes its consequences | Action needed | No automated test for mount-arg generation; acceptance criteria are behavioral (manual reproduction). A regression test (similar to `test_layer_sourcing.sh`) checking the new loop's output would prevent silent future breakage |
| Only what's needed | OK | ~3 extra mounts per launch (only the dispatched worktree), not 334 |
| Improve incrementally | OK | Small, targeted change to a known-bad path |
| Test what breaks | Watch | The symlink-guard logic (`[ ! -L "$ws_dir" ]`) is subtle; a unit-style test for the mount-argument builder would catch regressions without requiring a full Docker launch |
| Workspace vs. project separation | OK | Pure workspace infrastructure change |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| 0002 — Worktree isolation | Yes | Fix is directly about worktree/container boundary — aligns with the intent |
| 0003 — Project-agnostic workspace | Yes (any workspace-repo change) | Fix is generic ROS 2 infrastructure, no project coupling |
| 0005 — Layered enforcement | Watch | Fix provides runtime enforcement; adding an automated test would add a local-feedback layer |

### Consequences

Per the consequences map:
- `docker_run_agent.sh` description in `AGENTS.md` script table does not need updating (the entry description "Launch the sandboxed agent container for a worktree" is still accurate). No new entry needed.
- The host/container build-artifact contamination pattern is a non-obvious pitfall — implementation may surface a `.agent/knowledge/` note candidate (operator approval required before any edit lands).
- The image staleness check mentioned in the issue is explicitly called out as separable; a follow-up issue should be filed after this PR merges.

### Actions
- [ ] Add an automated test (akin to `test_layer_sourcing.sh`) that verifies the new worktree-path loop adds anonymous-volume mounts for real `*_ws` directories and skips symlinked ones.
- [ ] Ensure code comments in `docker_run_agent.sh` capture the symlink-guard rationale and the "only dispatched worktree, not all worktrees" design decision so it survives beyond the issue.
- [ ] File a follow-up issue for the image staleness check (launcher warning when image predates host ROS packages) after this PR merges.
