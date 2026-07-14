---
issue: 566
---

# Issue #566 — Fix docker_run_agent.sh anonymous volumes creating root-owned host dirs

## Issue Review
**Status**: complete
**When**: 2026-07-14 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #566
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Summary

`docker_run_agent.sh` mounts layer `build/install/log` dirs as anonymous volumes
without pre-creating the host mountpoints. When those dirs don't exist at container
start, Docker creates them as `root:root`, causing `EACCES` for host-side colcon
processes. The fix is one `mkdir -p` call per subdir inside the existing loop.
The issue was observed concurrently with the #559 heal rebuild.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | OK | Fix is minimal; behavior change is explicit — dirs created before mount |
| Enforcement over documentation | Watch | Optional `test_layer_sourcing.sh` hardening (root-ownership detection) would enforce the invariant; worth including if low-cost |
| Capture decisions, not just implementations | Watch | ADR-0016 Consequences describes the clean-rebuild procedure but omits the concurrent-container hazard; a short note there preserves institutional knowledge |
| A change includes its consequences | Action needed | Issue flags ADR-0016 update and optional test hardening as "Consider." Implementation must scope them in or explicitly defer with reasoning |
| Only what's needed | OK | Core fix is a single line; optional items are incremental |
| Improve incrementally | OK | Single clear PR |
| Test what breaks | Watch | No automated check for root-owned mountpoints; optional `test_layer_sourcing.sh` check would catch this in `make validate` |
| Workspace vs. project separation | OK | Fix is in workspace script `.agent/scripts/docker_run_agent.sh` |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| ADR-0016 — Runtime vs. baked layer chaining | Yes | Consequences section describes the clean rebuild (`rm -rf layers/main/*_ws/{build,install,log} && make build`) that triggered this bug; a note on concurrent-container-dispatch collision belongs there |
| ADR-0002 — Worktree isolation | Tangential | Fix supports the container-dispatch pathway used with worktrees |
| ADR-0004 — Enforcement hierarchy | Conditional | Applies if `test_layer_sourcing.sh` hardening is in scope — detection in CI/validate strengthens enforcement |

### Consequences

- `docker_run_agent.sh` change: no AGENTS.md script-table update needed (entry already exists; behavior description is unaffected at that level of detail).
- If ADR-0016 is updated: review guide ADR table title is unchanged — no cascade.
- If `test_layer_sourcing.sh` gets root-ownership detection: `make validate` surface and script-reference table in AGENTS.md already list the script; no structural update needed.

### Actions
- [ ] Add `mkdir -p "$ws_dir/$subdir"` before the anonymous-volume line in `docker_run_agent.sh` (the core fix)
- [ ] Evaluate including an ADR-0016 heal-instructions note about concurrent-container-dispatch collision (low cost, preserves context from the #559 incident)
- [ ] Evaluate whether root-ownership detection in `test_layer_sourcing.sh` (or `make build`) belongs in this PR or a separate follow-up issue

## Plan Authored
**Status**: complete
**When**: 2026-07-14 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-566/plan.md` at `e1b2182`
**Branch**: feature/issue-566 at `e1b2182`
**Phases**: single

### Open questions
- [ ] No open questions — plan is review-plan-ready.
