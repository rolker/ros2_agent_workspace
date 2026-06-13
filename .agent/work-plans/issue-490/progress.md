---
issue: 490
---

# Issue #490 — dispatch_subagent.sh + per-skill handoff boilerplate

## Plan Authored
**Status**: complete
**When**: 2026-06-13 17:14 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Plan**: `.agent/work-plans/issue-490/plan.md` at `24896d3`
**PR**: https://github.com/rolker/ros2_agent_workspace/pull/519 (`[PLAN]` prefix)
**Phases**: single (Scope A + B + E in one PR; #493 deferred, gated on #492)

### Open questions
- [ ] #493 sequencing: #490 adds the progress.md exit contract but leaves push_gateway in place; deletion is #493, gated on #492 — confirm the split.
- [ ] Kickoff output format: stream-json vs json vs text for the host to consume (leaning stream-json).
- [ ] PR split: dispatch+headless (A) and skill handoff blocks (B) as one PR or two (leaning one).

## Plan Review
**Status**: complete
**When**: 2026-06-13 17:49 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context)) — fresh-context sub-agent dispatch (independent of plan-author context; same agent identity)

**Plan**: `.agent/work-plans/issue-490/plan.md` at `4290908` (findings addressed)
**PR**: https://github.com/rolker/ros2_agent_workspace/pull/519
**Verdict**: changes-requested (2 must-fix + 3 suggestions; all folded into plan at `4290908`)

### Findings
- [x] (must-fix) Exit-contract under-specified: crash-before-write reads stale entry; progress_read.py has no tail selector; could match wrong skill's entry → gate on exit status + require newer entry + key on expected type — plan Approach step 3
- [x] (must-fix) 3 framework adapters not in Files-to-Change (handoff is Claude-Agent-specific) — now listed
- [x] (suggestion) Container-mode must forward AGENT_NAME/EMAIL for entrypoint --detect identity — plan Approach step 2
- [x] (suggestion) Layer-worktree container check is manual, not CI — clarified in plan Approach step 5
- [x] (suggestion) State no Makefile target for dispatch_subagent.sh — added to Consequences
