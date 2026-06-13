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
