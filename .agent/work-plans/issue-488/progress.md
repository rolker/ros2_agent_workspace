---
issue: 488
---

# Issue #488 — Add make merge-pr: merge + clean up worktree/branches + make sync (adapt from agent_workspace)

## Plan Authored
**Status**: complete
**When**: 2026-05-25 18:57 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**Plan**: `.agent/work-plans/issue-488/plan.md` at `4634f0b`
**PR**: https://github.com/rolker/ros2_agent_workspace/pull/494 (`[PLAN]` prefix)
**Phases**: single

### Open questions
- [ ] Project-repo PR resolution UX: default workspace + worktree-narrowed scan of layer repos with `--repo-slug`/`REPO=` override (proposed), vs. requiring `REPO=` explicitly for any non-workspace PR (simpler, less magic)?
- [ ] Should `merge-pr` write a terminal `progress.md` entry, or stay mechanical (lifecycle entries owned by #481-C / the `## Implementation` marker)? Proposed: stay mechanical — confirm the boundary.
