---
issue: 659
---

# Issue #659 — rosdep_local_sources.sh: discover rosdep.yaml in layer worktrees, not only layers/main (first-use gap from #654)

## Issue Review
**Status**: complete
**When**: 2026-09-23 (see commit timestamp for exact time)
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #659
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Actions
- [ ] Recommendation: update the `rosdep_local_sources.sh` entry in `AGENTS.md`'s
      Script Reference table in the same PR — it currently documents the
      generator as globbing only `layers/main/*_ws/src/*/rosdep.yaml`, which
      would become inaccurate once the worktree glob is added (Consequences
      Map: "A script in `.agent/scripts/`" → "Script reference table in
      `AGENTS.md`").
- [ ] Recommendation: note in the PR (not necessarily fix) that
      `stage_rosdep_manifests.sh` (agent-image bake) and `ci_local.sh` use
      their own, narrower rosdep.yaml discovery (per-repo root only, or
      package.xml staging from `layers/main`) and are correctly **out of
      scope** here — they operate on a repo already resolved to a specific
      checkout, not on a dev-host-wide glob — but a reviewer should confirm
      that reasoning rather than assume it silently.

## Plan Authored
**Status**: complete
**When**: 2026-09-23 12:26 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-659/plan.md` at `fbadffa`
**Branch**: feature/issue-659 at `fbadffa`
**Phases**: single

### Open questions
- [ ] No open questions — plan is review-plan-ready. (Two implementation-level
      calls the plan itself made, flagged for review-plan to confirm rather
      than re-decided as policy: staleness-check glob extended to match the
      generator (step 5); shape-rejection exit 4 takes precedence over the
      new conflict exit 6 when both occur in one run (step 3).)
