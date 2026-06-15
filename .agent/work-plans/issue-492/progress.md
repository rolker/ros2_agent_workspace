---
issue: 492
---

# Issue #492 — /run-issue host orchestration skill

## Issue Review
**Status**: complete
**When**: 2026-06-15 14:05 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Issue**: #492
**Comment**: https://github.com/rolker/ros2_agent_workspace/issues/492#issuecomment-4711263199
**Scope verdict**: well-scoped (design-heavy)

### Actions
- [ ] (plan-task, primary) Resolve plan-task early-PR vs local-first PR-at-end: recommend a plan-task opt-in flag (PR-at-end orchestrated default); decide in-PR vs stacked follow-up.
- [ ] (plan-task) Specify the phase-transition decision table (last entry type + verdict → next dispatch/checkpoint), incl. `## Implementation` preceding-entry disambiguation (shared routing key).
- [ ] (plan-task) Define field-mode end-of-pipeline behavior (push, no PR, no Copilot for gitcloud) per ADR-0011.
- [ ] (plan-task) Enumerate checkpoints precisely (post Issue Review iff open Qs; always post Plan Review; post each Integrated Review w/ findings; before any push/PR/merge).
- [ ] (consequences) Adapter skill lists + skill_workflows.md.
- [ ] Note overlap with the pending #491 de-dup follow-up so findings aren't double-handled.

## Plan Authored
**Status**: complete
**When**: 2026-06-15 14:20 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Plan**: `.agent/work-plans/issue-492/plan.md` at `HEAD`
**PR**: https://github.com/rolker/ros2_agent_workspace/pull/525 (`[PLAN]` prefix)
**Phases**: single (possible stacked plan-task tweak — open question 1)

### Open questions
- [x] plan-task PR behavior → **flip default to no-early-PR** (publish-early = opt-in `--draft-pr`); ships **in this PR**. Confirmed 2026-06-15.
- [x] Copilot at publish → **off by default**, `--copilot` opt-in at the publish checkpoint. Confirmed 2026-06-15.
- [ ] AGENTS.md "Plan-first workflow" wording tweak — confirm exact change with user before editing (instruction file).
