---
issue: 651
---

# Issue #651 — Sweep health doc: make audit coverage explicit and mandatory; never render Resolved for a section not re-examined

## Issue Review
**Status**: complete
**When**: 2026-09-22 08:15 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #651
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Actions
- [ ] Name a verification approach for the coverage-as-data behavior in the plan — these are prose skill files with no automated test harness; propose replaying the 2026-09-14 vs 2026-09-21 sweep contrast from the issue body as a manual check that the shallower run would now render `Not re-examined` instead of silently omitting findings.
- [ ] Clarify in the plan which `audit-project` sections genuinely need a new coverage line (item 4) vs. which already report coverage implicitly (its SKILL.md already frames itself as producing a "coverage report").
- [ ] Distinguish the new per-check coverage line (item 2) from the sweep's existing top-line `Checks: X of 4 completed` metric in `janitor-sweep` SKILL.md, so the plan doesn't read as duplicating that existing line.

## Plan Authored
**Status**: complete
**When**: 2026-09-22 08:19 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-651/plan.md` at `0e24bdb`
**Branch**: feature/issue-651 at `0e24bdb`
**Phases**: single

### Open questions
- [ ] Whether the audit-workspace Coverage line goes in per-section tables (new column) or a separate summary table — implementation detail, either satisfies the issue.
