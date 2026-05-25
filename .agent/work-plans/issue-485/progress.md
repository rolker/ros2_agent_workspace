---
issue: 485
---

# Issue #485 — #470 phase B: triage-reviews as progress.md integrator (## Integrated Review)

## Issue Review
**Status**: complete
**When**: 2026-05-25 14:19 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**Issue**: #485
**Comment**: https://github.com/rolker/ros2_agent_workspace/issues/485#issuecomment-4536264990
**Scope verdict**: well-scoped

### Actions
- [ ] (ADR-0013 gap) Write step 3 as "filter by entry type **+ correlation key**" (issue # / plan-commit SHA / PR-or-branch head SHA per ADR-0013's correlation-key table), not just "by entry type" — cross-source confirmation is keyed by both.
- [ ] (ADR-0013 gap) Make `## External Review` predecessor-integration explicit in the reader (integrate historical entries even as new writes switch names); cover with a test.
- [ ] (consequence) If `progress_read.sh` is added, update the Script Reference table in `AGENTS.md` (Makefile target only if warranted).
- [ ] (consequence) Verify no framework adapter (`copilot-instructions.md`, `gemini-cli.instructions.md`, `AGENT_ONBOARDING.md`) describes triage-reviews' now-stale `## External Review` output name.
- [ ] (test) Add parser/helper tests up front: malformed entries, missing correlation key, `Z` vs `+00:00` offset forms, predecessor recognition.
- [ ] (open question) Decide whether ADR-0013 needs a status note once new writes are `## Integrated Review`; if touched, stay within ADR-0012's addendum carve-out — do not alter the Decision table.
- [ ] (recommendation) Use `.agent/work-plans/issue-468/progress.md`'s `## Integrated Review` as the golden-output fixture; assert the redesigned skill reproduces its shape (sources column, cross-source flagging, separate false-positive justifications).
