---
issue: 533
---

# Issue #533 — start-deployment: issue-less start + offer-and-confirm + background propagation

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-21 00:02 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-533 at `a9f0634`
**Mode**: pre-push
**Depth**: Standard (reason: `.claude/skills/*/SKILL.md` is a Governance override-trigger file)
**Must-fix**: 1 | **Suggestions**: 3
**Round**: 1 | **Ship**: continue — lone must-fix is a genuine design gap (reconciliation mechanism asserted but unimplemented); re-read after it's addressed

### Findings
- [ ] (must-fix) Issue-less reconciliation asserted but unimplemented — wrap-up discovers only by `gh issue list`, bails on no issue; nothing scans logs for `pending`. Issue-less deployment can be silently never tracked/closed. Add backfill step or flag as manual + follow-up issue — `.claude/skills/start-deployment/SKILL.md:241,322-326`
- [ ] (suggestion) Issue-less path re-enters "4b's field branch" whose first step reads the issue body (`issue_sync.field_show`) — no issue exists; clarify entry at log-init, skip body/title/Logs checks — `.claude/skills/start-deployment/SKILL.md:239-241`
- [ ] (suggestion) Background `dev_push` ("report when it finishes") drops the original "On failure, print `issue_sync.failure_hint`" obligation; carry failure-surfacing into the background branch — `.claude/skills/start-deployment/SKILL.md:401-410`
- [ ] (suggestion) `deployment_mode.md` three-state table not updated for issue-less / offer-and-confirm / background dev_push (consequences map) — `.agent/knowledge/deployment_mode.md:221-230`
