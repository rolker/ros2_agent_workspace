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

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-21 00:51 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-533 at `d879913`
**Mode**: pre-push
**Depth**: Standard (reason: `.claude/skills/*/SKILL.md` is a Governance override-trigger file)
**Must-fix**: 1 | **Suggestions**: 3
**Round**: 2 | **Ship**: recommended — round-1 findings all resolved; lone must-fix is a mechanical one-string fix (pin canonical `pending` marker), not a design question

### Findings
- [ ] (must-fix) Reconciliation marker mismatch: seeded `Deployment issue: _pending — ...` but backfill scans for literal `issue: pending` (not a substring of `issue: _pending`) — verbatim match finds zero issue-less logs, silently skipping backfill (the "forever untracked" failure at 250-253). Pin one canonical grep-safe string across all 5 sites — `.claude/skills/start-deployment/SKILL.md:233,246,264,336` + `.agent/knowledge/deployment_mode.md:234,236`
- [ ] (suggestion) Wrap-up never scans logs for `pending`; loop-closure is human-memory-dependent — note orphaned-`pending`-log discovery in wrap-up — `.claude/skills/wrap-up-deployment/SKILL.md` (consequence of this change)
- [ ] (suggestion) Issue-less "continuing a prior deployment" branch overlaps Resume (4c) when a matching local log exists — add pointer to 4c to avoid a duplicate log — `.claude/skills/start-deployment/SKILL.md:234-237`
- [ ] (suggestion) Log-init title template shows `#<N>` first; issue-less alt title is 8 lines later — risk of stamping empty `#` under pressure; co-locate the variant — `.claude/skills/start-deployment/SKILL.md:326,334-335`
