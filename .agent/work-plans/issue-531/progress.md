---
issue: 531
---

# Issue #531 — run-issue: background container dispatch

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-20 17:27 +00:00
**By**: Claude Code Agent (Claude Opus 4.6)
**Verdict**: changes-requested

**Branch**: feature/issue-531 at `97463ac`
**Mode**: pre-push
**Depth**: Standard (reason: `.claude/skills/*/SKILL.md` is a Governance override-trigger)
**Must-fix**: 1 | **Suggestions**: 4

### Findings
- [ ] (must-fix) Inaccurate rationale: `--type "Local Review (Pre-Push)"` does NOT surface `## Local Review` per `progress_read.py` `_matches_type`; drop the false claim — `.claude/skills/address-findings/SKILL.md:74`
- [ ] (suggestion) Note that pre-push *suggestions* are auto-actioned (address-findings acts on all unchecked findings, unlike post-PR triage) — `.claude/skills/address-findings/SKILL.md:28`
- [ ] (suggestion) Add a max-rounds/escalation note to the automated changes-requested → address-findings → review-code loop — `.claude/skills/run-issue/SKILL.md:147`
- [ ] (suggestion) Document where the dispatcher FAILED report surfaces on the background re-invocation path — `.claude/skills/run-issue/SKILL.md:57`
- [ ] (suggestion) Note that "single latest entry" relies on progress.md append order — `.claude/skills/address-findings/SKILL.md:75`
- [ ] (consequence) Update `review-code/SKILL.md` Next-step to document the pre-push changes-requested → address-findings branch (file not in this diff) — `.claude/skills/review-code/SKILL.md:918`
