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
- [x] (must-fix) Inaccurate rationale: `--type "Local Review (Pre-Push)"` does NOT surface `## Local Review` per `progress_read.py` `_matches_type`; drop the false claim — `.claude/skills/address-findings/SKILL.md:74`
- [x] (suggestion) Note that pre-push *suggestions* are auto-actioned (address-findings acts on all unchecked findings, unlike post-PR triage) — `.claude/skills/address-findings/SKILL.md:28`
- [x] (suggestion) Add a max-rounds/escalation note to the automated changes-requested → address-findings → review-code loop — `.claude/skills/run-issue/SKILL.md:147`
- [x] (suggestion) Document where the dispatcher FAILED report surfaces on the background re-invocation path — `.claude/skills/run-issue/SKILL.md:57`
- [x] (suggestion) Note that "single latest entry" relies on progress.md append order — `.claude/skills/address-findings/SKILL.md:75`
- [x] (consequence) Update `review-code/SKILL.md` Next-step to document the pre-push changes-requested → address-findings branch (file not in this diff) — `.claude/skills/review-code/SKILL.md:918`

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-20 19:43 +00:00
**By**: Claude Code Agent (Claude Opus 4.8)
**Verdict**: changes-requested

**Branch**: feature/issue-531 at `529d390`
**Mode**: pre-push
**Depth**: Standard (reason: `.claude/skills/*/SKILL.md` is a Governance override-trigger)
**Must-fix**: 1 | **Suggestions**: 4

### Findings
- [x] (must-fix) Decision-table row not generalized: `## Implementation` preceded by `## Local Review (Pre-Push)` (the new pre-push address-findings state) matches no table row, contradicting the prose at :137 — add "or `## Local Review (Pre-Push)`" — `.claude/skills/run-issue/SKILL.md:119`
- [x] (suggestion) Background path: instruct gating on the dispatch exit/FAILED report before routing on the timeline, so a no-entry failure isn't silently re-tried — `.claude/skills/run-issue/SKILL.md:67`
- [x] (suggestion) Give the host a durable round counter for the address-findings⇄review-code loop (count `## Local Review (Pre-Push)` entries) — `.claude/skills/run-issue/SKILL.md:162`
- [x] (suggestion) Caveat that background re-invocation depends on the host-runtime completion callback (mirror the Agent-tool caveat at :48-51) — `.claude/skills/run-issue/SKILL.md:61`
- [x] (suggestion) Round-1 findings addressed in `529d390` but checkboxes remain unchecked and no `## Implementation` entry recorded — timeline misrepresents resolution state — `.agent/work-plans/issue-531/progress.md:19`

## Implementation
**Status**: complete
**When**: 2026-06-20 16:36 -04:00
**By**: Claude Code Agent (Claude Opus 4.8) [host-inline — not address-findings]
**Branch**: feature/issue-531 at `dcd9ce5`
**Addressed**: Local Review (Pre-Push) rounds 1 + 2

### Actions
- [x] Round 1: parser-claim must-fix + 4 suggestions + review-code Next-step consequence
- [x] Round 2: decision-table-row must-fix (generalized predecessor) + background gating/callback caveats + round-counter note + this timeline bookkeeping
