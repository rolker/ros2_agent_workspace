---
issue: 466
---

# Issue #466 — Persist review-issue and review-plan output to progress.md

## Issue Review
**Status**: complete
**When**: 2026-06-21 05:08 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #466
**Comment**: (not posted — GitHub posting is optional/best-effort per local-first contract #532)
**Scope verdict**: well-scoped

### Scope Assessment

**Well-scoped?** Yes — the persistence half is already shipped (both `review-issue` and
`review-plan` write typed `progress.md` entries). The remaining work is narrowly defined:
(1) make `review-issue` report `complete` (not `partial`) when the only unmet step is the
optional GitHub post, and (2) verify `dispatch_subagent.sh` does not surface a locally-
complete phase as FAILED solely because the post was skipped. Both fit a single PR.

**Right repo?** Yes — workspace repo. Changes are to `.claude/skills/review-issue/SKILL.md`
and `.agent/scripts/dispatch_subagent.sh` (verification/documentation; may not need a code
change). No project-repo changes required.

**Dependencies**: The issue body confirms the persistence half is shipped (observed driving #550).
No blocking open issues identified. Issue #532 (local-first posting contract) is referenced
as context, not a blocker.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | OK | Silently skipping an optional post and recording the skip in `progress.md` keeps the record honest; no hidden state change |
| Enforcement over documentation | OK | Skill instructions are the appropriate enforcement layer here; behavior fix in SKILL.md is sufficient |
| Capture decisions, not just implementations | Watch | The "posting is best-effort/optional" decision should be explicit in SKILL.md — not just implied by the fix. Issue text calls for skill docs update, which covers this. |
| A change includes its consequences | Action needed | SKILL.md must be updated; `dispatch_subagent.sh` behavior should be verified and documented; AGENTS.md/CLAUDE.md need no change (issue text confirms they don't reference per-skill persistence detail) |
| Only what's needed | OK | Minimal change: update the complete/partial/failed logic in the skill and SKILL.md; no scope creep needed |
| Improve incrementally | OK | Persistence half already shipped; this is a clean targeted follow-up |
| Workspace vs. project separation | OK | Workspace skills and scripts only |
| Workspace improvements cascade to projects | OK | All container-dispatched phases benefit from the corrected posting behavior |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| ADR-0002 — Worktree isolation | Yes | Work proceeds in the existing `issue-workspace-466` worktree |
| ADR-0004 — Enforcement hierarchy | Watch | The optional-posting rule lives only in skill instructions; no hook/CI enforcement possible for skill behavior, so docs-only is acceptable here |
| ADR-0006 — Shared AGENTS.md | No | Issue text explicitly confirms AGENTS.md/CLAUDE.md references are unchanged |
| ADR-0013 — progress.md vocabulary | Yes | Entry type `## Issue Review` with `complete` status is per ADR-0013. The fix must ensure the entry is committed *before* the optional posting attempt, so the timeline is correct even if posting is skipped |

### Consequences

Per the consequences map:
- Changing a framework skill → framework adapter file may need updating if it references posting behavior; issue text says no AGENTS.md/CLAUDE.md change needed
- If `dispatch_subagent.sh` changes → update AGENTS.md script reference table (§ Script Reference)
- The posting-as-optional change must be recorded in SKILL.md so the intent survives future reads

### Actions
- [ ] Update `review-issue` SKILL.md: document GitHub posting as best-effort/optional; when no gh auth, skip silently and report `complete`
- [ ] Ensure the `progress.md` write + commit happens *before* the optional posting attempt (so entry is always present regardless of posting outcome)
- [ ] Verify `dispatch_subagent.sh`: confirm that a `complete` progress.md entry from the skill results in exit 0 (no code change likely needed — the current logic already maps `complete → exit 0`; add a comment if helpful)
- [ ] If `dispatch_subagent.sh` is changed, update AGENTS.md script reference table entry for `dispatch_subagent.sh`
- [ ] Skill docs updated per acceptance criteria

## Plan Authored
**Status**: complete
**When**: 2026-06-21 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-466/plan.md`
**Scope**: Post-side only — `review-issue` SKILL.md posting behavior; `review-plan` has no posting step (no change needed); `dispatch_subagent.sh` already maps `complete → exit 0` (no code change needed).

### Findings
- [ ] Plan looks solid. Ready for implementation.
