---
issue: 596
---

# Issue #596 — Wire documentation and agent-instruction update review into the per-issue lifecycle

## Issue Review
**Status**: complete
**When**: 2026-07-31 18:50 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #596
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Scope Assessment

The issue proposes injecting documentation/instruction impact review into three existing seams of the per-issue lifecycle — no new phases. Files targeted are clearly identified (`.claude/skills/plan-task/SKILL.md`, `.claude/skills/review-code/SKILL.md`, `.agent/knowledge/principles_review_guide.md`, and optionally `.claude/skills/review-plan/SKILL.md`). The operator-settled design includes an explicit out-of-scope boundary (no auto-editing instruction files without approval, no new lifecycle phases, no project-repo changes). Fits a single PR.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | OK | Instruction-update candidates are flagged at checkpoints only; operator decides before any edits. Design explicitly rejects auto-drafting. |
| Enforcement over documentation | Watch | New doc-impact rows in plan-task template and review-code are guidance, not mechanically enforced. Acceptable given the domain (instruction files), but worth noting the absence of a hook or CI check for the new required plan section. |
| Capture decisions, not just implementations | Watch | The "flag at checkpoints, operator decides" design is a meaningful process decision settled in the issue body. No ADR is proposed. Issue body serves as an informal decision record; acceptable since this is a workflow process change, not a new architectural pattern. |
| A change includes its consequences | OK | Issue correctly identifies that review-plan/SKILL.md should be checked if it enumerates plan sections; flags this as a "verify against source" step. Consequences are accounted for within scope. |
| Only what's needed | OK | Three targeted seams, minimal text additions. No new lifecycle phase. "Consider" flag on document-package wiring keeps scope flexible without committing. |
| Improve incrementally | OK | Small additions to existing files. No rewrite. |
| Test what breaks | OK | No testable code is changed — instruction files only. The review-code governance specialist change adds checking, not bypasses it. |
| Workspace vs. project separation | OK | All changes stay in workspace infra. Consequences Map rows reference project repo `.agents/` files per ADR-0017, which is correct. |
| Workspace improvements cascade to projects | OK | The benefit (better doc-impact review) applies to any issue in any project repo using this lifecycle. |
| Primary framework first, portability where free | OK | `.claude/skills/` changes are Claude-specific; knowledge-doc changes are framework-agnostic. Appropriate split. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| ADR-0006 — Shared AGENTS.md | Watch | Changes to `.claude/skills/` (framework adapter). AGENTS.md itself is not directly modified by this issue, but the script reference table and "framework skill" Consequences Map row suggest checking framework adapters if plan-task/review-code behaviors change. |
| ADR-0004/0005 — Enforcement hierarchy | Watch | New plan section requirement is documentation-only (not hooked/CI-checked). Acceptable for instruction files; note it as a known gap. |
| ADR-0013 — progress.md entry-type vocabulary | Not triggered | No new entry types introduced. |
| ADR-0001 — Adopt ADRs | Watch | "Flag at checkpoints, operator decides" on instruction-update candidates is a design decision. Issue body serves as the record; an ADR is not required here but would be appropriate if this pattern generalizes further. |

### Consequences

Per the Consequences Map:
- Modifying `.claude/skills/plan-task/SKILL.md` and `.claude/skills/review-code/SKILL.md` → check framework adapters (`.github/copilot-instructions.md`, `.agent/instructions/gemini-cli.instructions.md`, `.agent/AGENT_ONBOARDING.md`) for skill list or behavior mentions. Issue does not flag this — **action needed** during implementation.
- Adding rows to `.agent/knowledge/principles_review_guide.md` → skills that reference it by name should be verified (review-issue, review-plan, review-code all reference this guide). Changes to the Consequences Map table are additive (new rows), so existing consumers are not broken.
- The issue correctly flags: if review-plan enumerates plan sections, it should check for the new "Documentation & instruction impact" section — verify against source before finalizing plan.

### Actions
- [ ] During implementation, check framework adapter files (`.github/copilot-instructions.md`, `.agent/instructions/gemini-cli.instructions.md`, `.agent/AGENT_ONBOARDING.md`) for any references to plan-task or review-code behavior that would become stale.
- [ ] Verify `.claude/skills/review-plan/SKILL.md` against source — confirm whether it enumerates expected plan sections, and if so add the new "Documentation & instruction impact" section to its checklist.
- [ ] Confirm the plan explicitly states "none" in the documentation impact section (or equivalent) to validate the required-explicit-acknowledgment design goal — the template wording should make silence a conspicuous omission.

## Plan Authored
**Status**: complete
**When**: 2026-07-31 19:02 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-596/plan.md` at `1c6ab2a`
**Branch**: feature/issue-596 at `1c6ab2a`
**Phases**: single

### Open questions
- [ ] No open questions — plan is review-plan-ready.

## Plan Review
**Status**: complete
**When**: 2026-07-31 19:05 +00:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-596/plan.md` at `1c6ab2a`
**PR**: PR-less (--issue mode)
**Verdict**: approve-with-suggestions

### Findings
- [x] (suggestion) plan-task file-change note points at "step 6" for line-count guidance; guidance actually lives in the Guidelines section (`plan-task/SKILL.md:409`) — `plan.md:51`
- [x] (suggestion) new required plan section is framed as documentation-only, but review-plan step-4 dimension + review-code 5b are review-layer checks — state this to strengthen the ADR-0004 posture — `plan.md:60-63,71`
- [x] (suggestion) `review-plan/SKILL.md` is in Files to Change but absent from the Consequences "framework skill → adapters" rows; add it (adapters reference by URL only, no update needed) — `plan.md:76-81`
