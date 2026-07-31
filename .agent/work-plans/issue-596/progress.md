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


## Implementation
**Status**: complete
**When**: 2026-07-31 19:17 +00:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-596 at `048c26a`
**Plan**: `.agent/work-plans/issue-596/plan.md`

### What landed

Five atomic commits on `feature/issue-596` (plan-sync + four seams):

1. `9c949cb` — **plan-sync**: folded the three operator-approved amendments
   into `plan.md` and checked off the three Plan Review findings in this file:
   (1) fixed the plan-task line-count pointer to the Guidelines section
   (`plan-task/SKILL.md:409`), not "step 6"; (2) restated the new plan
   section's ADR-0004 posture as instruction + review layer (review-plan
   step-4 + review-code 5b), not documentation-only; (3) added
   `review-plan/SKILL.md` to the plan's own Consequences rows.
2. `3d75f82` — **plan-task/SKILL.md**: added the required
   `## Documentation & Instruction Impact` section to the plan template
   (step 5), obliging an explicit "None — <reason>" so silence is a
   conspicuous omission; separates stale docs (same-PR) from
   agent-instruction candidates (operator decides, never auto-applied).
3. `f156180` — **principles_review_guide.md**: two additive Consequences Map
   rows — parameter/topic/service changes → package README/API docs +
   review-context.yaml; surfaced pattern/pitfall → operator-approved
   knowledge/README candidate.
4. `3b4d16a` — **review-code/SKILL.md**: doc-impact check added to Governance
   specialist 5b (parameter/topic/service doc updates Missing-if-stale;
   missed pattern raised as operator-facing candidate).
5. `048c26a` — **review-plan/SKILL.md**: new step-4 "Documentation &
   instruction impact" dimension + Evaluation-table row verifying the plan
   section exists, is non-silent, and frames instruction updates as
   candidates.

### Issue Review actions honored

- Framework adapters checked (`.github/copilot-instructions.md`,
  `.agent/instructions/gemini-cli.instructions.md`,
  `.agent/AGENT_ONBOARDING.md`): all reference plan-task/review-code/
  review-plan by SKILL.md **URL and skill-list name only**, no behavior
  prose — no adapter update needed (recorded in the plan Consequences).
- `review-plan/SKILL.md` verified against source before editing — it does
  enumerate plan dimensions (step 4 + report table), so both were updated.
- Instruction-update design kept flag-at-checkpoints: candidates are
  proposals in the plan section / review findings, never auto-applied.

### Verification

- Documentation claims verified against source (read the actual skill text
  cited before each edit).
- All six commits passed pre-commit hooks (identity, branch, issue-match)
  under the agent identity. Not pushed — host performs the push.

### Next step

Lifecycle: **Implementation** → **review-code** (pre-push). Hand off to a
fresh-context `review-code` sub-agent, then push / open the PR
(`Closes #596`).

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-31 19:21 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-596 at `816c09d`
**Mode**: pre-push
**Depth**: Standard (reason: governance-touching files — .claude/skills/**, principles_review_guide.md)
**Must-fix**: 0 | **Suggestions**: 3
**Round**: 1 | **Ship**: recommended — no Must-fix; cross-references verified consistent across all four docs, only low-severity suggestions remain

### Findings
- [x] (suggestion) plan.md:51 cites `plan-task/SKILL.md:409` for line-count guidance; `## Guidelines` is at 412, the 30-80-lines bullet at 423 — stale line ref — `.agent/work-plans/issue-596/plan.md:51`
- [x] (suggestion) doc-impact check lives in 5b Governance Specialist, which Light tier skips; param/topic/service doc check silently unenforced on the Light path — note it's Standard+ only or fold into 5a — `.claude/skills/review-code/SKILL.md:425`
- [x] (suggestion) `.agent/knowledge/` and `.agents/README.md` appear on both sides of the plan-task template split (stale-docs same-PR vs operator-decided candidates); a plan editing a knowledge doc directly could trip a spurious candidate finding — disambiguate — `.claude/skills/plan-task/SKILL.md:148-153`

Note: --no-local (standing decision #590); Copilot off (default). Local + Copilot adversarial specialists omitted. Guidance-doc calibration (#537) applied — all findings are Suggestions; none would actively mislead.

## Integrated Review
**Status**: complete
**When**: 2026-07-31 15:34 -04:00
**By**: Claude Code Agent (Claude Fable 5)

**PR**: #597 at `2f85576`
**Sources**: 2 (Copilot R1 @ `2f85576`, Local Review (Pre-Push) @ `816c09d`) + CI rollup
**Cross-source confirmations**: 0
**CI**: all-pass

### Findings
- [x] (minor, Copilot) plan.md:51 hard-coded refs `plan-task/SKILL.md:412`/`:423` stale again — the R1 fix commit `2f85576` updated the numbers and re-staled them in the same commit (6 lines inserted above Guidelines; now 418/429). Recurrence of Local Review R1 finding 1 (different head SHA, so not a formal cross-confirmation) — numeric refs are fragile; replace with a by-name reference to the Guidelines section — `.agent/work-plans/issue-596/plan.md:51`
- [x] (minor, Copilot) Implementation entry says "Six atomic commits … (plan-sync + four seams)" but five are listed and five exist on the branch — write-time counting typo; change "Six" to "Five" — `.agent/work-plans/issue-596/progress.md:93`

### False positives
- (Copilot) progress.md:158 Local Review entry cites "self-stale" line numbers (412/423) — verified via `git show 816c09d` that both were accurate at the entry's correlation key (branch head `816c09d`, ADR-0013 review-entry semantics); timeline entries are point-in-time historical records and the finding is closed, so retro-editing to track later drift would falsify the review record.

### Next step
No must-fix or cross-confirmed findings; CI green. Two minor valid one-liners remain — apply via address-findings (or by hand) then merge, or merge as-is at operator discretion.
