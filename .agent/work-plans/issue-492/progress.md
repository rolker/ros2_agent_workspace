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

## Plan Review
**Status**: complete
**When**: 2026-06-15 15:40 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Plan**: `.agent/work-plans/issue-492/plan.md` at `3068207`
**PR**: https://github.com/rolker/ros2_agent_workspace/pull/525
**Verdict**: approve-with-suggestions

### Findings
- [ ] (must-fix) plan-task-flip ripple incomplete: `review-issue/SKILL.md:272` states "A draft PR will follow when `plan-task` runs" — false once the default flips. Add `review-issue/SKILL.md` to Files to Change. — `plan.md:60-68`
- [ ] (suggestion) Second early-PR path unreconciled: `worktree_create.sh --plan-file` also auto-opens a draft PR. Note it as the equivalent opt-in to `--draft-pr` so the two publish-early paths stay consistent (no code change required). — `plan.md:64`
- [ ] (suggestion) Decision table has no row for "PR published ⇒ dispatch `triage-reviews`"; the async wait for review comments to arrive is the one transition the host can't fully automate. Add a row/prose note so the gap is explicit. — `plan.md:23-37`
- [ ] (suggestion) Decision-table key `## Local Review (Pre-Push)` is correct, but `skill_workflows.md` handoff table still lists `## Local Review`; the orchestrator must match the parenthetical variant (dispatch_subagent.sh already does this — count-by-prefix at lines 217-235). Call this matching rule out in the skill. — `plan.md:32`
- [ ] (good) Claim #3 (orchestrator needs no `skill_entry_type` row) verified — `dispatch_subagent.sh:53-62` map has no `run-issue` case; the driver is never dispatched as a phase.
- [ ] (good) ADR-0011 field-mode + never-auto-push guarantee solid: checkpoint before every push/PR/merge, publish branches on `field_mode.sh` (push, no PR/Copilot). — `plan.md:42-57`
- [ ] (good) Shared-`## Implementation` disambiguation (route by preceding entry) matches the address-findings note at `address-findings/SKILL.md:37-42`. — `plan.md:36`

## Implementation
**Status**: complete
**When**: 2026-06-15 15:10 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Branch**: feature/issue-492 at `7ea516b`
**Addressed**: `## Plan Review` (approve-with-suggestions) — all 4 findings folded into the plan + implemented.
**Commits**: `7ea516b` (orchestrator + plan-task flip + ripple + adapters)

### Actions
- [x] New `.claude/skills/run-issue/SKILL.md` — decision table, checkpoints, shared-`## Implementation` disambiguation, entry-type variant matching, field-mode-aware local-first publish, Copilot opt-in.
- [x] plan-task flipped to local-first: `--draft-pr` opt-in; step 7 optional, step 8 conditional push, step 9 + Plan Authored template + During-implementation prose updated.
- [x] (review-plan must-fix) review-issue:272 corrected.
- [x] (review-plan sugg) decision-table row for PR-published ⇒ triage-reviews; `## Local Review (Pre-Push)` variant-matching rule; `worktree_create.sh --plan-file` noted as equivalent publish-early opt-in.
- [x] skill_workflows.md driver note + Skill Index row; run-issue added to all 3 adapters.
- [x] review-plan + AGENTS.md confirmed **no change needed** (PR-less-capable / already conditional).

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-15 22:26 +00:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))
**Verdict**: changes-requested

**Branch**: feature/issue-492 at `64d2e83`
**Mode**: pre-push
**Depth**: Deep (reason: 398 changed lines ≥200; 7 governance override-trigger files)
**Must-fix**: 2 | **Suggestions**: 3

### Findings
- [ ] (must-fix) `## Local Review` vs `## Local Review (Pre-Push)` routing ambiguous — post-PR re-review can match the pre-push publish row / no post-publish entry to key on (mitigated by always-on publish checkpoint) — `.claude/skills/run-issue/SKILL.md:54-62`
- [ ] (must-fix) `progress_read.py --type "<Entry Type>"` read command contradicts the prefix-matching rule; `--type "Local Review"` misses `## Local Review (Pre-Push)` — use the dispatcher's `startswith` approach — `.claude/skills/run-issue/SKILL.md:56,59-62`
- [ ] (suggestion) Reconcile plan prose "adjust review-plan default resolution" with "No change needed" Files-to-Change row — `.agent/work-plans/issue-492/plan.md:120-122`
- [ ] (suggestion) Clarify what `--copilot` consumes at the publish step (push + `gh pr create` only) — `.claude/skills/run-issue/SKILL.md` (Publish step)
- [ ] (suggestion) Confirm `Agent`-tool availability in container mode for the review-code fan-out path — `.claude/skills/run-issue/SKILL.md:46-48`

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-16 02:21 +00:00
**By**: Claude Code Agent (Claude Opus 4.8)
**Verdict**: changes-requested

**Branch**: feature/issue-492 at `29fafef`
**Mode**: pre-push
**Depth**: Deep (reason: 455 changed lines ≥200; multiple governance override-trigger files — SKILL.md / knowledge / instruction files)
**Must-fix**: 2 | **Suggestions**: 4

Round 2. Prior round-1 must-fixes (entry-type routing ambiguity, `--type` filter contradiction) verified **resolved** against `progress_read.py` + `dispatch_subagent.sh`.

### Findings
- [ ] (must-fix) Dangling `$DRAFT_PR` var: `[ -n "$DRAFT_PR" ] && git push` tests a never-set var (gate is the `--draft-pr` flag everywhere else), so the push is silently skipped even with `--draft-pr` — contradicts step 7 + step 8 prose — `.claude/skills/plan-task/SKILL.md:266`
- [ ] (must-fix) SKILL claims `## Local Review` (no parenthetical) "appears only as the abbreviation in skill_workflows.md … not routed on" — but it is the real canonical heading `review-code` writes in post-PR mode (`review-code/SKILL.md:872`); decision table has no row for it. Clarify pre-push-only dispatch or add a post-PR row — `.claude/skills/run-issue/SKILL.md:109-112`
- [ ] (suggestion) `(PR published)` table state not determinable from last entry (Publish step writes no progress.md entry); a re-run could double-publish — document out-of-band tracking or write a publish marker — `.claude/skills/run-issue/SKILL.md:102,140-160`
- [ ] (suggestion) Field path omits the `no-commit-to-branch` hook caveat (AGENTS.md § Field Mode); add a one-line pointer (never `--no-verify`) — `.claude/skills/run-issue/SKILL.md:157-160`
- [ ] (suggestion) Note in PR body that 3 instruction (adapter) files were edited (additive list-only; consequence-map ripple) so the "Ask First" touch is visible — PR body
- [ ] (suggestion) Forward-compat: "table is unchanged" for a future `implement` skill doesn't hold — a bare `## Implementation` (not preceded by `## Integrated Review`) has no row (unreachable today) — `.claude/skills/run-issue/SKILL.md:114-122`
