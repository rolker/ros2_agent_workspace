---
issue: 491
---

# Issue #491 — 'address findings' skill (consume ## Integrated Review)

## Issue Review
**Status**: complete
**When**: 2026-06-15 09:30 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Issue**: #491
**Comment**: https://github.com/rolker/ros2_agent_workspace/issues/491#issuecomment-4704591505
**Scope verdict**: well-scoped

### Actions
- [ ] (must resolve in plan-task) Entry-type gate (ADR-0013): default to reusing `## Implementation`; mint `## Findings Addressed` only via a superseding ADR if #492 needs to distinguish it. This is the plan's primary open question.
- [ ] New workflow skill ⇒ update skill list in non-Claude adapters (`.github/copilot-instructions.md`, `.agent/instructions/gemini-cli.instructions.md`, `.agent/AGENT_ONBOARDING.md`).
- [ ] Persist a typed `progress.md` entry per ADR-0013 (follows from the entry-type decision).
- [ ] Scope the triage-reviews/review-code de-duplication (in-PR vs follow-up) so findings aren't double-handled.
- [ ] Update the #491 issue body to note #485 is merged (blocker cleared).

## Plan Authored
**Status**: complete
**When**: 2026-06-15 09:45 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Plan**: `.agent/work-plans/issue-491/plan.md` at `108a089` (refreshed from `52f523a` after open-questions resolution)
**PR**: https://github.com/rolker/ros2_agent_workspace/pull/524 (`[PLAN]` prefix)
**Phases**: single

### Open questions
- [x] Entry type → **reuse `## Implementation`** (no new ADR). Confirmed with user 2026-06-15.
- [x] De-dup scope → **follow-up PR**; this PR only adds the skill. Confirmed with user 2026-06-15.

## Plan Review
**Status**: complete
**When**: 2026-06-15 10:20 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context)) (in-context — author self-review)

**Plan**: `.agent/work-plans/issue-491/plan.md` at `108a089`
**PR**: https://github.com/rolker/ros2_agent_workspace/pull/524
**Verdict**: approve-with-suggestions

### Findings
- [x] (must-fix) Missing consequence: `dispatch_subagent.sh` `skill_entry_type()` had no `address-findings` case → returned `""`, silently skipping the exit-contract check. FIXED in `8a91aef`: added `address-findings) echo "Implementation"` (verified resolves). Plan consequences table updated.
- [x] (suggestion) Missing consequence: `.agent/knowledge/skill_workflows.md`. FIXED in `8a91aef`: added `address-findings` to the lifecycle diagram + Skill Index row; plan consequences updated.
- [x] (suggestion) Stale SHA in `## Plan Authored`. FIXED: refreshed `52f523a` → `108a089`.

## Implementation
**Status**: complete
**When**: 2026-06-15 10:45 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Addressed**: `## Plan Review` (approve-with-suggestions) — all 3 findings.
**Commits**: `8a91aef` (skill + dispatcher wiring + lifecycle doc + 3 adapters + plan consequences)

### Actions
- [x] New `.claude/skills/address-findings/SKILL.md` (thin; reads last `## Integrated Review` via `progress_read.py`, commits each fix atomically, writes `## Implementation`).
- [x] `dispatch_subagent.sh` `skill_entry_type()` → `address-findings) echo "Implementation"` (must-fix).
- [x] `skill_workflows.md` diagram + Skill Index updated.
- [x] `address-findings` added to all 3 framework adapter skill lists.
- [ ] De-dup of old triage-reviews/review-code external-review handling — **deferred to a follow-up PR** (per settled open question).

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-15 10:33 +00:00
**By**: Claude Code Agent (Claude Opus 4.8)
**Verdict**: changes-requested

**Branch**: feature/issue-491 at `6d1f30e`
**Mode**: pre-push
**Depth**: Standard (reason: governance/workflow files — dispatcher, adapters, lifecycle docs)
**Must-fix**: 2 | **Suggestions**: 1

### Findings
- [x] (must-fix) Lifecycle Handoff Convention table not updated for `address-findings`; same-file diagram + Skill Index were — internally inconsistent — `.agent/knowledge/skill_workflows.md:80`. FIXED in `b53086e`: table now lists triage→address-findings→re-review rows.
- [x] (must-fix) `triage-reviews` "Next step" still claims it is the terminal automated skill with no handoff; now false — `.claude/skills/triage-reviews/SKILL.md:363-381`. FIXED in `b53086e`: points to the address-findings handoff (Scope-E note retained).
- [x] (suggestion) Step 2 doesn't handle the "file exists, zero Integrated Review entries" case — `.claude/skills/address-findings/SKILL.md:45-62`. FIXED in `b53086e`: explicit no-entry guard (don't fall back to other types).

### Resolution
**By**: Claude Code Agent (Claude Opus 4.8 (1M context)) — 2026-06-15. Sandboxed review-code = changes-requested; all 3 findings addressed in `b53086e`. Lifecycle coherence verified (address-findings named consistently across diagram, both tables, and triage-reviews handoff).

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-15 11:00 +00:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))
**Verdict**: changes-requested

**Branch**: feature/issue-491 at `c21ff05`
**Mode**: pre-push
**Depth**: Standard (reason: governance/workflow files — dispatcher, framework adapters, lifecycle docs, new workflow skill)
**Must-fix**: 3 | **Suggestions**: 5

### Findings
- [x] (must-fix) `## Implementation` template omits ADR-0013's required correlation field — FIXED `25b0ecd`: template now carries `**Branch**:`/`**PR**: <name> at <sha>`.
- [x] (must-fix) ADR-0013 Writer registration for `## Implementation` — FIXED `adab51c`: ADR-0012 cross-reference addendum registers `address-findings` as a second writer (Decision table unchanged; user-approved approach).
- [x] (must-fix) Step-5 "Commit and push" contradicted the no-push dispatch contract — FIXED `25b0ecd`: commit only, host pushes.
- [x] (suggestion) Deferred-finding dead-end — FIXED `25b0ecd`: deferred items written checked+`(deferred:)` so they're not re-attempted (`address-findings` acts only on `checked == false`).
- [x] (suggestion) `## Implementation` shared routing key — FIXED `25b0ecd`: forward-flag note added for #492 (orchestrator disambiguates by preceding entry).
- [x] (suggestion) Staging vs box-check tension — FIXED `25b0ecd`: box-check is the one permitted addition to a finding's commit.
- [x] (suggestion) `## External Review` legacy match — FIXED `25b0ecd`: step 2 filters on `base_type == "Integrated Review"`.
- [x] (suggestion) Worktree/commit-consistency — FIXED `25b0ecd`: commits run from the issue worktree; placeholder removed.

### Resolution (round 2)
**By**: Claude Code Agent (Claude Opus 4.8 (1M context)) — 2026-06-15. Sandboxed re-review = changes-requested (3 must-fix + 5 suggestions); all 8 addressed in `25b0ecd` (skill) + `adab51c` (ADR-0013 addendum). The ADR registration approach (ADR-0012 addendum vs superseding ADR) was confirmed with the user. Round-2 cross-confirmed-clean items (dispatcher exit-contract, Scope-E, adapters, tables) left intact.

### Notes
- Cross-confirmed clean: dispatcher exit-contract is **not** confused by the shared `## Implementation` type — both adversarial passes independently verified the count-delta (`entry_count` PRE vs POST) mechanism in `dispatch_subagent.sh`.
- Scope-E (no auto-chaining) and commit-identity/no-`--no-verify` verified consistent across `triage-reviews`, `address-findings`, and `skill_workflows.md`. Adapters (3/3), lifecycle diagram + both tables, and the dispatcher mapping all updated correctly.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-15 12:53 +00:00
**By**: Claude Code Agent (Claude Opus 4.8)
**Verdict**: changes-requested

**Branch**: feature/issue-491 at `3707c49`
**Mode**: pre-push
**Depth**: Standard (reason: governance/workflow files — new workflow skill, dispatcher, framework adapters, lifecycle docs, ADR addendum)
**Must-fix**: 1 | **Suggestions**: 3

### Findings
- [x] (must-fix) Guidelines bullet "Deferred/non-actionable items stay **unchecked**" contradicted Steps 3.2/5 (which write deferred items checked) — FIXED `aa8c25d`: Guidelines bullet reconciled to the checked+annotated convention.
- [x] (suggestion) "re-attempted every run" rationale overstated — FIXED `aa8c25d`: reworded as an idempotency guard for a re-run against the *same* entry, not "every run".
- [ ] (suggestion, deferred → de-dup follow-up) Cross-round dropout: a still-open `(deferred:)` finding looks the same as a resolved checked one to `progress_read.py`; `triage-reviews` isn't told to re-raise it. Reviewer tagged this for the de-dup follow-up (touches `triage-reviews`, out of this PR's scope). Carry into the de-dup follow-up issue.
- [ ] (suggestion, deferred → low priority) `review-code` "Next step" doesn't explicitly acknowledge the re-review entry path; existing wording mostly covers it and the file isn't in this diff. Optional polish.

### Resolution (round 3)
**By**: Claude Code Agent (Claude Opus 4.8 (1M context)) — 2026-06-15. Sandboxed re-review (round 3) = changes-requested (1 must-fix + 3 suggestions). The must-fix was a self-introduced round-2 contradiction (Guidelines vs Steps), now fixed in `aa8c25d` along with the related rationale suggestion. The 2 remaining suggestions are deferred by design — one to the de-dup follow-up (reviewer-tagged), one low-priority polish on a file not in this diff. Findings converged 3→8→1; round 3 cross-confirmed all mechanics clean (consequences 3/3 adapters + dispatcher + skill_workflows + ADR addendum, Scope-E, commit-identity/no-push, shellcheck).

### Notes
- Round-3 cross-confirmed clean (unchanged, re-verified): all consequence categories (3/3 adapters, dispatcher `skill_entry_type`+`skill_model`, `skill_workflows.md` index/diagram/handoff table, ADR-0013 addendum), Scope-E, commit-identity/no-push, and the ADR-0012-addendum governance choice. shellcheck clean on `dispatch_subagent.sh`.
- The single must-fix is a **new** finding (internal Guidelines↔Steps contradiction) not surfaced in rounds 1–2; the deferred-checkbox prose was added in round 2 (`25b0ecd`) and the Guidelines bullet was not reconciled with it.
