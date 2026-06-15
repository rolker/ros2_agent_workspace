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
- [ ] (must-fix) `## Implementation` template omits ADR-0013's required correlation field (`**PR**:`/`**Branch**: <name> at <sha>`); `progress_read.py` lists Implementation in `_PR_BRANCH_TYPES` → entries parse to `correlation: null` — `.claude/skills/address-findings/SKILL.md:107-120`
- [ ] (must-fix) ADR-0013 Writer column for `## Implementation` still says "future implement skill" only; `address-findings` is now a second writer and must be registered (light addendum, no new ADR) — `docs/decisions/0013-progress-md-entry-type-vocabulary.md:56`
- [ ] (must-fix) Step-5 heading "Commit and push" but block only commits; contradicts the sub-agent no-push dispatch contract — `.claude/skills/address-findings/SKILL.md:122`
- [ ] (suggestion) Deferred-finding dead-end: not-actionable items stay unchecked and are re-attempted every run; nothing consumes the deferred list — `.claude/skills/address-findings/SKILL.md:78-81,117-119`
- [ ] (suggestion) `## Implementation` doubles as orchestrator routing key; ambiguous once a future `implement` skill also writes it — flag for #492 — `.claude/skills/address-findings/SKILL.md:32-36`
- [ ] (suggestion) Internal tension: "stage only the files for this finding" vs box-check riding in same commit — `.claude/skills/address-findings/SKILL.md:83-94`
- [ ] (suggestion) `progress_read.py --type "Integrated Review"` also matches legacy `## External Review`; take last entry whose base type is Integrated Review — `.claude/skills/address-findings/SKILL.md:50-61`
- [ ] (suggestion) Step 3 bare `git commit` vs step 5 `git -C <worktree>` with undefined `<worktree>` placeholder — `.claude/skills/address-findings/SKILL.md:87,126`

### Notes
- Cross-confirmed clean: dispatcher exit-contract is **not** confused by the shared `## Implementation` type — both adversarial passes independently verified the count-delta (`entry_count` PRE vs POST) mechanism in `dispatch_subagent.sh`.
- Scope-E (no auto-chaining) and commit-identity/no-`--no-verify` verified consistent across `triage-reviews`, `address-findings`, and `skill_workflows.md`. Adapters (3/3), lifecycle diagram + both tables, and the dispatcher mapping all updated correctly.
