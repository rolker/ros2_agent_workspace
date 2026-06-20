---
issue: 530
---

# Issue #530 — v1 `/wrap-up-deployment` skill — phase [3] orchestrator (deployment-mode gap)

## Issue Review
**Status**: partial
**When**: 2026-06-20 00:00 +00:00
**By**: Claude Code Agent (Claude Opus 4.6)

**Issue**: #530
**Comment**: not posted — `gh` CLI lacks auth in this container environment (no GH_TOKEN, no `~/.config/gh/`). Review content saved to `.agent/work-plans/issue-530/review-comment.md` for the host to post.
**Scope verdict**: well-scoped

### Review Summary

Issue proposes creating `.claude/skills/wrap-up-deployment/SKILL.md` + AGENTS.md pointer update, codifying a 4-times-validated manual deployment wrap-up workflow. Single PR, clear acceptance criteria, no blocking dependencies.

**Principle findings** (Action needed):
- A change includes its consequences: Non-Claude adapter skill lists (`.github/copilot-instructions.md`, `.agent/instructions/gemini-cli.instructions.md`, `.agent/AGENT_ONBOARDING.md`) and `make generate-skills` are not called out in acceptance criteria but are required by the consequences map.

**ADR findings** (Action needed):
- ADR-0006: Non-Claude adapter skill lists need updating alongside AGENTS.md.
- ADR-0013: SKILL.md must specify which `progress.md` entry type the skill writes (or confirm it does not write one). If a new type is needed, an ADR addendum is required before implementation.

**Watch items**:
- ADR-0003: SKILL.md must not embed project-specific content; all project details must come from `.agents/deployment.yaml`.
- ADR-0014: ADR Status is "Proposed" — updating to "Accepted" after this skill ships is a follow-up.

### Actions
- [ ] Update non-Claude adapter skill lists when adding the skill (`.github/copilot-instructions.md`, `.agent/instructions/gemini-cli.instructions.md`, `.agent/AGENT_ONBOARDING.md`)
- [ ] Run `make generate-skills` after adding `.claude/skills/wrap-up-deployment/`
- [x] **RESOLVED** (operator, 2026-06-20): skill writes **NO `progress.md` entry**. `/wrap-up-deployment` is a deployment-lifecycle skill (writes the deployment dev log + GitHub issue), mirroring `/start-deployment`; `progress.md`/ADR-0013 govern the per-issue *development* pipeline only. No ADR-0013 addendum needed.
- [ ] Keep project-specific config in `.agents/deployment.yaml`, not in SKILL.md (ADR-0003 three-tier split)
- [x] Posted by host (gh auth available on host) — #530 issue comment.
- [ ] After merge: consider updating ADR-0014 Status from "Proposed" to "Accepted"

## Plan Authored
**Status**: complete
**When**: 2026-06-20 10:30 +00:00
**By**: Claude Code Agent (Claude Sonnet 4.6)

**Plan**: `.agent/work-plans/issue-530/plan.md` at `7cc1133`
**Branch**: feature/issue-530 at `7cc1133`
**Phases**: single

### Open questions
- [ ] Should `.agents/deployment.yaml` gain a `wrap_up:` section for project-specific steps (bag extraction commands, analysis scripts)? V1 defers bag extraction to operator/debrief (#435); out of scope unless a project has concrete needs.

## Plan Review
**Status**: complete
**When**: 2026-06-20 13:34 +00:00
**By**: Claude Code Agent (Claude Opus 4.6) (in-context — author self-review)

**Plan**: `.agent/work-plans/issue-530/plan.md` at `7cc1133`
**PR**: PR-less (--issue mode; dispatched sub-agent review)
**Verdict**: approve-with-suggestions

### Findings
- [ ] (suggestion) Make `issue_sync`/`dev_push` absence graceful in SKILL.md — dev-side optional per `deployment_config.yaml:79-81`; mirror `start-deployment` — `plan.md:40`
- [ ] (suggestion) Open the ADR-0014 Status-flip ("Proposed"→"Accepted") follow-up issue rather than leaving it implicit — `plan.md:102`
- [ ] (suggestion) State bag-extraction/analysis as operator-driven / out-of-scope for v1 in SKILL.md so adopters see the deferral — `plan.md:108-112`

Plan is well-scoped, addresses all review-issue findings, and correctly resolves the ADR-0003 (project-agnostic config) and ADR-0013 (no per-issue progress entry) risks. Config keys referenced (`log_dir`, `issue_sync.dev_push`) verified against the template. Ready for implementation; the three items are refinements, not blockers.

## Implementation
**Status**: complete
**When**: 2026-06-20 (sub-agent)
**By**: Claude Code Agent (Claude Sonnet 4.6)

Created `.claude/skills/wrap-up-deployment/SKILL.md` — a 9-step wrap-up orchestrator mirroring `start-deployment/SKILL.md` in structure and tone. The skill adds an operator-correction interview (step 5, one question at a time before consolidating logs) and separates field-code reconciliation into two distinct mechanisms: SHA-preserving `git merge` for the primary deployment repo (step 7a) and `/import-field-changes` for other repos (step 7b). Bag extraction is explicitly deferred to the project debrief checklist (#435). `issue_sync`/`dev_push` absence is graceful (dev-side optional, mirroring start-deployment).

Also updated AGENTS.md "Deployment mode" section (replaced #495 placeholder with skill pointer + #496 recovery note), all three non-Claude adapter skill lists, and plan.md to reflect the 9-step implemented flow. `make generate-skills` ran successfully (generated `.claude/skills/make_*/` files which are gitignored).

**Host follow-up**: ADR-0014 Status "Proposed"→"Accepted" flip requires `gh` auth to post — post once the PR merges to main.

**Files changed**:
- `.claude/skills/wrap-up-deployment/SKILL.md` — created (main deliverable)
- `AGENTS.md` — deployment mode section updated
- `.github/copilot-instructions.md` — `wrap-up-deployment` added to skill list
- `.agent/instructions/gemini-cli.instructions.md` — `wrap-up-deployment` added to skill list
- `.agent/AGENT_ONBOARDING.md` — `wrap-up-deployment` added to skill list
- `.agent/work-plans/issue-530/plan.md` — synced to reflect 9-step flow
