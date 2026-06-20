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
- [ ] Decide and document ADR-0013 entry type used by the skill before implementation starts
- [ ] Keep project-specific config in `.agents/deployment.yaml`, not in SKILL.md (ADR-0003 three-tier split)
- [ ] Post review-comment.md as GitHub issue comment (blocked by missing `gh` auth in this container)
- [ ] After merge: consider updating ADR-0014 Status from "Proposed" to "Accepted"
