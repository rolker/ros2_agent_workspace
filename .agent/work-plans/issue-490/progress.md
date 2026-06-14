---
issue: 490
---

# Issue #490 — dispatch_subagent.sh + per-skill handoff boilerplate

## Plan Authored
**Status**: complete
**When**: 2026-06-13 17:14 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Plan**: `.agent/work-plans/issue-490/plan.md` at `24896d3`
**PR**: https://github.com/rolker/ros2_agent_workspace/pull/519 (`[PLAN]` prefix)
**Phases**: single (Scope A + B + E in one PR; #493 deferred, gated on #492)

### Open questions
- [ ] #493 sequencing: #490 adds the progress.md exit contract but leaves push_gateway in place; deletion is #493, gated on #492 — confirm the split.
- [ ] Kickoff output format: stream-json vs json vs text for the host to consume (leaning stream-json).
- [ ] PR split: dispatch+headless (A) and skill handoff blocks (B) as one PR or two (leaning one).

## Plan Review
**Status**: complete
**When**: 2026-06-13 17:49 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context)) — fresh-context sub-agent dispatch (independent of plan-author context; same agent identity)

**Plan**: `.agent/work-plans/issue-490/plan.md` at `4290908` (findings addressed)
**PR**: https://github.com/rolker/ros2_agent_workspace/pull/519
**Verdict**: changes-requested (2 must-fix + 3 suggestions; all folded into plan at `4290908`)

### Findings
- [x] (must-fix) Exit-contract under-specified: crash-before-write reads stale entry; progress_read.py has no tail selector; could match wrong skill's entry → gate on exit status + require newer entry + key on expected type — plan Approach step 3
- [x] (must-fix) 3 framework adapters not in Files-to-Change (handoff is Claude-Agent-specific) — now listed
- [x] (suggestion) Container-mode must forward AGENT_NAME/EMAIL for entrypoint --detect identity — plan Approach step 2
- [x] (suggestion) Layer-worktree container check is manual, not CI — clarified in plan Approach step 5
- [x] (suggestion) State no Makefile target for dispatch_subagent.sh — added to Consequences

## Implementation
**Status**: complete
**When**: 2026-06-14 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet 4.6)

**Scope**: Scope B — per-skill handoff blocks and knowledge doc
**Commits**: `e1d0efb` (5 skill SKILL.md files), `703e7ff` (skill_workflows.md)

### What was added

Five lifecycle skill files each received a `### Next step` block as
the last section of their procedure (before `## Guidelines`):

- `review-issue/SKILL.md` → hands off to `plan-task` (in-process); entry pair: `## Issue Review` → `## Plan Authored`
- `plan-task/SKILL.md` → hands off to `review-plan` (in-process); entry pair: `## Plan Authored` → `## Plan Review`
- `review-plan/SKILL.md` → describes two-phase hand-off: implement (container, no skill yet) then `review-code` (in-process); entry pair: `## Plan Review` → `## Local Review`
- `review-code/SKILL.md` → hands off to `triage-reviews` (in-process, after push); entry pair: `## Local Review` → `## Integrated Review`
- `triage-reviews/SKILL.md` → end-of-loop description (address findings / merge); no dispatch line needed

`.agent/knowledge/skill_workflows.md` received a new
`## Lifecycle Handoff Convention (#490)` section: entry-type table,
dispatcher usage examples, and the Scope-E no-auto-chaining rule.

### Ask-first files intentionally omitted

`.github/copilot-instructions.md`, `.agent/instructions/gemini-cli.instructions.md`,
`.agent/AGENT_ONBOARDING.md`, and `AGENTS.md` are held for human approval per
the plan's Files-to-Change "ask-first" annotation and the task prompt's Do-NOT-touch list.

### Actions
- [ ] Host: push branch and open/update PR for review
- [ ] Ask-first: add one-line "handoff is Claude-Agent-specific" note to the 3 framework adapters + AGENTS.md Script Reference row (human approval needed)
