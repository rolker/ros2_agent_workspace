# Plan: Document skill lifecycle in a centralized location

## Issue

**Issue**: #302
https://github.com/rolker/ros2_agent_workspace/issues/302
**Status**: Draft

## Context

The workspace has 8 governance skills and ~23 `make_*` utility skills. Two governance
skills (`plan-task`, `brainstorm`) have lifecycle position labels; six do not. No
centralized document lists available skills or their workflow position.

Per the owner's [comment on #302](https://github.com/rolker/ros2_agent_workspace/issues/302#issuecomment-3973118121),
[#305](https://github.com/rolker/ros2_agent_workspace/issues/305) will point other
frameworks to these skills, so the lifecycle document should be in a framework-neutral
location (`.agent/knowledge/`), not under `.claude/`.

## Approach

1. **Create `.agent/knowledge/skill_workflows.md`** — A single document with:
   - The per-issue lifecycle sequence (brainstorm → review-issue → plan-task → implement → review-pr)
   - A table of all governance skills: name, lifecycle position, one-line purpose
   - A short note that `make_*` skills are auto-generated Makefile wrappers (not listed individually)
   - Distinction between lifecycle skills (per-issue) and utility skills (periodic/on-demand)

2. **Add lifecycle labels to 6 skills** — Add a `**Lifecycle position**:` line to:
   - `review-issue/SKILL.md` — position: brainstorm → **review-issue** → plan-task
   - `review-pr/SKILL.md` — position: implement → **review-pr** → (done)
   - `research/SKILL.md` — utility, not per-issue
   - `gather-project-knowledge/SKILL.md` — utility, not per-issue
   - `audit-workspace/SKILL.md` — utility/periodic
   - `audit-project/SKILL.md` — utility/periodic

3. **Add reference to `CLAUDE.md`** — Add the new doc to the References section.

## Files to Change

| File | Change |
|------|--------|
| `.agent/knowledge/skill_workflows.md` | **New** — lifecycle sequence + skill index table |
| `.claude/skills/review-issue/SKILL.md` | Add lifecycle position label after Overview heading |
| `.claude/skills/review-pr/SKILL.md` | Add lifecycle position label after Overview heading |
| `.claude/skills/research/SKILL.md` | Add lifecycle position label (utility) |
| `.claude/skills/gather-project-knowledge/SKILL.md` | Add lifecycle position label (utility) |
| `.claude/skills/audit-workspace/SKILL.md` | Add lifecycle position label (utility/periodic) |
| `.claude/skills/audit-project/SKILL.md` | Add lifecycle position label (utility/periodic) |
| `CLAUDE.md` | Add skill_workflows.md to References section |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Only what's needed | One lean document (~30 lines), not a comprehensive guide. Labels are one-liners. |
| Capture decisions, not just implementations | The lifecycle sequence currently exists only in an issue comment — this captures it durably. |
| A change includes its consequences | CLAUDE.md reference update is included in scope. |
| Primary framework first, portability where free | Placing in `.agent/knowledge/` makes it framework-neutral per owner guidance on #305. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0006 — Shared AGENTS.md | Partially — adding framework-neutral doc | Placed in `.agent/knowledge/`, referenceable from any adapter. Not modifying AGENTS.md itself. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Framework skills (`.claude/skills/`) | CLAUDE.md | Yes — adding reference |
| `.agent/knowledge/` files | Knowledge README if one exists | Yes — will check |

## Open Questions

None — scope is clear from issue + review comments.

## Success Criteria

- `.agent/knowledge/skill_workflows.md` exists with lifecycle sequence and skill index
- All 8 governance skills have a `**Lifecycle position**:` label in their SKILL.md
- `CLAUDE.md` references the new document
- `.agent/knowledge/README.md` updated if it indexes knowledge files

## Estimated Scope

Single PR, ~8 files changed, all small edits.
