# Plan: Point non-Claude framework adapters to existing workflow skills

## Issue

**Issue**: #305
**Status**: Plan — approved

## Context

The workspace has 9 workflow skills in `.claude/skills/*/SKILL.md` that are
plain markdown procedures (not Claude Code-specific). Non-Claude framework
adapters (Gemini CLI, Copilot, generic onboarding) don't reference them,
so agents on those frameworks can't discover or use them.

The review comment recommended using a glob instruction over an explicit list
to avoid staleness. However, the issue asks for an optional explicit list for
discoverability, and listing 9 names in a short block is manageable. Since the
consequences map already says "If you change a framework skill → also update
that framework's adapter file," the maintenance burden is already documented.

The user confirmed that `brand-guidelines` should be included in the list.

## Approach

1. **Add a "Workflow Skills" section to each adapter file** — Insert a short
   section pointing to `.claude/skills/*/SKILL.md` with a list of the 9
   workflow skills by name. Place it before the References section in each file.

2. **Use repo-root-relative paths** — All three files are at different depths,
   so use repo-root-relative paths (e.g., `.claude/skills/`) rather than
   relative paths to keep the text consistent across files.

3. **List the 9 workflow skills explicitly** — For discoverability, name them:
   `review-issue`, `plan-task`, `review-pr`, `brainstorm`, `research`,
   `audit-workspace`, `audit-project`, `gather-project-knowledge`,
   `brand-guidelines`. Exclude `make_*` auto-generated skills (those are
   Claude Code Makefile wrappers, not portable workflow procedures).

4. **Update the consequences map** — Add a row noting that adding/removing a
   workflow skill should also update the non-Claude adapter files. This
   complements the existing "framework skill → adapter file" entry.

## Files to Change

| File | Change |
|------|--------|
| `.agent/instructions/gemini-cli.instructions.md` | Add "Workflow Skills" section before References |
| `.github/copilot-instructions.md` | Add "Workflow Skills" section before References |
| `.agent/AGENT_ONBOARDING.md` | Add "Workflow Skills" section before References |
| `.agent/knowledge/principles_review_guide.md` | Add consequences map row for skill list maintenance |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Primary framework first, portability where free | Skills stay in `.claude/skills/`; other adapters just point there — "portability where free" |
| Only what's needed | 3-5 lines per file; one consequences map row. Minimal. |
| A change includes its consequences | Updating consequences map ensures future skill changes trigger adapter updates |
| Improve incrementally | Small, self-contained change |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0006 — Shared AGENTS.md | Yes — modifying adapter files | Consistent with ADR-0006: adapters stay thin wrappers pointing to shared resources |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Framework adapter files | Verify consistency across all adapters | Yes — all 3 updated together |
| Consequences map | Review guide stays accurate | Yes — step 4 |

## Open Questions

None — user confirmed `brand-guidelines` inclusion; approach is straightforward.

## Estimated Scope

Single PR, 4 files changed.
