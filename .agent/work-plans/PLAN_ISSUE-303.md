# Plan: Finish unifying project_knowledge / .agents/workspace-context path references

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/303

## Context

PR #284 unified project agent directories under `.agents/` and established
`.agents/workspace-context/` as the on-disk location in project/manifest repos.
A symlink `.agent/project_knowledge/` points to it from the workspace side.
However, documentation and skills still reference both paths inconsistently.

**Canonical convention** (already established by `setup.sh` and `gather-project-knowledge`):
- `.agents/workspace-context/` — the real directory in project/manifest repos (git-tracked there)
- `.agent/project_knowledge/` — workspace-side symlink (gitignored), created by `setup.sh`

Both paths are valid for *reading* in the workspace. The distinction is about
*where content lives* vs *how the workspace accesses it*.

## Approach

1. **Update instruction files** — In each file's "References" section, change the
   `.agent/project_knowledge/` entry to explain the relationship clearly:
   the symlink points to the active manifest repo's `.agents/workspace-context/`.

2. **Update skill read-path references** — Skills that say "read
   `.agent/project_knowledge/`" should keep that path (it's correct for reading
   in the workspace) but add a parenthetical noting it's a symlink to
   `.agents/workspace-context/`.

3. **Update ADR-0003** — Replace the 2 mentions of `.agent/project_knowledge/`
   with text that names both paths and explains the relationship.

4. **Update ARCHITECTURE.md** — The directory tree and knowledge section already
   explain the relationship well; just ensure terminology is consistent.

5. **No changes to `setup.sh`** — The script already implements the correct
   convention. No functional changes needed.

## Files to Change

| File | Change |
|------|--------|
| `AGENTS.md:253` | Update reference to explain symlink relationship |
| `CLAUDE.md:31` | Mirror AGENTS.md change |
| `.github/copilot-instructions.md:55` | Mirror AGENTS.md change |
| `.agent/instructions/gemini-cli.instructions.md:41` | Mirror AGENTS.md change |
| `.agent/AGENT_ONBOARDING.md` | Check for references, update if found |
| `docs/decisions/0003-...md:42,66` | Name both paths, explain relationship |
| `ARCHITECTURE.md:60,140` | Ensure consistent terminology |
| `.claude/skills/brainstorm/SKILL.md:30-31` | Clarify symlink in read-path references |
| `.claude/skills/review-issue/SKILL.md:53` | Clarify symlink in read-path reference |
| `.claude/skills/review-pr/SKILL.md:58` | Clarify symlink in read-path reference |
| `.claude/skills/research/SKILL.md:47` | Clarify symlink in read-path reference |
| `.claude/skills/audit-project/SKILL.md:104` | Clarify symlink reference |

**No changes needed** (already use correct paths):
- `.claude/skills/gather-project-knowledge/SKILL.md` — correctly uses `.agents/workspace-context/`
- `.agent/scripts/setup.sh` — implements the convention correctly

## Principles Self-Check

| Principle | Consideration |
|---|---|
| A change includes its consequences | All files referencing either path are included; ADR-0003 is included (was missing from original issue) |
| Only what's needed | Pure documentation consistency — no functional changes, no new abstractions |
| Enforcement over documentation | No enforcement mechanism needed for path naming — this is documentation/convention |
| Workspace vs. project separation | The change clarifies the separation: workspace reads via symlink, project repos own the real directory |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0003 — Project-agnostic workspace | Yes | ADR text itself is updated to reflect the unified convention |
| 0006 — Shared AGENTS.md | Yes | AGENTS.md change is mirrored to all framework adapters |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `AGENTS.md` | Framework adapters | Yes |
| A skill | Framework adapter skill lists | No — skill names/descriptions unchanged |

## Open Questions

None — the canonical convention is already established by working code (`setup.sh`,
`gather-project-knowledge`). This plan just aligns documentation with reality.

## Estimated Scope

Single PR, ~12 files, documentation-only changes.
