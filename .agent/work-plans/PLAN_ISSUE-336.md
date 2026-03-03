# Plan: ADR-0008 — Follow ROS 2 official conventions by default

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/336

## Context

Agents repeatedly diverge from ROS 2 community conventions (launch file naming,
licensing, package.xml patterns). Two specific issues exist (#327, #131) but the
root cause is the same: no umbrella decision and no unambiguous knowledge file.

The existing `ros2_development_patterns.md` has a thin "ROS2 Conventions" section
(lines 58–70) with ambiguous guidance — e.g., "Descriptive with `.launch.py`
extension" doesn't distinguish `*_launch.py` (recommended) from `*.launch.py`.
There is no licensing guidance at all.

Owner direction (issue comment): conventions target Rolling; allow exceptions when
working in a different distro where a newer convention isn't feasible. Reference
#261 (multi-distro support).

## Approach

1. **Write ADR-0008** (`docs/decisions/0008-follow-ros2-official-conventions.md`)
   - Policy: follow ROS 2 conventions by default; deviations require their own ADR
   - Conventions target Rolling; distro-specific exceptions allowed (per owner)
   - Knowledge file is the "instructions" layer per ADR-0004; enforcement (hooks,
     CI) is separate work per ADR-0004/0005
   - Reference key sources: Developer Guide, REP-144, REP-140, launch tutorial,
     migration guide (licensing)

2. **Expand `ros2_development_patterns.md`** — rewrite the "ROS2 Conventions"
   section with explicit, unambiguous guidance on:
   - **Launch files**: `*_launch.py` suffix (recommended by ROS 2 docs); must end
     in `launch.py` for `ros2 launch` autocomplete; XML/YAML use `_launch.xml` /
     `_launch.yaml`
   - **Licensing**: Apache 2.0 recommended for new ROS 2 packages; existing
     packages preserve their declared license; always check project's LICENSE file
     and `package.xml` `<license>` tag; include copyright + license header in
     every source file; match the project's existing pattern
   - **Package naming**: lowercase with underscores (REP-144); at least 2 chars;
     avoid "utils" catchalls; don't prefix with "ros"
   - **`package.xml`**: format 3; use SPDX license identifiers; correct dependency
     tags (`<depend>` vs `<build_depend>` vs `<exec_depend>` vs `<test_depend>`)
   - **Copyright headers**: required in every source file; year, author, institution
   - **Distro note**: conventions target Rolling; note distro-specific differences
     when relevant

3. **Update `principles_review_guide.md`** — add ADR-0008 to the ADR applicability
   table (required by consequences map)

## Files to Change

| File | Change |
|------|--------|
| `docs/decisions/0008-follow-ros2-official-conventions.md` | New ADR |
| `.agent/knowledge/ros2_development_patterns.md` | Expand conventions section |
| `.agent/knowledge/principles_review_guide.md` | Add ADR-0008 to applicability table |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Enforcement over documentation | This PR is the documentation/instructions layer. ADR explicitly notes enforcement is separate work (hooks, CI) per ADR-0004/0005. |
| Capture decisions, not just implementations | The ADR captures the "why" — follow community conventions to avoid per-item debates. |
| A change includes its consequences | Knowledge file and review guide updated in the same PR. |
| Only what's needed | One ADR + one knowledge file update. No speculative enforcement hooks. |
| Workspace vs. project separation | All content is generic ROS 2, not project-specific. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0001 — Adopt ADRs | Yes | New ADR follows established format |
| 0003 — Project-agnostic | Yes | Content is generic ROS 2 conventions |
| 0004 — Enforcement hierarchy | Yes | ADR references the hierarchy; knowledge file is the "instructions" layer |
| 0005 — Layered enforcement | Yes | ADR notes enforcement is separate follow-up work |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| ADR in `docs/decisions/` | `principles_review_guide.md` ADR table | Yes (step 3) |
| Knowledge file | Docs that reference it | N/A — no docs reference specific sections |

## Open Questions

None — owner's comment resolved the distro targeting question.

## Estimated Scope

Single PR, three files changed.
