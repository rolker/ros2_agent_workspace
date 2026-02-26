# Principles Review Guide

How to evaluate work against workspace principles and ADRs. Read this during
issue triage, planning, or PR review. Skills reference specific tables below;
humans use it as a checklist.

## Principle Quick Reference

| Principle | Adherence looks like | Watch for |
|---|---|---|
| Human control and transparency | PR explains what, how, and why; no hidden side effects; controls are configurable | Unexplained automation, invisible state changes, choices without rationale |
| Enforcement over documentation | New rule has a hook, CI check, or guardrail — not just markdown | Rules that exist only in docs; bypassing existing enforcement |
| Capture decisions, not just implementations | Design choice recorded in ADR or issue; rationale survives the author | Decisions buried in commit messages or chat; "why" missing from the record |
| A change includes its consequences | Tests, docs, and dependent references updated in the same PR | "Follow-up" items for stale docs or broken tests; partial changes merged |
| Only what's needed | Minimal files, process, and resource use; solves a concrete pain | Speculative features, bloated context, unnecessary abstraction, premature tooling |
| Improve incrementally | Small, reviewable change; workspace is better after than before | Large rewrites, all-or-nothing PRs, scope creep beyond the issue |
| Test what breaks | Tests target regressions that matter — timing, sensors, degraded conditions | Coverage-chasing, testing framework glue, no tests for risky logic |
| Workspace vs. project separation | Workspace infra is generic ROS 2; project content in project repos | Project-specific config leaking into workspace; workspace depending on a project |
| Workspace improvements cascade to projects | New workflow pattern is portable; workspace is the reference implementation | Improvements locked to one repo; patterns that can't be adopted downstream |
| Primary framework first, portability where free | Full use of active framework; rules naturally portable are expressed portably | Hobbling the primary tool for hypothetical frameworks; framework lock-in where avoidable |

## ADR Applicability

| ADR | Triggered when | Key requirement |
|---|---|---|
| 0001 — Adopt ADRs | A design decision is made that future agents/humans need to understand | Record in `docs/decisions/` with context, decision, and consequences |
| 0002 — Worktree isolation | Any feature work begins | Use worktree, not branch switch; enforced by `env.sh` guardrail |
| 0003 — Project-agnostic workspace | Adding content to the workspace repo | Content must be generic ROS 2 infra, not project-specific |
| 0004 — Enforcement hierarchy | A new compliance rule is proposed | Enforce at multiple layers (instructions → hooks → CI); no single layer sufficient |
| 0005 — Layered enforcement | Adding or modifying enforcement | CI/branch protection is authoritative; pre-commit provides local feedback; framework hooks provide early feedback |
| 0006 — Shared AGENTS.md | Changing agent instructions | Shared rules in `AGENTS.md`; framework adapters are thin wrappers |

## Consequences Map

| If you change... | Also update... |
|---|---|
| A principle in `docs/PRINCIPLES.md` | This review guide; any skills that reference the principle by name |
| An ADR in `docs/decisions/` | This review guide's ADR table |
| `AGENTS.md` | Framework adapters if affected (`.github/copilot-instructions.md`, etc.) |
| A script in `.agent/scripts/` | Script reference table in `AGENTS.md`; `Makefile` if it has a target |
| A template in `.agent/templates/` | Docs that reference the template; skills that use it |
| A skill in `.claude/skills/` | `CLAUDE.md` references if applicable; regenerate skills if needed |
| Package interfaces (`.msg`/`.srv`) | Downstream packages, documentation, tests |
| Worktree scripts | `WORKTREE_GUIDE.md`; `AGENTS.md` worktree section |

## Governance Layering

Workspace and project repos may both have principles, ADRs, and agent guides.
When both exist:

- **Workspace principles** govern *process* — how work is done (worktrees,
  PR workflow, documentation standards)
- **Project principles** govern *domain* — what is built (sensor behavior,
  mission logic, safety constraints)
- **Project rules take precedence** on domain questions
- **Both apply**; neither is optional
- When they conflict on process, flag it — don't silently pick one
