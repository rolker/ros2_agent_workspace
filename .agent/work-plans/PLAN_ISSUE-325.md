# Plan: ADR — Retain Make with Dependency Tracking

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/325

## Context

The workspace uses a GNU Makefile with 19+ targets as a task runner. Issue #249
research evaluated `just`, `task`, and `mise` as alternatives. The owner's direction
(issue comment) reframes the question: the problem isn't Make itself, but that the
Makefile doesn't use Make's dependency model. The decision is to keep Make and use
it properly — stamp files for dependency tracking rather than migrating to a new tool.

This issue is **ADR-only** — the Makefile rewrite is tracked in #332, and the
first-run initialization UX is tracked in #330.

## Approach

1. **Write ADR `docs/decisions/0007-retain-make-with-dependency-tracking.md`**
   - Context: current Makefile is a command runner with no dependencies; #249
     research evaluated alternatives
   - Options: `just` (strong command runner, no dependency model), `task` (YAML,
     moderate fit), `mise` (polyglot, overkill), idiomatic Make with stamp files
   - Decision: retain Make, adopt stamp-file pattern for dependency tracking
   - Consequences: positive (fresh-clone `make build`, incremental re-runs, no new
     tool, `generate-skills` preserved); negative (stamp files add a hidden state
     directory, Make syntax remains awkward)
   - Reference #249 research and link to #332 for implementation

2. **Update `principles_review_guide.md`** — add ADR-0007 to the ADR Applicability
   table so future reviews know when it's triggered

3. **Verify and commit**

## Files to Change

| File | Change |
|------|--------|
| `docs/decisions/0007-retain-make-with-dependency-tracking.md` | New ADR file |
| `.agent/knowledge/principles_review_guide.md` | Add ADR-0007 row to applicability table |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Capture decisions, not just implementations | This is the principle — the entire issue is recording a decision as an ADR |
| Only what's needed | Decision rejects adding a new tool; ADR is minimal |
| A change includes its consequences | Review guide updated in the same PR |
| Improve incrementally | ADR only, implementation deferred to #332 |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0001 — Adopt ADRs | Yes | Following the format: Context, Decision, Consequences |
| 0003 — Project-agnostic | Trivially | Make is generic workspace infra |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| ADR in `docs/decisions/` | `principles_review_guide.md` ADR table | Yes (step 2) |

## Open Questions

None — the direction is clear from the issue discussion and scope is confirmed as ADR-only.

## Estimated Scope

Single PR, two files changed.
