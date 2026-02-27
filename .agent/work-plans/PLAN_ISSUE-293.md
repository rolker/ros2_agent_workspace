# Plan: Document env.sh requirement for per-package testing

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/293

## Context

The Build & Test section in `AGENTS.md` and the Testing section in
`.agent/knowledge/ros2_development_patterns.md` show `colcon test` as a
standalone command. AI agents run each command in an isolated subprocess
(fresh shell), so without `env.sh` sourced in the same shell, `colcon test`
fails with `ModuleNotFoundError`. Agents waste multiple attempts before
discovering they need to chain environment sourcing with the test command.

## Approach

1. **Update `AGENTS.md` Build & Test section** — Replace the standalone
   `colcon test` line with a chained command that sources `env.sh` in the
   same shell. Keep the `colcon build` line as-is (it already chains with
   `cd`). Add a comment noting `make test` handles env automatically.

2. **Update `.agent/knowledge/ros2_development_patterns.md` Testing section**
   — Replace the standalone test commands with a chained version that
   sources `env.sh` and `cd`s into the layer directory. Add a note about
   `make test` for full workspace runs.

## Files to Change

| File | Change |
|------|--------|
| `AGENTS.md` (lines 145-146) | Chain `env.sh` sourcing with `colcon test`; add `make test` note |
| `.agent/knowledge/ros2_development_patterns.md` (lines 103-115) | Chain `env.sh` sourcing with test commands; add `make test` note |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| A change includes its consequences | Both files with the problematic pattern are updated in this plan. Framework adapters don't duplicate these commands — no cascading updates needed. |
| Enforcement over documentation | This is a docs fix. `make test` already handles env sourcing for full runs. A per-package helper script could enforce this further but is out of scope for this issue. |
| Improve incrementally | Small, focused two-file edit. |
| Only what's needed | Solves a concrete, recurring agent pain point. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0006 — Shared AGENTS.md | Yes | Framework adapters checked — they don't reference `colcon test` directly, so no adapter updates needed. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `AGENTS.md` | Framework adapters if affected | Yes — verified not affected |

## Open Questions

None — the approach is straightforward and the review confirmed no blocking concerns.

## Estimated Scope

Single PR, two file edits.
