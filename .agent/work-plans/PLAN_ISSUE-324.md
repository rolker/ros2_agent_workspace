# Plan: Rename env.sh to setup.bash and setup.sh to setup_layers.sh

**Issue**: #324
**Status**: Draft

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/324

## Context

`.agent/scripts/env.sh` sources the ROS 2 environment and contains the `git checkout`
guardrail. The owner wants it renamed to `setup` to mirror ROS workspace conventions
(e.g. `install/setup.bash`, `install/setup.sh`).

A naming conflict exists: `.agent/scripts/setup.sh` already exists as the layer
initialization script (clones repos via vcstool). It must be renamed to
`setup_layers.sh` first. This creates two sequential phases in one PR.

Decisions from issue thread:
- Existing `setup.sh` → `setup_layers.sh`
- `env.sh` → `setup.bash` (primary); `setup.sh` → symlink to `setup.bash`
- No compatibility shim for the old `env.sh` path

## Approach

### Phase 1 — Rename `setup.sh` → `setup_layers.sh`

1. **Rename the file** — `git mv .agent/scripts/setup.sh .agent/scripts/setup_layers.sh`
2. **Update internal header comment** in `setup_layers.sh` (filename reference on line 2)
3. **Update all 14 external references** (see Files to Change)

### Phase 2 — Rename `env.sh` → `setup.bash` + symlink `setup.sh`

4. **Rename the file** — `git mv .agent/scripts/env.sh .agent/scripts/setup.bash`
5. **Update internal header comment** in `setup.bash` (filename reference on line 2)
6. **Create symlink** — `ln -s setup.bash .agent/scripts/setup.sh && git add .agent/scripts/setup.sh`
7. **Update all 7 external references** (see Files to Change)

### Verification

8. **Smoke-test** — `source .agent/scripts/setup.bash` and `source .agent/scripts/setup.sh`
   both succeed; confirm ROS environment is active and `git checkout main` is blocked

## Files to Change

### Phase 1 — `setup.sh` → `setup_layers.sh`

| File | Change |
|------|--------|
| `.agent/scripts/setup.sh` | Rename to `setup_layers.sh`; update line 2 header comment |
| `Makefile` | 6 calls to `setup.sh` → `setup_layers.sh` |
| `README.md` | 4 references |
| `QUICKSTART.md` | 3 references (lines 66, 137, 140, 176) |
| `CONTRIBUTING.md` | 2 references (lines 20, 164) |
| `AGENTS.md` | 1 reference in script reference table |
| `CLAUDE.md` | 1 reference |
| `.github/copilot-instructions.md` | 1 reference |
| `.agent/instructions/gemini-cli.instructions.md` | 1 reference |
| `.agent/scripts/README.md` | ~8 references |
| `.agent/scripts/bootstrap.sh` | 1 reference |
| `.agent/scripts/health_check.sh` | 1 reference |
| `.agent/scripts/env.sh` | 1 internal comment reference |
| `.agent/knowledge/README.md` | 1 reference |
| `ARCHITECTURE.md` | ~2 references |
| `.claude/skills/research/SKILL.md` | 1 reference |
| `docs/decisions/0003-workspace-infrastructure-is-project-agnostic.md` | 1 reference |

### Phase 2 — `env.sh` → `setup.bash` + `setup.sh` symlink

| File | Change |
|------|--------|
| `.agent/scripts/env.sh` | Rename to `setup.bash`; update line 2 header comment |
| `.agent/scripts/setup.sh` | New symlink → `setup.bash` |
| `AGENTS.md` | Script table + inline examples (`env.sh` → `setup.bash`) |
| `CLAUDE.md` | Environment setup section |
| `.github/copilot-instructions.md` | Environment setup section |
| `.agent/instructions/gemini-cli.instructions.md` | Environment setup section |
| `QUICKSTART.md` | 2 references (lines 108, 114) |
| `ARCHITECTURE.md` | 2 references |
| `README.md` | References |
| `.agent/AGENT_ONBOARDING.md` | References |
| `.agent/WORKTREE_GUIDE.md` | References |
| `.agent/knowledge/principles_review_guide.md` | 1 reference |
| `.agent/knowledge/ros2_development_patterns.md` | ~3 references |
| `.agent/scripts/README.md` | References (also updated in Phase 1) |
| `.agent/scripts/build.sh` | References |
| `.agent/scripts/setup_layers.sh` | 1 internal comment reference |
| `.agent/scripts/test.sh` | References |
| `.agent/scripts/worktree_create.sh` | References |
| `.agent/scripts/worktree_enter.sh` | References |
| `.agent/templates/project_agents_guide.md` | References |
| `.claude/skills/audit-project/SKILL.md` | References |
| `.devcontainer/agent/agent-entrypoint.sh` | References |
| `.github/instructions/ros2.instructions.md` | References |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Only what's needed | Pure rename — no new behaviour, no new tooling |
| A change includes its consequences | All references updated in the same PR; no stale docs |
| Enforcement over documentation | Checkout guardrail in `setup.bash` is preserved unchanged; smoke-test confirms it |
| Improve incrementally | Single focused PR; two phases are sequential steps within it |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0002 — Worktree isolation | Yes | Work done in `feature/issue-324` worktree |
| 0006 — Shared AGENTS.md | Yes | `AGENTS.md` and all three framework adapters updated in Phase 2 |

## Consequences

| If we change... | Also update... | Included? |
|---|---|---|
| `setup.sh` (layer script) | Makefile, README, QUICKSTART, CONTRIBUTING, AGENTS.md, CLAUDE.md, all 3 adapters, `.agent/scripts/README.md`, bootstrap.sh, health_check.sh, env.sh comment, knowledge/README.md, ARCHITECTURE.md, `.claude/skills/research/SKILL.md`, `docs/decisions/0003-...` | Yes — Phase 1 |
| `env.sh` (env script) | AGENTS.md, CLAUDE.md, copilot-instructions.md, gemini-cli.instructions.md, QUICKSTART.md, ARCHITECTURE.md, README.md, AGENT_ONBOARDING.md, WORKTREE_GUIDE.md, principles_review_guide.md, ros2_development_patterns.md, build.sh, test.sh, worktree_create.sh, worktree_enter.sh, project_agents_guide.md, audit-project/SKILL.md, agent-entrypoint.sh, ros2.instructions.md | Yes — Phase 2 |
| `.agent/work-plans/PLAN_ISSUE-303.md` | References `setup.sh` (2x) | No — historical artifact, not updated |

## Open Questions

- None. Naming decisions resolved in issue thread: `setup_layers.sh`, `setup.bash` + `setup.sh` symlink, no shim.

## Estimated Scope

Single PR — all changes are mechanical text substitutions and file renames across ~32 files.
