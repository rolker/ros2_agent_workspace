# Plan: Improve human UX in worktrees: add local setup.bash and build.sh

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/356

## Context

Layer worktrees currently require users to either know about `worktree_enter.sh`
or manually figure out the ROS 2 source chain. The brainstorm comment expanded the
scope to include `test.sh`, `colcon/defaults.yaml`, and environment validation.

Currently, `worktree_create.sh` creates the hybrid directory structure (git worktrees
for modified packages, symlinks for others) but provides no self-contained scripts
for setup or building. The main `setup.bash` and `build.sh` exist at the workspace
level and require `ROS2_LAYERS_BASE` to be set correctly.

## Approach

### 1. Generate `setup.bash` at worktree root

At the end of `worktree_create.sh` (layer worktrees only), generate a `setup.bash`
that:
- Sources `/opt/ros/jazzy/setup.bash`
- Reads `layers.txt` (same logic as main `setup.bash`) to get layer order
- Sources each `<layer>_ws/install/setup.bash` in order using the worktree's own paths
- Exports env vars: `WORKTREE_ISSUE`, `WORKTREE_TYPE`, `WORKTREE_ROOT`, `ROS2_LAYERS_BASE`
- Installs the `git checkout` guardrail and `pre-commit` function
- Sets `GIT_EDITOR=true`

### 2. Generate `build.sh` per layer workspace

Generate `<layer>_ws/build.sh` for each layer workspace directory in the worktree
that is a real directory (not a symlink into `layers/main/`). Symlinked layer
workspaces are intentionally skipped to avoid polluting `layers/main/`. Each script:
- Sources the ROS 2 base + all lower layers' `install/setup.bash` in order
- Runs `colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`
- Accepts optional package names (`./build.sh pkg1 pkg2` → `--packages-select pkg1 pkg2`)
- Validates that `AMENT_PREFIX_PATH` includes expected layers before building

This calls `colcon` directly rather than wrapping `make build` because:
- Worktrees don't have the `.make/` stamp directory (it's gitignored)
- The purpose is a self-contained, zero-setup build — Make adds indirection
- ADR-0007's stamp-file dependencies solve a different problem (first-run setup ordering)

### 3. Generate `test.sh` per layer workspace

Same pattern as `build.sh` but runs `colcon test` + `colcon test-result --verbose`.
Accepts optional package names.

### 4. Generate `colcon/defaults.yaml` per layer workspace

Create `<layer>_ws/colcon/defaults.yaml` with:
```yaml
build:
  symlink-install: true
  cmake-args:
    - -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

This uses colcon's native configuration mechanism. The generated `build.sh` and
`test.sh` benefit from this automatically, and users running raw `colcon` in the
right directory also get correct flags.

### 5. Update `worktree_enter.sh` helpful commands output

Update the "Helpful commands" section to reference the generated scripts instead
of raw `colcon` commands.

### 6. Update documentation

Update `WORKTREE_GUIDE.md` and `AGENTS.md` to document the generated scripts.

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/worktree_create.sh` | Add generation functions for `setup.bash`, `build.sh`, `test.sh`, and `colcon/defaults.yaml` at end of layer worktree creation |
| `.agent/scripts/worktree_enter.sh` | Update "Helpful commands" to reference `./build.sh`, `./test.sh` |
| `.agent/WORKTREE_GUIDE.md` | Document generated scripts, usage examples, directory layout |
| `AGENTS.md` | Update worktree section and build instructions to mention generated scripts |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Enforcement over documentation | Generated scripts enforce correct sourcing — users can't forget to set `ROS2_LAYERS_BASE` |
| A change includes its consequences | WORKTREE_GUIDE.md and AGENTS.md updated in the same PR |
| Only what's needed | Four small generated files per worktree; each solves a concrete pain point |
| Improve incrementally | Additive change — existing workflow still works; new scripts are optional convenience |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0002 — Worktree isolation | Yes | Generated scripts encode worktree-specific paths; no cross-worktree references |
| 0007 — Retain Make | Yes | `build.sh` calls `colcon` directly, not `make`. Justified: worktrees lack `.make/` stamps and the goal is zero-setup convenience. Make remains the orchestrator for the main workspace |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `.agent/scripts/worktree_create.sh` | `AGENTS.md` script reference, `WORKTREE_GUIDE.md` | Yes (steps 5-6) |
| Worktree directory layout (new files) | `WORKTREE_GUIDE.md` directory diagrams | Yes (step 6) |

## Open Questions

- **Workspace worktrees**: Should workspace-type worktrees also get a `setup.bash`?
  They don't have layer workspaces. The brainstorm didn't address this. Recommend
  skipping for now — workspace worktrees are for infrastructure work, not building.
- **Existing worktrees**: Generated scripts only appear in newly created worktrees.
  This is acceptable — document the limitation. Users can recreate worktrees if needed.

## Estimated Scope

Single PR. ~200 lines of new generation code in `worktree_create.sh`, ~30 lines of
doc updates across two files, ~10 lines of update in `worktree_enter.sh`.
