# Plan: docker_run_agent.sh: shield worktree build artifacts from host contamination

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/602

## Context

`docker_run_agent.sh` punches anonymous-volume "holes" through its workspace
bind mount to give containers private `build/`, `install/`, and `log/` scratch
space in `layers/main/*_ws`. That loop was never extended to the dispatched
worktree's layer workspace, because worktrees didn't exist when the loop was
written.

`WORKTREE_PATH` is already resolved by line 244 before the mount block begins.
Inside a workspace worktree, `$WORKTREE_PATH/*_ws` contains only the
`.workspace-worktrees` root (no real layer workspaces), so those are no-ops.
Inside a layer worktree, one `*_ws` dir is real (the `--layer` target) and the
others are symlinks into `layers/main` which are already shielded by the
existing loop. The `-L` symlink guard prevents `mkdir -p` from reaching through
symlinks into `layers/main`.

## Approach

1. **Add worktree-path shield loop to `docker_run_agent.sh`** — after the
   existing section 4 loop (line ~357), insert a parallel block that iterates
   `"$WORKTREE_PATH"/*_ws`, skips non-directories and symlinks, and appends
   anonymous-volume mounts for `build/`, `install/`, and `log/`. Mirror the
   existing loop's `mkdir -p` + loud-fail pattern exactly.

2. **Add inline code comments** — capture the symlink-guard rationale (`-L`
   prevents reaching through symlinks into already-shielded `layers/main`)
   and the "only dispatched worktree, not all worktrees" design decision
   (334 paths vs. 3) in-line so the reasoning survives beyond the issue body.

3. **Write a test for mount-argument generation** — add
   `.agent/scripts/tests/test_docker_run_mount_args.sh`. The test creates a
   temp directory tree mimicking a layer worktree (`*_ws/` real dirs + a
   symlink), then sources the minimal mount-logic from the script (or calls the
   script with a `--print-mounts` flag if that's cleaner) and asserts:
   - Real `*_ws` dirs get anonymous-volume `-v path` entries.
   - Symlinked `*_ws` dirs do NOT appear in mount args.
   - The `layers/main` dirs are NOT duplicated (symlink guard working).

   The cleanest approach without adding a Docker dependency to CI: add a
   `--print-mounts` dry-run flag to `docker_run_agent.sh` that dumps
   `MOUNT_ARGS[@]` and exits (no Docker needed). The test calls
   `docker_run_agent.sh --issue X --print-mounts` with `ROOT_DIR` pointed at
   a temp tree and checks the output. This also makes the mount logic
   independently inspectable going forward.

4. **File follow-up issue for image staleness check** — after the PR merges,
   file a new issue for the launcher warning (when image predates host ROS
   packages). Not in this PR.

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/docker_run_agent.sh` | Add worktree-path shield loop after section 4; add `--print-mounts` flag; add inline comments |
| `.agent/scripts/tests/test_docker_run_mount_args.sh` | New test: verifies real dirs get anonymous mounts, symlinks are skipped |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Enforcement over documentation | Existing comment said "doesn't pollute host" — this makes it true |
| A change includes its consequences | Automated test added; image-staleness check filed as follow-up |
| Only what's needed | Shield only the dispatched worktree (3 mounts), not all 334 worktree paths |
| Test what breaks | The symlink guard is subtle — unit-style test catches regressions without Docker |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0002 — Worktree isolation | Yes | Fix directly closes the worktree/container boundary gap |
| 0003 — Project-agnostic workspace | Yes | Generic infra fix; no project coupling |
| 0005 — Layered enforcement | Yes | `--print-mounts` flag makes the mount logic locally testable (local feedback layer) |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `docker_run_agent.sh` | `AGENTS.md` script table description | No — "Launch sandboxed agent container for a worktree" remains accurate; no update needed |
| `docker_run_agent.sh` | `.agent/knowledge/` note about host/container build contamination | No — knowledge-doc additions are operator-approved; flagged as candidate below |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): None — the AGENTS.md script table
  entry description ("Launch the sandboxed agent container for a worktree")
  remains accurate. The code comments added in step 2 are the durable
  record.
- **Agent-instruction candidates** (proposals only): The host/container
  build-artifact contamination pattern (host build → container build → host
  build in one worktree) is a non-obvious pitfall worth capturing in
  `.agent/knowledge/`. Defer to operator; do not add in this PR.

## Open Questions

- Should `--print-mounts` be the dry-run API, or is it cleaner to extract
  the mount-generation block into a sourced helper and test it directly?
  Either approach works; `--print-mounts` is simpler (no refactor needed)
  and makes the flag available at the command line for debugging.

## Estimated Scope

Single PR.
