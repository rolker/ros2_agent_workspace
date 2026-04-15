# Plan: fix layer worktree Python paths shadowed by main tree symlink-install

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/427

## Context

Layer worktrees symlink all non-target layers to the main tree. Colcon's
`setup.bash` bakes absolute `layers/main/` paths at build time, so sourcing
any symlinked higher layer re-sources the main tree's install for the target
layer. The main tree's `pythonpath_develop.dsv` hook adds its `build/<pkg>`
directory to `PYTHONPATH`, and that build directory has stale egg-info. The
worktree's own paths end up later on `sys.path`, so `pkg_resources` finds
the main tree first.

## Approach

1. **Pass `TARGET_LAYER` into `generate_worktree_scripts()`** — Add a third
   parameter so the function knows which layer is the target. Currently it
   only receives `wt_dir` and `main_root`.

2. **Append a PYTHONPATH override block to the generated `setup.bash`** —
   After the "Source workspace layers" loop and before the "Environment
   ready" message, add a block that iterates over the target layer's
   `build/*/` directories and force-prepends both the build dir (for
   egg-info) and the matching install site-packages dir to `PYTHONPATH`.
   Use `local_setup.bash`-style prepend (raw `export`) to bypass ament's
   `prepend-non-duplicate` which refuses to reorder existing entries.

3. **Only emit the block for layer worktrees** — The function already knows
   `WORKTREE_TYPE="layer"`. Guard the new block so workspace worktrees
   (which don't have a target layer) are unaffected.

4. **Regenerate the existing issue-16 worktree's `setup.bash`** — Manually
   verify the fix by re-running `worktree_create.sh` for issue 16 (or
   patching its `setup.bash` in place) and confirming `ros2 launch -h`
   shows `--tui`.

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/worktree_create.sh` | Pass `TARGET_LAYER` to `generate_worktree_scripts()`; emit PYTHONPATH override block in generated `setup.bash` |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| A change includes its consequences | Existing worktrees won't auto-update; document that users should recreate or manually patch `setup.bash`. Verify with the ros2launch_gui worktree. |
| Only what's needed | Single targeted fix — no refactoring of the script generator |
| Enforcement over documentation | The fix is automatic in generated scripts, not a manual workaround |
| Workspace improvements cascade to projects | All future layer worktrees for any project benefit |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0002 (worktree isolation) | Yes | Fix ensures worktree isolation actually works for Python packages |
| ADR-0008 (ROS 2 conventions) | No | No packaging changes; works within colcon's existing mechanisms |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Generated `setup.bash` format | Existing worktrees need recreation or manual patch | No — document in PR description |
| `generate_worktree_scripts()` signature | Call site at line ~942 | Yes |

## Open Questions

- None — approach is verified experimentally.

## Estimated Scope

Single PR, one file changed.
