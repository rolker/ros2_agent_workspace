# Plan: fix: layer worktree LD_LIBRARY_PATH shadowed by main tree install

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/484

## Context

[#428](https://github.com/rolker/ros2_agent_workspace/pull/428) added a `_wt_path_prepend` block to the generated `setup.bash` that force-prepends the worktree's target-layer paths for `AMENT_PREFIX_PATH` and `PYTHONPATH`. That closed the Python-side variant of the shadow bug.

The C++-side variant survived: `LD_LIBRARY_PATH` isn't in the override block, so the dynamic linker still finds main's stale `<pkg>.so` before the worktree's freshly-built one. This breaks POST_BUILD generator binaries (the dlopen-on-build case that hit [`rolker/unh_marine_navigation#27`](https://github.com/rolker/unh_marine_navigation/pull/27) earlier today) and would also affect `pluginlib` resolution at runtime for any downstream consumer of a worktree-modified `.so`.

Scope confirmed via [the review-issue comment](https://github.com/rolker/ros2_agent_workspace/issues/484#issuecomment-4536226615): only `LD_LIBRARY_PATH` needs to be added. `AMENT_PREFIX_PATH` is already covered by #428 (verified by reading the existing block in a real worktree's `setup.bash`). `CMAKE_PREFIX_PATH` has no concrete repro — defer.

## Approach

1. **Extend the override block in `worktree_create.sh::generate_worktree_scripts()`** — inside the existing per-package `for _wt_pkg_build in …` loop, after the `AMENT_PREFIX_PATH` branch, add two `_wt_path_prepend LD_LIBRARY_PATH` calls:
   - `<install>/<pkg>/lib` — covers post-install runtime resolution
   - `<build>/<pkg>` (the loop variable's directory itself) — covers POST_BUILD generator binaries that dlopen sibling `.so` before install happens
   Both go through the existing `_wt_path_prepend` helper so they inherit its idempotency guarantee.

2. **Extend test 21 (`test_layer_setup_bash_has_path_priority_block`)** in `test_worktree_create.sh` — add one `grep -Fq "_wt_path_prepend LD_LIBRARY_PATH"` assertion so the symmetry with `AMENT_PREFIX_PATH` / `PYTHONPATH` is enforced going forward.

3. **Extend test 23 (`test_layer_setup_bash_idempotent_resourcing`)** — fabricate a `<build>/<pkg>` and `<install>/<pkg>/lib` pair, source the extracted block twice, and assert `LD_LIBRARY_PATH` contains each entry exactly once. Mirrors the existing `ament_count` / `py_sp_count` / `py_build_count` checks.

4. **Validate against the original repro** — recreate the unh_marine_navigation issue-26 worktree fresh, run `./core_ws/build.sh marine_nav_behavior_tree` without manual `LD_LIBRARY_PATH` munging, confirm POST_BUILD generator succeeds. This is the acceptance gate from the issue.

5. **Sanity check `WORKTREE_GUIDE.md` and `AGENTS.md`** — both already omit any mention of the path-priority block (verified). No doc updates needed; consistent with how #428 landed.

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/worktree_create.sh` | In the `# 5. Prioritize worktree target layer…` heredoc, add `LD_LIBRARY_PATH` prepends inside the per-package loop (~5 lines: empty check for `<install>/lib`, two `_wt_path_prepend` calls, brief comment) |
| `.agent/scripts/tests/test_worktree_create.sh` | Extend test 21 with one `grep -Fq "_wt_path_prepend LD_LIBRARY_PATH"` assertion. Extend test 23: pre-create `<install>/<pkg>/lib`, set `LD_LIBRARY_PATH=''` in the sub-bash, capture/check `ld_count` for both `lib` and `build/<pkg>` entries |
| `.agent/work-plans/issue-484/plan.md` | This plan |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Enforcement over documentation | Fix is automatic in generated `setup.bash` — no manual workaround. Mirrors #428's mechanism exactly. |
| A change includes its consequences | Test parity covered (assertion + idempotency). No doc updates needed (#428 didn't update WORKTREE_GUIDE.md or AGENTS.md either; the override block is internal mechanism, not user-facing API). Existing worktrees won't auto-update — will note in PR description that users with active C++ worktrees should recreate (`worktree_remove` + `worktree_create`). |
| Only what's needed | Two lines of generator code + assertion parity. Resisted the issue's "while we're in there, check AMENT_PREFIX_PATH and CMAKE_PREFIX_PATH" — `AMENT_PREFIX_PATH` is already handled by #428; `CMAKE_PREFIX_PATH` has no concrete repro. |
| Improve incrementally | Small additive change on top of existing block; no refactor. |
| Test what breaks | Two test extensions: (a) assertion that the env var is wired through the helper, (b) functional idempotency. Real-world acceptance test is the unh_marine_navigation#27 reproducer. |
| Workspace improvements cascade to projects | All future layer worktrees benefit automatically on next `worktree_create`. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0002 (worktree isolation) | Yes | This fix is exactly what isolation means in practice — the worktree's freshly built `.so` should win over the main tree's stale install. Strengthens the principle. |
| ADR-0004 (enforcement hierarchy) | Yes | Fix is at the generated-`setup.bash` layer (same as #428); the regression tests in `test_worktree_create.sh` are the load-bearing defense. No CI gate beyond the existing test runner needed. |
| ADR-0008 (ROS 2 conventions) | No | Works *with* colcon's `LD_LIBRARY_PATH` ordering, not against it. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `worktree_create.sh` | `.agent/WORKTREE_GUIDE.md` per the consequences map | Not needed — the path-priority block isn't documented there (verified); #428 didn't add it either. Including for consistency would be over-scoping. |
| `worktree_create.sh` | `AGENTS.md` worktree section | Not needed — same reason. |
| `worktree_create.sh::generate_worktree_scripts()` | `test_worktree_create.sh` tests 21 + 23 | Yes — step 2 + step 3 |
| Generated `setup.bash` format | Existing worktrees don't auto-update | Document in PR description; manual workaround (prepend `LD_LIBRARY_PATH` before `colcon build`) still works for the interim |

## Open Questions

- None. The scope, mechanism, and tests are all bounded by #428's precedent.

## Estimated Scope

Single PR. ~10–15 lines of generator code + ~10 lines of test extensions. No design decisions remain; this is a mechanical mirror of #428 for the C++ env var.
