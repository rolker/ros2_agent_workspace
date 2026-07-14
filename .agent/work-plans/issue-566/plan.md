# Plan: Fix docker_run_agent.sh anonymous volumes creating root-owned host dirs

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/566

## Context

`docker_run_agent.sh` adds anonymous volumes for each layer's `build/`, `install/`,
and `log/` subdirectories to isolate container build artifacts from the host. When
those directories don't exist at container start, Docker creates them as `root:root`
on the host. Any subsequent host-side colcon process writing to those paths fails with
`EACCES`.

This was observed during the #559 heal rebuild: a concurrent agent container for issue
#563 re-created the still-missing `simulation_ws/{build,install,log}` as root-owned,
blocking `make build`.

The fix is a single `mkdir -p` call inside the existing loop, which pre-creates each
mountpoint as the invoking user before Docker can claim it. Docker only creates missing
mountpoints and never chowns existing ones.

## Approach

1. **Core fix — pre-create mountpoints in `docker_run_agent.sh`** — add `mkdir -p
   "$ws_dir/$subdir"` inside the inner loop (section 4, ~line 340), immediately before
   the `-v "$ws_dir/$subdir"` anonymous-volume entry. The outer `[ -d "$ws_dir" ]`
   guard already ensures `$ws_dir` exists; `mkdir -p` for each subdir is safe to call
   unconditionally (a no-op if the dir already exists and user-owned).

2. **ADR-0016 Consequences update** — add a note to the `## Consequences` section of
   `docs/decisions/0016-runtime-vs-baked-layer-chaining.md` documenting the
   concurrent-container-dispatch collision hazard and referencing the fix. The issue
   body derives directly from the #559 incident; without this note the hazard and its
   fix exist only in issue history.

3. **`test_layer_sourcing.sh` Check 4 — root-ownership detection** — add a
   warning-level check that scans `layers/main/*_ws/{build,install,log}` for
   directories owned by root. Warning-only (not a hard fail): the core fix prevents
   new occurrences, but the check surfaces the state if a root-owned dir survives from
   before the fix or enters via another path. Run only when `LAYERS_BASE` is visible
   (same skip condition as Checks 2–3). Add a one-line entry to the
   `## Consequences` section of ADR-0016 noting that `make validate` now detects the
   stale-root-owned state.

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/docker_run_agent.sh` | Add `mkdir -p "$ws_dir/$subdir"` before anonymous-volume `-v` line in section-4 loop |
| `docs/decisions/0016-runtime-vs-baked-layer-chaining.md` | Add Consequences note: concurrent-container hazard + `make validate` detection |
| `.agent/scripts/test_layer_sourcing.sh` | Add Check 4: warning if any `build/install/log` dir under `layers/main` is root-owned |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Enforcement over documentation | Check 4 in `test_layer_sourcing.sh` catches root-owned dirs in `make validate`, not just in prose |
| Capture decisions, not just implementations | ADR-0016 Consequences note documents the concurrent-dispatch hazard with reference to #559 so the why isn't buried in issue history |
| A change includes its consequences | All three files updated together; no separate follow-up needed for the ADR or detection |
| Only what's needed | Three targeted changes; Check 4 is warning-only and reuses the existing layer-scan infrastructure |
| Test what breaks | Check 4 covers the failure mode (root-owned mountpoints) that the fix prevents |
| Improve incrementally | Single focused PR; no unrelated cleanup |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0016 — Runtime vs. baked layer chaining | Yes | Consequences section update documents concurrent-container collision; Check 4 enforces the invariant in validate |
| ADR-0002 — Worktree isolation | Tangential | Fix supports the container-dispatch pathway used alongside worktrees; no structural change needed |
| ADR-0004 — Enforcement hierarchy | Yes (Check 4 path) | Adding a detection check to `test_layer_sourcing.sh` (run by `make validate`) satisfies the "enforce mechanically" principle |

## Consequences

| If we change… | Also update… | Included in plan? |
|---|---|---|
| `docker_run_agent.sh` section-4 loop | No API/interface change; AGENTS.md script-table entry is already accurate at the level of detail it describes | Yes — no cascade needed |
| ADR-0016 Consequences | Review guide ADR table unchanged (title unchanged); no cascade | Yes — self-contained |
| `test_layer_sourcing.sh` Check 4 | ADR-0016 Consequences gets a one-liner noting `make validate` detects root-owned dirs (bundled with step 2) | Yes |

## Open Questions

- [ ] No open questions — plan is review-plan-ready.

## Estimated Scope

Single PR. All three changes are in workspace infrastructure files, no ROS packages affected.
