# Plan: Fix O(N²) layer sourcing and inverted overlay precedence in setup.bash

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/559

## Context

`.agent/scripts/setup.bash` sources `install/setup.bash` (the baked chained
version) for each layer. Each chained file re-sources its entire parent chain via
Python helpers, producing O(N²) work: 7 layers × 7 chain expansions ≈ 4.5s per
invocation, vs. 0.5s for `local_setup.bash` (current layer only).

Two correctness bugs compound this:
1. **Inverted precedence**: sourcing chained files in ascending order gives
   `underlay` higher `AMENT_PREFIX_PATH` priority than overlays — the reverse of
   colcon's intended semantics.
2. **`build.sh` pollutes baked chains**: line 78 does
   `source "$SCRIPT_DIR/setup.bash"` before building any layer, so when
   `underlay_ws` is (re)built, colcon records all other layers as its parents.

`local_setup.bash` sets up only the current layer's prefixes; runtime chaining
(source jazzy → source each `local_setup.bash` in order) reproduces the full
environment correctly with O(N) work and canonical precedence.

## Approach

1. **Fix `setup.bash` inner loop** — change `install/setup.bash` →
   `install/local_setup.bash` (line 86). Source jazzy is already at the top; the
   loop order (ascending layers.txt) naturally gives the last-sourced layer
   highest precedence, which is correct colcon semantics.

2. **Fix `build.sh` layer-list load** — replace the blind
   `source "$SCRIPT_DIR/setup.bash"` (line 78) with a targeted parse of
   `layers.txt` to populate the `LAYERS` array. After building each layer,
   continue sourcing `install/local_setup.bash` (not `install/setup.bash`) for
   progressive chaining (line 126–127 pattern stays, file changes).

3. **Fix generated worktree `setup.bash`** in `worktree_create.sh`
   `generate_worktree_scripts()` — lines 129–136: change
   `install/setup.bash` → `install/local_setup.bash` in the generated per-layer
   conditionals.

4. **Fix generated per-layer `build.sh` preamble** in `worktree_create.sh` —
   lines 279–281: the preamble for each layer sources prior layers via
   `install/setup.bash`; change to `install/local_setup.bash`.

5. **Fix `test.sh`** — lines 97–98 source `install/setup.bash` to stack
   layers between test runs; change to `install/local_setup.bash`.

6. **Fix `verify_change.sh`** — lines 63–64: same pattern; change to
   `install/local_setup.bash`.

7. **Write an ADR** (`docs/decisions/0016-runtime-vs-baked-layer-chaining.md`)
   recording the decision to use runtime chaining (`local_setup.bash`) rather than
   baked chaining (`setup.bash`). Issue review flagged this as an ADR-0001 trigger.

8. **Update `WORKTREE_GUIDE.md`** — brief note in the generated-scripts table that
   the generated `setup.bash` sources `local_setup.bash` per layer (not the baked
   chain).

9. **One-time clean rebuild instructions in PR description** — document the manual
   bottom-up clean rebuild needed to heal polluted baked chains in existing
   installations. This is an operator step, not an automated script change.

10. **Sweep result** — the grep in step 5–6 finds all remaining
    `install/setup.bash` references in `.agent/`. `dashboard.sh:197` uses it only
    for existence check (`-f`), which is unaffected. No other callers need
    updating.

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/setup.bash` | Line 86: `install/setup.bash` → `install/local_setup.bash` |
| `.agent/scripts/build.sh` | Line 78: replace `source setup.bash` with direct layers.txt parse; lines 126–127: `install/setup.bash` → `install/local_setup.bash` |
| `.agent/scripts/worktree_create.sh` | Lines 130,132: generated setup.bash uses `local_setup.bash`; lines 279–281: build preamble uses `local_setup.bash` |
| `.agent/scripts/test.sh` | Lines 97–98: `install/setup.bash` → `install/local_setup.bash` |
| `.agent/scripts/verify_change.sh` | Lines 63–64: `install/setup.bash` → `install/local_setup.bash` |
| `docs/decisions/0016-runtime-vs-baked-layer-chaining.md` | New ADR documenting runtime chaining decision |
| `.agent/WORKTREE_GUIDE.md` | Note in generated-scripts table about `local_setup.bash` |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Enforcement over documentation | Behavioral fix; ADR prevents reversion by documenting the why |
| A change includes its consequences | Sweep of all `install/setup.bash` callers included; WORKTREE_GUIDE updated |
| Only what's needed | No changes outside the five scripts, ADR, and guide update |
| Test what breaks | Manual verification steps below; dashboard.sh existence check unaffected |
| Improve incrementally | Single PR; existing worktrees age out naturally |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0001 — Adopt ADRs | Yes | New ADR-0016 records runtime-vs-baked decision |
| ADR-0007 — Retain Make with Dependency Tracking | Watch | `build.sh` change must not break Make stamp-file semantics; `make build` calls `build.sh`, which still builds layers in order and sources each on success |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `setup.bash` inner loop | `worktree_create.sh` generated `setup.bash` template | Yes — step 3 |
| `build.sh` sourcing | progressive `install/setup.bash` refs in build loop | Yes — step 2 |
| Generated worktree scripts | `WORKTREE_GUIDE.md` generated-scripts description | Yes — step 8 |
| All `install/setup.bash` callers | `test.sh`, `verify_change.sh` | Yes — steps 5–6 |
| Design decision in scripts | ADR to prevent future reversion | Yes — step 7 |

## Open Questions

- [ ] No open questions — plan is review-plan-ready.

## Estimated Scope

Single PR. All changes in workspace infrastructure; no ROS package modifications.
