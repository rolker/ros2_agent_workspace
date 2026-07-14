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
   `source "$SCRIPT_DIR/setup.bash"` (line 78) with sourcing
   `/opt/ros/jazzy/setup.bash` (the base only — the first `colcon build` needs a
   ROS environment; sourcing layer overlays pre-build is the pollution bug being
   fixed) plus a targeted parse of `layers.txt` to populate the `LAYERS` array
   (same worktree-aware fallback as `setup.bash`). After building each layer,
   continue sourcing `install/local_setup.bash` (not `install/setup.bash`) for
   progressive chaining (line 126–127 pattern stays, file changes).

3. **Fix generated worktree `setup.bash`** in `worktree_create.sh`
   `generate_worktree_scripts()` — lines 129–136: change
   `install/setup.bash` → `install/local_setup.bash` in the generated per-layer
   conditionals.

4. **Fix generated per-layer `build.sh` preamble** in `worktree_create.sh` —
   lines 279–281: the preamble for each layer sources prior layers via
   `install/setup.bash`; change to `install/local_setup.bash`.

5. **Fix `test.sh`** — lines 97–98 source `install/setup.bash` per layer inside
   the test loop; safe to change to `install/local_setup.bash` because line 61
   already sources the full workspace environment via
   `.agent/scripts/setup.bash` before the loop.

6. **Fix `verify_change.sh`** — lines 63–64. *Deviation from the reviewed plan,
   found during implementation*: unlike `test.sh`, this script never sources the
   workspace environment — it depends **entirely** on the layer's baked chained
   `install/setup.bash` to bring in jazzy and lower layers, so a naive swap to
   `local_setup.bash` would strip the ROS base out of its test environment.
   Correct fix: source `.agent/scripts/setup.bash` (fast after step 1) instead of
   the layer's baked chain.

7. **Write an ADR** (`docs/decisions/0016-runtime-vs-baked-layer-chaining.md`)
   recording the decision to use runtime chaining (`local_setup.bash`) rather than
   baked chaining (`setup.bash`). Issue review flagged this as an ADR-0001 trigger.

8. **Update `WORKTREE_GUIDE.md`** — brief note in the generated-scripts table that
   the generated `setup.bash` sources `local_setup.bash` per layer (not the baked
   chain).

9. **One-time clean rebuild instructions in PR description** — document the manual
   bottom-up clean rebuild needed to heal polluted baked chains in existing
   installations. This is an operator step, not an automated script change.

10. **Sweep result** — full grep of `install/setup.bash` across `.agent/`,
    triaged:
    - `dashboard.sh:197` — existence check (`-f`) only; unaffected, no change.
    - `.agent/templates/ci_workflow.yml:76` — **excluded**: sources a single
      built `ws` in a flat CI workspace, no layered chaining, so neither the
      O(N²) cost nor the precedence bug applies.
    - `.agent/knowledge/ros2_development_patterns.md:154–155` — **updated**
      (step 11): it documents the exact chained-ascending anti-pattern this PR
      removes.
    - Legacy work plans (`PLAN_ISSUE-324.md`, `PLAN_ISSUE-356.md`) — historical
      records, not touched.

11. **Update `ros2_development_patterns.md`** — replace the Environment
    Sourcing Order example (chained `install/setup.bash` per layer) with the
    runtime-chaining pattern (jazzy base + per-layer `local_setup.bash`).

12. **Regression guard** (review-issue Action #4, operator-approved) — new
    `.agent/scripts/test_layer_sourcing.sh` wired into `make validate`:
    (a) asserts the sourced `AMENT_PREFIX_PATH` layer order matches `layers.txt`
    reversed (top layer first, underlay just above jazzy) — catches precedence
    inversion; (b) asserts each built layer's baked chain
    (`COLCON_CURRENT_PREFIX` lines in `install/setup.sh`) references only jazzy
    and strictly-lower layers — catches a `build.sh` pollution regression and
    flags unhealed installs needing the one-time clean rebuild. Skips cleanly
    (exit 0 with notice) when no layers are built, so CI is unaffected.

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/setup.bash` | Line 86: `install/setup.bash` → `install/local_setup.bash` |
| `.agent/scripts/build.sh` | Line 78: replace `source setup.bash` with direct layers.txt parse; lines 126–127: `install/setup.bash` → `install/local_setup.bash` |
| `.agent/scripts/worktree_create.sh` | Lines 130,132: generated setup.bash uses `local_setup.bash`; lines 279–281: build preamble uses `local_setup.bash` |
| `.agent/scripts/test.sh` | Lines 97–98: `install/setup.bash` → `install/local_setup.bash` |
| `.agent/scripts/verify_change.sh` | Lines 63–64: source workspace `setup.bash` instead of the layer's baked chain (see step 6 deviation) |
| `.agent/scripts/test_layer_sourcing.sh` | New regression guard: precedence order + baked-chain purity (step 12) |
| `Makefile` | `validate` target also runs `test_layer_sourcing.sh` |
| `.agent/knowledge/ros2_development_patterns.md` | Environment Sourcing Order example → runtime chaining (step 11) |
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
