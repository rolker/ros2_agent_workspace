# ADR-0016: Runtime Layer Chaining via local_setup.bash (not Baked Chains)

## Status

Accepted.

## Context

Colcon generates two entry points per workspace install:

- `install/setup.bash` — the **baked chain**: re-sources every parent prefix
  recorded on `COLCON_PREFIX_PATH` *at build time*, then the workspace itself.
- `install/local_setup.bash` — the workspace itself only.

The workspace's environment scripts (`setup.bash`, `build.sh`, `test.sh`,
`verify_change.sh`, and the worktree-generated scripts) sourced each layer's
baked `install/setup.bash` in ascending `layers.txt` order. Investigation for
[#559](https://github.com/rolker/ros2_agent_workspace/issues/559) found three
compounding problems:

1. **O(N²) cost.** Each baked chain re-sources its full parent list (each link
   spawning a Python environment helper), so sourcing N layers does
   N + (N−1) + … chain expansions. Measured on 7 built layers: **~4.5s vs
   ~0.5s** — paid on nearly every agent build/test command.
2. **Inverted overlay precedence.** The combination of baked chains, colcon's
   prepend-dedup, and ascending sourcing produced an `AMENT_PREFIX_PATH` with
   `underlay` *outranking* every overlay — the reverse of colcon's overlay
   semantics.
3. **Chain pollution.** `build.sh` sourced the *full* workspace environment
   before building the first layer, so every rebuilt layer baked **all other
   layers — including higher ones — into its parent chain** (e.g.
   `underlay_ws` recording `ui_ws` as a parent). Every `make build` re-wrote
   the corruption; every layer in the tree was found polluted.

The baked chain is a snapshot of the build-time environment. In a layered
workspace whose composition is defined by a config file (`layers.txt`) and
whose layers rebuild independently, that snapshot is inherently prone to going
stale — and, per (3), was actively wrong.

## Decision

**Chain layers at source time (runtime chaining), never via baked chains:**

1. Source `/opt/ros/jazzy/setup.bash` once, then each built layer's
   `install/local_setup.bash` in ascending `layers.txt` order. The last-sourced
   (topmost) layer gets highest precedence — canonical colcon semantics,
   decided by `layers.txt` at source time and immune to baked-chain staleness.
2. Build steps see only the ROS base plus layers **strictly below** the layer
   being built (sourced progressively, post-build). The full workspace
   environment is never sourced before building — that is what polluted the
   chains.
3. Worktree-generated scripts (`setup.bash`, per-layer `build.sh`/`test.sh`
   preambles) follow the same rule.

The regression guard `.agent/scripts/test_layer_sourcing.sh` (run by
`make validate`) enforces this: a static check that no workspace script
sources a baked `install/setup.bash`, a runtime check that the sourced
precedence order is canonical, and a warning-level purity check on the baked
chains themselves.

## Consequences

- Environment setup drops from ~4.5s to ~0.5s per shell; overlay precedence is
  canonical and deterministic.
- Existing installs keep polluted baked chains until a **one-time bottom-up
  clean rebuild** (`rm -rf layers/main/*_ws/{build,install,log} && make build`).
  Runtime chaining doesn't read them, but anything sourcing a layer's
  `install/setup.bash` directly inherits them — the guard warns until healed.
- Sourcing a *single* layer's `install/setup.bash` by hand no longer matches
  how the workspace composes its environment; use `.agent/scripts/setup.bash`
  (or the worktree's generated `setup.bash`), which is now cheap.
- Existing worktrees keep their old generated scripts (slow but functional)
  and age out as worktrees are removed.

## References

- [#559](https://github.com/rolker/ros2_agent_workspace/issues/559) —
  measurement, root-cause analysis, and implementation.
- Colcon documentation: `setup.bash` (prefix chain) vs `local_setup.bash`
  (single prefix).
