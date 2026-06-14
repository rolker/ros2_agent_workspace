# Plan: Bake layer rosdep dependencies into the agent Docker image at build time

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/520

## Context

The agent image (`.devcontainer/agent/Dockerfile`, base `ros:jazzy-perception`)
never runs `rosdep install` for the workspace's layer packages, so their system
deps aren't baked in. Because each container FS is ephemeral (`docker run --rm`),
the launch-time `rosdep check`-then-skip added in #489 (`agent-entrypoint.sh`
step 4) finds almost everything missing and re-installs it via apt on every
launch. Baking the deps into image layers makes the skip actually fire; only the
delta installs at launch.

The wrinkle: the build context is `.devcontainer/agent/`, which has **no layer
source** (`layers/` is gitignored and mounted at runtime, never copied). So the
Dockerfile has no `package.xml` to `rosdep install` against. Solution is the ROS
"copy only the manifests" idiom — stage the **108** `package.xml` files
(recursive count across the checked-out layers) into the build context host-side
(where `layers/` exists), `COPY` just those, `rosdep install`, then discard.
Source stays mounted, not baked.

## Approach

1. **Stage manifests host-side in `docker_run_agent.sh`** — in the build block
   (around line 260, before `docker build`), gather every `package.xml` under
   the layer sources **recursively** (`find "$ROOT_DIR"/layers/main/*_ws/src
   -name package.xml`) into `.devcontainer/agent/.rosdep-manifests/`, preserving
   each file's path relative to `$ROOT_DIR` (so each manifest lands in its **own
   directory** — `rosdep install --from-paths` reads one `package.xml` per dir).
   A non-recursive `src/*/package.xml` glob would catch only 21 of 108 manifests
   — most project repos are multi-package and nest manifests in subdirs
   (`marine_control/marine_control_interfaces/package.xml`, etc.), so a shallow
   glob would leave ~80% of layer deps un-baked and the launch-time apt storm
   would persist. Wipe and regenerate the dir on every `--build` so a stale
   manifest set can never bake outdated deps.
2. **Add `.devcontainer/agent/.dockerignore`** — keep the build context lean and
   explicit: include the staging dir + the two tracked files (`Dockerfile`,
   `agent-entrypoint.sh`), exclude `README.md` and anything else.
3. **Gitignore the staging dir** — `.devcontainer/agent/.rosdep-manifests/` is a
   generated build artifact; add it to `.gitignore` (workspace-cleanliness rule).
4. **Bake deps in the Dockerfile** — after the dev-tools apt block and the
   existing `rosdep init`, while still **root** (apt needs root, and the image
   has no sudo): `apt-get update` (the dev-tools blocks each end with `rm -rf
   /var/lib/apt/lists/*`, so the apt index is empty at the bake point and the
   install would fail without a fresh index) → `rosdep update` (root cache) →
   `COPY .rosdep-manifests/ ...` → `rosdep install --from-paths <staged src dirs>
   --ignore-src -y --rosdistro jazzy` → `rm -rf /var/lib/apt/lists/*` + remove
   the staged copy (keep the image layer lean). Keep the existing user-side
   `rosdep update` (line 80) so the runtime check still works under the `ros`
   user. Use best-effort (`|| true`) on the install so one transiently-unavailable
   dep doesn't break the whole image build, matching the entrypoint's posture —
   but echo what failed.
5. **Docs** — update `.devcontainer/agent/README.md` (image now bakes layer deps;
   when to rebuild: dep drift / `package.xml` changes) and re-check the
   `docker_run_agent.sh` row in AGENTS.md's Script Reference.

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/docker_run_agent.sh` | Add **recursive** manifest-staging step before `docker build`; regenerate fresh each build |
| `.devcontainer/agent/Dockerfile` | `COPY` staged manifests + root-side `apt-get update`/`rosdep update`/`install`/cleanup |
| `.devcontainer/agent/.dockerignore` | New — constrain build context to intended files |
| `.gitignore` | Ignore `.devcontainer/agent/.rosdep-manifests/` |
| `.devcontainer/agent/README.md` | Document dep-baking + rebuild triggers |
| `AGENTS.md` | Revisit `docker_run_agent.sh` Script Reference row if `--build` semantics change |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Only what's needed | Solves a measured pain (apt storm in dispatch logs); manifest-staging is the minimum mechanism to do it |
| A change includes its consequences | README + `.dockerignore` + `.gitignore` + AGENTS.md row updated in the same PR, not deferred |
| Human control and transparency | Build logs show the bake; launch logs show "satisfied — skipping"; `FORCE_DEPS_REFRESH=1` escape hatch unchanged |
| Test what breaks | Manifest-gather logic is the testable, silently-breakable part: assert the recursive gather copies all 108 manifests (not the 21 a shallow glob catches), each into its own dir, and handles empty/missing `src/`. A regression here silently re-creates the apt storm |
| Improve incrementally | Small, layered on #489's already-merged entrypoint check |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0009 — Python pkg mgmt | Yes | Uses `rosdep`/apt for system deps — exactly as prescribed; no bare pip |
| 0008 — ROS conventions | Yes | Standard ROS "copy manifests only → rosdep install → mount source" Docker idiom |
| 0003 — Project-agnostic workspace | Watch | Baked deps are a snapshot of the checked-out project layers; acceptable since the image is a local, uncommitted, per-workspace artifact — stated explicitly in README/PR |
| 0007 — Retain Make | No | Build routes through `docker_run_agent.sh --build`, not a Make target |
| 0001 — Adopt ADRs | No | Slower-build/faster-launch trade-off already captured in the issue; an implementation choice, not a new architecture decision |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `docker_run_agent.sh` (a `.agent/scripts/` script) | AGENTS.md Script Reference row | Yes |
| The image build / Dockerfile | `.devcontainer/agent/README.md` | Yes |
| Add a generated dir under a tracked path | `.gitignore` + `.dockerignore` | Yes |

## Open Questions

_Both resolved with the user (2026-06-14) — recorded here, no longer open:_

- **rosdep cache ownership at build time** → **Add a root-side `rosdep update`**
  before the bake's `rosdep install`, keeping the existing user-side `rosdep
  update` (line 80) so the runtime check stays warm for the `ros` user. Accepts a
  second cache + small extra build time.
- **Build-failure posture** → **Best-effort + log** (`rosdep install ... || true`,
  echo which layer/deps failed). The image build always succeeds; a missing dep
  falls through to the existing launch-time install path. Matches the #489
  entrypoint posture.

## Estimated Scope

Single PR.
