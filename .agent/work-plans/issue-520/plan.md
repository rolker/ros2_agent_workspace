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
"copy only the manifests" idiom — stage the 102 `package.xml` files into the
build context host-side (where `layers/` exists), `COPY` just those, `rosdep
install`, then discard. Source stays mounted, not baked.

## Approach

1. **Stage manifests host-side in `docker_run_agent.sh`** — in the build block
   (around line 260, before `docker build`), gather every
   `layers/main/*_ws/src/*/package.xml` into
   `.devcontainer/agent/.rosdep-manifests/`, preserving the
   `layers/main/<ws>/src/<pkg>/` structure. Wipe and regenerate the dir on every
   `--build` so a stale manifest set can never bake outdated deps.
2. **Add `.devcontainer/agent/.dockerignore`** — keep the build context lean and
   explicit: include the staging dir + the two tracked files (`Dockerfile`,
   `agent-entrypoint.sh`), exclude `README.md` and anything else.
3. **Gitignore the staging dir** — `.devcontainer/agent/.rosdep-manifests/` is a
   generated build artifact; add it to `.gitignore` (workspace-cleanliness rule).
4. **Bake deps in the Dockerfile** — after the dev-tools apt block and the
   existing `rosdep init`, while still **root** (apt needs root, and the image
   has no sudo): `rosdep update` (root cache) → `COPY .rosdep-manifests/ ...` →
   `rosdep install --from-paths <staged src dirs> --ignore-src -y --rosdistro
   jazzy` → remove the staged copy. Keep the existing user-side `rosdep update`
   (line 80) so the runtime check still works under the `ros` user. Use
   best-effort (`|| true`) on the install so one transiently-unavailable dep
   doesn't break the whole image build, matching the entrypoint's posture — but
   echo what failed.
5. **Docs** — update `.devcontainer/agent/README.md` (image now bakes layer deps;
   when to rebuild: dep drift / `package.xml` changes) and re-check the
   `docker_run_agent.sh` row in AGENTS.md's Script Reference.

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/docker_run_agent.sh` | Add manifest-staging step before `docker build`; regenerate fresh each build |
| `.devcontainer/agent/Dockerfile` | `COPY` staged manifests + root-side `rosdep update`/`install`/cleanup |
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
| Test what breaks | Manifest-gather logic (structure-preserving copy, empty/missing `src/`) is the testable, silently-breakable part |
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

- **rosdep cache ownership at build time**: the bake must run as root (apt, no
  sudo in image), so it needs a **root-side** `rosdep update` before `rosdep
  install`. The current Dockerfile only runs `rosdep update` as the `ros` user
  (line 80). Plan adds a root-side `rosdep update` before the install. Confirm
  this is acceptable (small extra build time + a second rosdep cache) vs. an
  alternative ordering.
- **Build failure posture**: best-effort (`|| true`) so a transiently-missing
  dep doesn't break the image, matching the entrypoint — or hard-fail to catch
  real dep errors at build time? Plan proposes best-effort-with-logging; flag for
  the reviewer.

## Estimated Scope

Single PR.
