# Plan: Rosdep bake follow-up (#520 gaps + resilience)

## Issue
https://github.com/rolker/ros2_agent_workspace/issues/522

## Context
The #520 bake had two gaps found during a real `make agent-build`:
(1) the `make agent-build` staging step was edited in the main tree by mistake
and never committed, so `main`'s target is the bare `docker build` that fails on
`COPY .rosdep-manifests/`; (2) staging gathered `COLCON_IGNORE`'d packages, and
since rosdep ignores that marker, one dead ROS1 package (`image_warper`,
dep `project11`) aborted the all-or-nothing `rosdep install` — baking nothing.

## Approach
1. **Fix A — `make agent-build`**: stage via `stage_rosdep_manifests.sh` (trap-cleaned) before `docker build`. The commit that was lost in #520.
2. **Fix B — skip ignored packages**: `stage_rosdep_manifests.sh` skips any package under a `COLCON_IGNORE`/`AMENT_IGNORE`/`CATKIN_IGNORE` marker.
3. **Fix C — resilient bake**: install all layers in one rosdep pass (so `--ignore-src` ignores cross-layer local packages — a per-layer pass would wrongly treat e.g. `ui_ws`'s use of `sensors_ws`'s `marine_radar_control_msgs` as an unresolvable key), but pre-compute genuinely-unresolvable external keys via `rosdep check` and pass them as `--skip-keys` so one bad key only skips itself.

## Files to Change
| File | Change |
|------|--------|
| `Makefile` | `agent-build` stages manifests (trap-cleaned) before the bare `docker build` |
| `.agent/scripts/stage_rosdep_manifests.sh` | Skip `COLCON_IGNORE`/`AMENT_IGNORE`/`CATKIN_IGNORE` packages |
| `.devcontainer/agent/Dockerfile` | All-at-once bake with dynamic `--skip-keys` |
| `.devcontainer/agent/README.md` | Document ignore-skip + skip-keys resilience |

## Principles Self-Check
| Principle | Consideration |
|---|---|
| A change includes its consequences | Found via real build; fixes every build path (Makefile + script) and the docs |
| Test what breaks | Staging verified to skip ignored pkgs (108→107); real `make agent-build` confirms apt installs happen |
| Only what's needed | Dynamic skip-keys is the minimal robust mechanism; avoids per-layer's cross-layer breakage |

## ADR Compliance
| ADR | Triggered | How addressed |
|---|---|---|
| 0009 — Python pkg mgmt | Yes | rosdep/apt only |
| 0007 — Retain Make | Yes | `agent-build` recipe stays a Make target; no new runner |

## Consequences
| If we change... | Also update... | In plan? |
|---|---|---|
| Bake/build paths | README + every build entry point (Makefile + docker_run_agent.sh) | Yes |

## Open Questions
- [ ] None — bake strategy settled with user (all-at-once + dynamic skip-keys).

## Estimated Scope
Single PR. Depends conceptually on rqt_operator_tools#62 (fixes one real bad key), but skip-keys makes the bake resilient regardless.
