# Plan: Add agent knowledge for ros2launch tooling

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/230

## Context

Agents frequently need to launch ROS 2 nodes for integration testing or interactive
use. Today this is done ad-hoc with `ros2 launch` subprocesses and `time.sleep()` for
readiness, leading to zombie processes, fragile timing, and missed errors.

Two packages in `underlay_ws` solve these problems:
- **`ros2launch_session`** — programmatic `LaunchSession` API for integration tests
  (startup detection, output monitoring, guaranteed cleanup)
- **`ros2launch_gui`** — adds a `-g` flag to `ros2 launch` for visual process
  monitoring via `DisplayUserInterface`

Neither is documented in workspace knowledge, so agents don't know they exist.

Per the owner's comment, the knowledge doc should be workspace-level (generic ROS 2
tooling) and cover both packages. The review comment recommends naming it
`launch_tooling.md` rather than `integration_testing.md`.

## Approach

1. **Create `.agent/knowledge/launch_tooling.md`** — new knowledge doc covering:
   - `ros2launch_session` API: `LaunchSession`, `run()`, `wait_for_output()`,
     `wait_for_startup()`, `shutdown()`, `from_service()` context manager
   - `ros2launch_gui` and the `-g` flag for visual monitoring
   - Availability check pattern (both packages come from `rolker/ros2launch_gui`
     and `rolker/ros2launch_session`, branch `jazzy`; may not be in every project's
     underlay)
   - Note that `-g` is a `ros2launch_gui` extension, not upstream ROS 2
   - When to use `LaunchSession` vs `colcon test`
   - Minimal examples for both standalone and `from_service()` modes

2. **Add cross-link in `ros2_development_patterns.md`** — line 80 area (Launch Files
   section) should reference the new doc with a "see also" note

3. **Add cross-link in `ros2_cli_best_practices.md`** — add a section or note about
   `ros2 launch -g` in the context of the existing CLI cookbook

## Files to Change

| File | Change |
|------|--------|
| `.agent/knowledge/launch_tooling.md` | New file: ros2launch tooling knowledge doc |
| `.agent/knowledge/ros2_development_patterns.md` | Add "see also" link to `launch_tooling.md` in Launch Files section (around line 83) |
| `.agent/knowledge/ros2_cli_best_practices.md` | Add note about `ros2 launch -g` for visual monitoring |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Workspace vs. project separation | Content is generic ROS 2 tooling guidance — compliant with ADR-0003. Availability check pushes project-specific config to project repos. |
| Only what's needed | Solves concrete pain: zombie processes, fragile readiness checks, undiscoverable tooling. |
| Workspace improvements cascade to projects | Workspace knowledge makes tooling discoverable for all projects. |
| A change includes its consequences | Cross-links in existing knowledge docs are included in plan. |
| Documentation accuracy | API documented from actual source code (`launch_session.py`, `gui.py`), not assumptions. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0003 — Project-agnostic workspace | Yes | Content is generic ROS 2 tooling; availability check pattern keeps project config in project repos |
| 0008 — Follow ROS 2 conventions | Watch | Doc will note that `-g` is a `ros2launch_gui` extension, not standard ROS 2, so agents understand it may not be available everywhere |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Add `launch_tooling.md` | Cross-link from `ros2_development_patterns.md` | Yes |
| Add `launch_tooling.md` | Cross-link from `ros2_cli_best_practices.md` | Yes |
| Document `-g` flag | Note it's not upstream ROS 2 | Yes |

## Open Questions

None — the owner's comment and review comment provide clear direction on scope,
naming, and placement.

## Estimated Scope

Single PR, 3 files (1 new, 2 minor edits).
