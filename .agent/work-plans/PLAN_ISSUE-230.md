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
Additionally, the two packages compose cleanly — an agent can wrap a launch with
both GUI and programmatic monitoring, giving the user interactive control while the
agent monitors output and lifecycle events on a background thread.

Per the owner's comment on the issue, the knowledge doc should be workspace-level
(generic ROS 2 tooling) and cover both packages. The review comment recommends
naming it `launch_tooling.md` rather than `integration_testing.md`.

## Approach

### Step 1: Create `.agent/knowledge/launch_tooling.md`

New knowledge doc with these sections:

1. **Overview** — what the two packages provide and when to use them
2. **`ros2 launch -g` (GUI monitoring)** — the simplest entry point
   - When launching interactively, prefer `ros2 launch -g <pkg> <launch_file>`
   - Note that `-g` is a `ros2launch_gui` extension, not upstream ROS 2
   - Qt window shows process tree, output, lifecycle states
3. **`ros2launch_session` API** — programmatic launch management
   - `LaunchSession(ld)` + `session.run(on_ready=callback)` — standalone mode
   - `wait_for_output(text, stream, timeout)` — replaces `time.sleep()`
   - `wait_for_startup(process, timeout)` — block until process starts
   - `shutdown()` — clean SIGINT → SIGTERM → SIGKILL escalation
   - `LaunchSession.from_service(ls, ld)` — context manager for borrowing
     an existing LaunchService
   - When to use `LaunchSession` vs `colcon test`
4. **Composing both: GUI + programmatic monitoring** — the key pattern
   - Wrap LD with `DisplayUserInterface` first, then with `LaunchSession`
   - Threading model: main thread runs LaunchService + Qt polling,
     `on_ready` callback runs on daemon thread
   - Both sets of event handlers fire for the same process events
   - Shutdown coordination: user closes GUI vs agent calls `shutdown()`
   - Complete working example
5. **Availability** — both packages come from `rolker/ros2launch_gui` and
   `rolker/ros2launch_session` (branch `jazzy`). Include a check snippet:
   `python3 -c "import ros2launch_session"` and `ros2 launch --help | grep -q '\-g'`

### Step 2: Cross-link in `ros2_development_patterns.md`

Add a "see also" note in the Launch Files section (around line 83) pointing to
`launch_tooling.md` for programmatic launch management and GUI monitoring.

### Step 3: Cross-link in `ros2_cli_best_practices.md`

Add a short section or note about `ros2 launch -g` for visual monitoring, with a
pointer to `launch_tooling.md` for the full API.

## Files to Change

| File | Change |
|------|--------|
| `.agent/knowledge/launch_tooling.md` | New file: ros2launch tooling knowledge doc (5 sections as above) |
| `.agent/knowledge/ros2_development_patterns.md` | Add "see also" link in Launch Files section (~line 83) |
| `.agent/knowledge/ros2_cli_best_practices.md` | Add note about `ros2 launch -g` for visual monitoring |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Workspace vs. project separation | Content is generic ROS 2 tooling guidance — compliant with ADR-0003. Availability check pushes project-specific config to project repos. |
| Only what's needed | Solves concrete pain: zombie processes, fragile readiness checks, undiscoverable tooling. The composition pattern addresses a real use case (agent + user monitoring the same launch). |
| Workspace improvements cascade to projects | Workspace knowledge makes tooling discoverable for all projects. |
| A change includes its consequences | Cross-links in existing knowledge docs are included in plan. |
| Documentation accuracy | API documented from actual source code (`launch_session.py`, `gui.py`, `display_user_interface.py`, `user_interface.py`), not assumptions. Threading model verified by reading the event handler implementations. |

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
| Document composition pattern | Note `noninteractive` flag open item | Yes (in doc) |

## Open Questions

- **`noninteractive` flag**: `LaunchSession` defaults to `noninteractive=True`.
  The GUI works through the launch event system (not terminal I/O) so this should
  be fine, but the doc should note this as something to verify if issues arise.
  Not a blocker for the knowledge doc — can be updated after testing.

## Estimated Scope

Single PR, 3 files (1 new, 2 minor edits).
