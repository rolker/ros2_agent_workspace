# ROS 2 Launch Tooling for Agents

Two workspace-provided packages solve common agent pain points with ROS 2
launch: `ros2launch_session` provides programmatic launch control with output
monitoring (no more fragile sleeps or zombie processes), and `ros2launch_gui`
adds a Qt-based visual process monitor via `ros2 launch -g`.

## GUI Monitoring with `ros2 launch -g`

Use `ros2 launch -g` to launch nodes with a visual process monitor:

```bash
ros2 launch -g <package_name> <launch_file>
```

This opens a Qt window showing the process tree, stdout/stderr output, and
lifecycle states. Useful when a human operator wants to watch what an
agent-triggered launch is doing.

> **Note**: The `-g` flag is provided by `ros2launch_gui`, not upstream ROS 2.
> It will not be available unless `ros2launch_gui` is installed.

## `ros2launch_session` API

`LaunchSession` wraps `LaunchService` with a clean lifecycle and output
monitoring API. It runs the launch in the main thread and calls your
`on_ready` callback in a daemon thread once the launch service is running.

### Standalone Mode

```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ros2launch_session import LaunchSession

ld = LaunchDescription([
    ExecuteProcess(cmd=['ros2', 'run', 'demo_nodes_cpp', 'talker'])
])

session = LaunchSession(ld)

def on_ready(s):
    s.wait_for_output("Hello World", stream="stderr", timeout=30)
    s.shutdown()

exit_code = session.run(on_ready=on_ready)
```

### Key Methods

| Method | Description |
|--------|-------------|
| `wait_for_output(expected, *, process=None, timeout=10, stream='stderr')` | Block until output text appears. Filter by process and stream. |
| `wait_for_startup(process, *, timeout=10)` | Block until a process has started. |
| `wait_for_shutdown(process, *, timeout=10)` | Block until a process has exited. |
| `shutdown()` | Trigger graceful shutdown of the launch service. |
| `get_proxy(process_action)` | Get a `ProcessProxy` for a specific `ExecuteProcess` action. |

### Service Mode

For integration with external orchestration, `LaunchSession` can also be
created from a ROS 2 service:

```python
with LaunchSession.from_service() as session:
    session.wait_for_output("ready", timeout=30)
```

### When to Use `LaunchSession` vs `colcon test`

- **`LaunchSession`**: Runtime integration tests that need to monitor process
  output, wait for readiness signals, or orchestrate multi-process scenarios.
- **`colcon test`**: Unit tests, linter checks, and standard `launch_testing`
  tests that follow the pytest pattern.

## Composing GUI + Programmatic Monitoring

To give the user a visual monitor while the agent controls the launch
programmatically, wrap the launch description with `DisplayUserInterface`
before passing it to `LaunchSession`:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ros2launch_gui.actions import DisplayUserInterface
from ros2launch_session import LaunchSession

ld = LaunchDescription([
    IncludeLaunchDescription(
        AnyLaunchDescriptionSource('/path/to/launch_file.py')
    )
])
gui_ld = LaunchDescription([
    DisplayUserInterface(launch_description=ld)
])
session = LaunchSession(gui_ld)

def on_ready(s):
    s.wait_for_output("Managed nodes are active", stream="stderr", timeout=60)
    s.shutdown()

exit_code = session.run(on_ready=on_ready)
```

### Threading Model

- **Main thread**: Runs `LaunchService` and Qt event-loop polling.
- **Daemon thread**: Runs the `on_ready` callback.

### Shutdown Behavior

- **User closes GUI**: Triggers a `Shutdown` action in the launch service.
  The agent's `on_ready` callback should handle `LaunchServiceShutdown`
  exceptions or check for early termination.
- **Agent calls `shutdown()`**: The GUI window closes and the launch service
  shuts down gracefully.

## Availability

Both packages come from the `rolker` GitHub organization:

- [`rolker/ros2launch_gui`](https://github.com/rolker/ros2launch_gui) (branch `jazzy`)
- [`rolker/ros2launch_session`](https://github.com/rolker/ros2launch_session) (branch `jazzy`)

### Checking Installation

```bash
# Check ros2launch_session
python3 -c "import ros2launch_session"

# Check ros2launch_gui
ros2 launch --help | grep -q '\-g'
```

If either check fails, add the missing package to the project's
`underlay.repos` file.
