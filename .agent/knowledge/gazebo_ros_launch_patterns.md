# Gazebo + ROS 2 Launch Patterns

Agent guidance for launching Gazebo (gz-sim) with ROS 2 launch files. Covers
lifecycle coupling (so closing one side cleanly stops the other), instance
management (avoiding accidental duplicates), and common pitfalls.

## Two Launch Architectures

| Aspect | ExecuteProcess (`gz_sim.launch.py`) | GzServer ROS Node |
|--------|--------------------------------------|-------------------|
| **Mechanism** | Runs `gz sim` as an OS process via `ExecuteProcess` | Loads gz-sim as a ROS 2 component node |
| **Lifecycle coupling** | Manual — requires `on_exit` event handlers or `on_exit_shutdown` | Automatic — `rclcpp::shutdown()` propagates |
| **Signal handling** | `shell=True` complicates SIGTERM delivery; fixed in gz-sim 8.8.x+ | Standard ROS 2 node signal handling |
| **GUI support** | Separate `-g` process or combined | Separate process only |
| **Composition** | No — separate process | Yes — load bridge in same container |
| **When to use** | Most common; used by ros_gz_sim, VRX, workspace packages | Advanced; when you need tight ROS lifecycle integration |

**This workspace uses ExecuteProcess exclusively** — all simulation launch files
delegate to `ros_gz_sim`'s `gz_sim.launch.py` or call `gz sim` directly.

## Lifecycle Coupling

### Pattern A: IncludeLaunchDescription with on_exit_shutdown

The simplest approach — include `ros_gz_sim`'s launch file and enable its
built-in shutdown coupling:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': '-v4 -r path/to/world.sdf',
            'on_exit_shutdown': 'true',       # <-- key parameter
        }.items(),
    )
    return LaunchDescription([gz_sim_launch])
```

> **`on_exit_shutdown` defaults to `false`** — you must explicitly opt in.
> Without it, killing Gazebo leaves the ROS launch running (and vice versa).

### Pattern B: RegisterEventHandler with OnProcessExit

For custom launch files that run `gz sim` via `ExecuteProcess` directly,
use `OnProcessExit` to emit a `Shutdown` event:

```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

def generate_launch_description():
    gz_server = ExecuteProcess(
        cmd=['gz', 'sim', '-v4', '-s', '-r', 'world.sdf'],
        output='screen',
    )
    gz_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-v4', '-g'],
        output='screen',
    )
    shutdown_on_gz_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=gz_server,
            on_exit=[EmitEvent(event=Shutdown(reason='Simulation ended'))],
        )
    )
    return LaunchDescription([gz_server, gz_gui, shutdown_on_gz_exit])
```

This is the pattern used by VRX (`vrx_gz/launch.py`) — it monitors a sim
process and emits `Shutdown` when it exits.

**Workspace examples**:
- `ben_gazebo/launch/gazebo.launch.py` — Pattern A (includes `gz_sim.launch.py`)
- `drix_gazebo/launch/gazebo.launch.py` — Pattern A
- `portsmouth_nh_gazebo/launch/*.launch.py` — Direct `ExecuteProcess` without
  shutdown coupling (no `on_exit` handler — **these will leave orphans**)
- `vrx_gz/launch.py` — Pattern B via `monitor_sim.py`

### Pattern C: LaunchSession + Gazebo (Agent Debugging)

For agents that need programmatic control of Gazebo sessions (e.g., iterative
parameter tuning), combine `LaunchSession` with `DisplayUserInterface`:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ros2launch_gui.actions import DisplayUserInterface
from ros2launch_session import LaunchSession

ld = LaunchDescription([
    IncludeLaunchDescription(
        AnyLaunchDescriptionSource('/path/to/gazebo_launch.py')
    )
])
gui_ld = LaunchDescription([DisplayUserInterface(launch_description=ld)])
session = LaunchSession(gui_ld)

def on_ready(s):
    # Wait for model spawn, not just Gazebo startup
    s.wait_for_output('Entity creation successful', timeout=60)

    # Monitor pose via gz topic subprocess (more reliable than ROS bridges)
    import subprocess, re
    proc = subprocess.Popen(
        ['gz', 'topic', '-e', '-t', '/world/ocean/dynamic_pose/info'],
        stdout=subprocess.PIPE, text=True,
    )
    for line in proc.stdout:
        match = re.search(r'position.*?x:\s*([\d.-]+).*?y:\s*([\d.-]+).*?z:\s*([\d.-]+)', line)
        if match:
            print(f"pose: x={match.group(1)} y={match.group(2)} z={match.group(3)}")
    s.shutdown()

exit_code = session.run(on_ready=on_ready)
```

**Field-tested findings** (from buoyancy debugging on BEN):

- **`wait_for_output` caveat**: `'Serving world'` never matches — use
  `'Entity creation successful'` instead (fires when `ros_gz_sim create`
  completes, confirming Gazebo is running and the model is spawned).
- **Pose monitoring**: The `Pose_V → TFMessage` bridge produces empty
  `child_frame_id` fields. Use `gz topic -e` with the dynamic pose topic
  for reliable world-frame pose data.
- **rclpy in on_ready**: Importing `rclpy` in the daemon thread may fail with
  `ModuleNotFoundError` in certain sourcing contexts. Use subprocess-based
  monitoring (`gz topic` CLI) to avoid this dependency.
- **Iterative workflow**: Launch → observe → shutdown → adjust parameters →
  rebuild → relaunch. Each iteration takes ~30 seconds.

See [Launch Tooling](launch_tooling.md) for full `LaunchSession` and
`DisplayUserInterface` API details.

## Instance Management

### Detection

Gazebo has no built-in `--list` command. Use these approaches:

```bash
# Process-level detection
pgrep -af "gz sim"

# Transport-level detection (lists active Gazebo topics)
gz topic -l | grep "/world/"

# Check specific discovery ports
ss -ulnp | grep -E '1031[78]'
```

### Prevention

**Pre-launch guard** — add to launch files or wrapper scripts:

```bash
#!/bin/bash
if pgrep -f "gz sim" > /dev/null 2>&1; then
    echo "Error: Gazebo is already running. Stop it first:"
    echo "  pkill -f 'gz sim'"
    exit 1
fi
ros2 launch my_package my_launch.py "$@"
```

**Transport isolation** with `GZ_PARTITION`:

```bash
# Each session gets its own Gazebo transport partition
export GZ_PARTITION="agent_$(id -u)_$$"
ros2 launch my_package my_launch.py
```

This prevents topic cross-talk between instances but does not prevent
resource contention (GPU memory, ports).

**Port awareness**: Gazebo uses UDP multicast for discovery:
- `GZ_DISCOVERY_MSG_PORT` — default 10317
- `GZ_DISCOVERY_SRV_PORT` — default 10318

Override these for true multi-instance isolation (each instance needs unique ports).

## Common Pitfalls

| Pitfall | Details | Fix |
|---------|---------|-----|
| `on_exit_shutdown` defaults to `false` | Closing Gazebo leaves ROS launch running | Set `on_exit_shutdown: 'true'` explicitly |
| SIGTERM not forwarded to gz-sim | `shell=True` in `gz_sim.launch.py` wraps the process in a shell, which intercepts SIGTERM | Fixed in gz-sim 8.8.x+ (backport of [gz-sim#2747](https://github.com/gazebosim/gz-sim/pull/2747)); check your version with `gz sim --version` |
| `parameter_bridge` double-SIGINT | Bridge sometimes needs two SIGINTs to exit cleanly | Known issue; ensure launch shutdown handler sends SIGINT then waits before escalating |
| `Pose_V → TFMessage` bridge: empty frame IDs | `Pose_V` messages don't carry frame names that map to TF | Use `gz topic -e` for world-frame pose data instead |
| URDF `<gazebo>` block plugin loading | System plugins in a model-level `<gazebo>` block without `reference` attribute silently don't load | Use `<gazebo reference="link_name">` — Gz may emit a warning but the plugin loads |
| rclpy import in daemon thread | `ModuleNotFoundError` in `LaunchSession` `on_ready` callback | Use subprocess monitoring (`gz topic` CLI) instead |
| Multiple Gazebo instances | Re-running launch without stopping previous instance causes port conflicts | Add pre-launch guard (see Instance Management above) |

## Version Compatibility

```bash
# Check gz-sim version (SIGTERM fix requires 8.8.x+)
gz sim --version

# Check ros_gz packages (Jazzy)
ros2 pkg list | grep ros_gz
# Expected: ros_gz_bridge, ros_gz_image, ros_gz_sim, ros_gz_interfaces
```

Key versions for Jazzy (ROS 2):
- **gz-sim**: Harmonic (gz-sim 8.x) — SIGTERM fix in 8.8.0+
- **ros_gz**: Jazzy branch — `on_exit_shutdown` parameter available

## References

- [ros_gz repository](https://github.com/gazebosim/ros_gz) — ROS + Gazebo integration
- [Gazebo Sim documentation](https://gazebosim.org/docs/harmonic) — Official docs
- [gz-sim#2747](https://github.com/gazebosim/gz-sim/pull/2747) — SIGTERM propagation fix
- [ROS 2 launch event handlers](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Using-Event-Handlers.html) — `OnProcessExit`, `RegisterEventHandler`
- [Launch Tooling](launch_tooling.md) — `LaunchSession` and `ros2launch_gui` API
- [ROS 2 Development Patterns](ros2_development_patterns.md) — Package structure and launch file conventions
