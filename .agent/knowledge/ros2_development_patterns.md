# ROS2 Development Patterns for Workspace Management

This guide covers ROS2 development patterns specific to this layered workspace architecture.

## ROS2 Package Structure

When creating or modifying ROS2 packages, follow the standard structure:

```
package_name/
├── package.xml          # Package metadata and dependencies
├── CMakeLists.txt       # Build configuration (for C++)
├── setup.py            # Build configuration (for Python)
├── src/                # Source code
├── include/            # Header files (C++)
├── launch/             # Launch files
├── config/             # Configuration files
└── test/               # Test files
```

## Colcon Build System

### Build Commands
```bash
# Build specific packages
colcon build --packages-select <package_name>

# Build with symlink install (faster iteration)
colcon build --symlink-install

# Build with specific CMake args
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Common Build Flags
- `--symlink-install` - Symlink Python scripts and launch files
- `--packages-select` - Build only specified packages
- `--packages-up-to` - Build package and its dependencies
- `--event-handlers console_direct+` - Show detailed output

## vcstool (VCS) Usage

### Repository Management
```bash
# Import repositories from .repos file
vcs import src < config.repos

# Check status of all repositories
vcs status src

# Pull updates for all repositories
vcs pull src

# Export current repository state
vcs export src > snapshot.repos
```

## ROS 2 Conventions

This workspace follows ROS 2 official conventions by default (ADR-0008).
Conventions target Rolling; when working in a different distro, exceptions are
allowed where a newer convention is not yet feasible. Deliberate deviations from
any convention below require their own ADR.

When in doubt, match the existing code in the project repo rather than
introducing a mix.

### Naming

- **Packages**: lowercase with underscores (e.g., `marine_autonomy`). At least
  two characters. Avoid catchall names like `utils`. Don't prefix with `ros`.
  See [REP-144](https://ros.org/reps/rep-0144.html).
- **Nodes**: descriptive, lowercase with underscores
- **Topics / services**: use namespaces, lowercase with slashes (e.g.,
  `/sensors/gps`). See [topic and service name design](https://design.ros2.org/articles/topic_and_service_names.html).

### Launch Files

- **Python**: use `*_launch.py` suffix (e.g., `simulator_launch.py`). The
  filename must end in `launch.py` for `ros2 launch` autocomplete to work.
  See [launch system tutorial](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-system.html).
- **XML / YAML**: use `*_launch.xml` or `*_launch.yaml` suffix
- Place all launch files in the `<package>/launch/` directory
- **Wrong**: `simulator.launch.py`, `my_launch.py`, `start.py`
- **Right**: `simulator_launch.py`, `robot_bringup_launch.py`
- For programmatic launch management and GUI monitoring, see
  [Launch Tooling for Agents](launch_tooling.md)
- For Gazebo simulation launch patterns and lifecycle coupling, see
  [Gazebo + ROS 2 Launch Patterns](gazebo_ros_launch_patterns.md)

### Licensing

- **New packages**: Apache 2.0 is the ROS 2 recommendation. See [migration
  guide](https://docs.ros.org/en/jazzy/How-To-Guides/Migrating-from-ROS1/Migrating-Packages.html).
- **Existing packages**: preserve the declared license. Changing a license
  requires permission from all contributors.
- **Before writing code in a project repo**, check the license:
  1. Read the `LICENSE` file at the repo root
  2. Check `<license>` in `package.xml`
  3. Look at existing source file headers for the pattern in use
  4. Match that pattern exactly in new files
- **Copyright headers are required** in every source file. Include year, author,
  and institution. Example (BSD 2-Clause):
  ```python
  # Copyright 2026 Author Name, Institution
  # All rights reserved.
  #
  # Software License Agreement (BSD 2-Clause Simplified License)
  # [full license text]
  ```
- Use [SPDX license identifiers](https://spdx.org/licenses/) in `package.xml`
  (e.g., `Apache-2.0`, `BSD-2-Clause`, `BSD-3-Clause`, `MIT`)

### Package Manifest (`package.xml`)

- Use format 3: `<package format="3">`
- Use the correct dependency tag for each situation:
  - `<depend>` — needed at build time and run time (most common)
  - `<build_depend>` — needed only at build time
  - `<exec_depend>` — needed only at run time (e.g., Python packages,
    launch-only dependencies)
  - `<test_depend>` — needed only for testing
  - `<buildtool_depend>` — build system tools (e.g., `ament_cmake`,
    `rosidl_default_generators`)
- Include a `<license>` tag with an SPDX identifier
- Include a `<description>` that explains what the package does
- See [REP-140](https://reps.openrobotics.org/rep-0140/) for the full format spec

### Message / Service / Action Types

- Use standard ROS 2 message types when possible
- Custom messages go in `<package>/msg/` directory
- Custom services go in `<package>/srv/` directory
- Custom actions go in `<package>/action/` directory
- Follow CamelCase for type names (e.g., `LaserScan`, `MissionPlan`)

### Code Style

- **C++**: follow the [ROS 2 C++ style guide](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html)
- **Python**: follow PEP 8, max 100 characters per line
- **CMake**: lowercase command names, snake_case for variables and functions

## Workspace Overlaying

This repository uses a layered workspace approach:

1. **Underlay** - Base ROS2 installation and common dependencies
2. **Core** - Marine autonomy framework packages
3. **Overlay layers** - Additional capabilities (UI, sensors, simulation)

### Environment Sourcing Order
```bash
source /opt/ros/<distro>/setup.bash  # ROS2 base
source layers/main/underlay_ws/install/setup.bash
source layers/main/core_ws/install/setup.bash
# ... additional layers
```

The `.agent/scripts/setup.bash` script handles this automatically.

## Dependencies

### Adding Dependencies
1. Update `package.xml` with the dependency
2. If it's a ROS2 package, add to the appropriate `.repos` file in `configs/`
3. Run `rosdep install` to install system dependencies

### rosdep Usage
```bash
# Install dependencies for all packages in workspace
rosdep install --from-paths src --ignore-src -r -y
```

## Testing

### Running Tests
```bash
# Build with tests (from repo root)
cd layers/main/<layer>_ws && colcon build --packages-select <package_name>

# Run tests for a package — setup.bash must be sourced in the same shell
# (agents run each command in a fresh subprocess, so colcon test alone will fail)
source .agent/scripts/setup.bash && cd layers/main/<layer>_ws && colcon test --packages-select <package_name>

# Show test results
colcon test-result --verbose

# Full workspace test (env is handled automatically)
make test
```

## Common Issues

### Build Failures
- Check that all dependencies are installed (`rosdep install`)
- Ensure correct ROS2 distribution is sourced
- Verify package.xml and CMakeLists.txt are correct
- Clean build artifacts: `rm -rf build/ install/ log/`

### Runtime Issues
- Verify environment is sourced correctly
- Check topic/service names with `ros2 topic list` or `ros2 service list`
- Use `ros2 run` to launch nodes individually for debugging

## Best Practices

1. **Always source the workspace** before building or running
2. **Use symlink install** during development for faster iteration
3. **Build incrementally** - test packages individually before full builds
4. **Keep package.xml updated** - ensure all dependencies are listed
5. **Follow ROS2 coding standards** - use rclcpp/rclpy idioms
