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

## ROS2 Conventions

### Naming
- **Packages**: Use lowercase with underscores (e.g., `marine_autonomy`)
- **Nodes**: Descriptive names, lowercase with underscores
- **Topics**: Use namespaces, lowercase with slashes (e.g., `/sensors/gps`)
- **Launch files**: Descriptive with `.launch.py` extension

### Message/Service Types
- Use standard ROS2 message types when possible
- Custom messages go in `<package>/msg/` directory
- Custom services go in `<package>/srv/` directory
- Follow CamelCase for message type names

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

The `.agent/scripts/env.sh` script handles this automatically.

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

# Run tests for a package — env.sh must be sourced in the same shell
# (agents run each command in a fresh subprocess, so colcon test alone will fail)
source .agent/scripts/env.sh && cd layers/main/<layer>_ws && colcon test --packages-select <package_name>

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
