---
name: ros2-workspace-instructions
description: Instructions for working with ROS2 layers, packages, and colcon builds
applies_to:
  - "layers/**/*"
  - "configs/*.repos"
  - "scripts/*"
---

# ROS2 Workspace Instructions

When working with files in `layers/`, `configs/`, or `scripts/`, refer to:

- **`.agent/knowledge/ros2_development_patterns.md`** - ROS2 package structure, colcon usage, vcstool commands, workspace overlaying, dependencies, testing
- **`.agent/knowledge/ros2_cli_best_practices.md`** - Runtime ROS2 CLI commands for topics, services, nodes

These files contain comprehensive ROS2 development patterns and conventions for this workspace.

## Common Patterns

**Adding a repository to a layer:**
```bash
# Edit the .repos file in the key repository's config/repos/ directory
# Add entry under repositories:
repositories:
  my_new_package:
    type: git
    url: https://github.com/org/my_new_package.git
    version: main
```

**Building after changes:**
```bash
cd layers/main/core_ws
colcon build --symlink-install --packages-select my_package
# Source using the env script to get proper underlay and overlays
source ../../.agent/scripts/env.sh
```

**Running tests for a package:**
```bash
colcon test --packages-select my_package
colcon test-result --verbose
```
