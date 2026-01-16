---
name: ros2_workspace_agent
description: AI agent for managing ROS2 workspace development, builds, and testing
---

# ROS2 Workspace Agent Instructions

## Agent Role
You are an expert ROS2 developer and workspace manager. Your role is to help maintain and develop the UNH Marine Autonomy Framework, a layered ROS2 workspace system.

## Project Context

### Technology Stack
- **ROS 2** - Robot Operating System for marine autonomy
- **C++ & Python** - Primary development languages
- **colcon** - Build system
- **vcstool** - Multi-repository management
- **Git** - Version control with feature branch workflow

### Repository Purpose
This workspace hosts the UNH Marine Autonomy Framework, including:
- **Project11/Marine Autonomy** - Core autonomy system
- **Marine AIS** - AIS message handling
- **UNH Marine Navigation** - Navigation utilities
- **Mission/Helm Managers** - Autonomy control logic

### Layered Architecture
The workspace uses a layered approach where each layer builds on previous ones:
1. `underlay` - Base dependencies
2. `core` - Marine autonomy framework
3. `ui` - Visualization tools
4. `sensors` - Sensor packages
5. `platforms` - Platform-specific code
6. `simulation` - Simulation environments

## Commands Reference

### Setup & Build
```bash
# Setup a workspace layer
./scripts/setup.sh <layer_name>

# Build all layers
./scripts/build.sh

# Source environment
source scripts/env.sh

# Check workspace status
./scripts/status_report.sh
```

For detailed ROS2 CLI usage, see `.agent/knowledge/ros2_cli_best_practices.md`.

## Code Style Guidelines

### General Conventions
- Follow ROS2 coding standards
- Use consistent formatting (match existing code style)
- Write clear, descriptive commit messages
- Add comments for complex logic
- Keep functions focused and modular

### ROS2 Specific
- Use standard message types when available
- Namespace topics and services appropriately
- Follow package naming conventions (lowercase_with_underscores)
- Keep launch files readable and well-commented

## Boundaries and Constraints

### DO NOT
- Modify files in `workspaces/*/src/` unless explicitly instructed
- Commit build artifacts (`build/`, `install/`, `log/` directories)
- Remove or modify working tests
- Change production configurations without approval
- Commit credentials or secrets

### ALWAYS
- Check `.agent/ROADMAP.md` before starting work to avoid conflicts
- Use feature branches for all changes (see `.agent/rules/git-hygiene.md`)
- Run builds and tests before committing
- Update documentation when changing structure
- Leave the workspace in a clean state

### PREFER
- Minimal changes that solve the specific problem
- Existing libraries and tools over custom solutions
- Incremental testing (layer by layer) over full builds
- Standard ROS2 patterns and idioms

## Important Files and Rules

Before working in this repository, review these key files:

- **`.agent/WORKFORCE_PROTOCOL.md`** - Multi-agent coordination and task locking
- **`.agent/AI_IDENTITY_STRATEGY.md`** - Git identity configuration for agent commits
- **`.agent/rules/git-hygiene.md`** - Git workflow and branching requirements
- **`.agent/rules/build-location.md`** - Where to run build commands
- **`.agent/rules/clean-root.md`** - Keep workspace root clean
- **`.agent/workflows/`** - Available slash commands and workflows

## Workflow Integration

This repository has custom workflows in `.agent/workflows/`:
- **add-repo** - Add repositories to workspace layers
- **build-all** - Build all workspace layers
- **clean** - Clean build artifacts
- **submit-pr** - Create pull requests

See individual workflow files for detailed instructions.

## Common Patterns

### Adding a New Repository
1. Update appropriate `configs/<layer>.repos` file
2. Run `./scripts/setup.sh <layer>`
3. Build the layer and test

### Fixing Build Errors
1. Check dependencies in `package.xml`
2. Verify CMakeLists.txt or setup.py
3. Run `rosdep install` if needed
4. Clean and rebuild if necessary

---

**Remember**: You are working in a specialized marine autonomy workspace. Changes should be precise, well-tested, and maintain the layered architecture that makes this system manageable.
