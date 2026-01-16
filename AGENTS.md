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

# Clean build artifacts
./scripts/clean.sh

# Source environment
source scripts/env.sh

# Check workspace status
./scripts/status_report.sh
```

### ROS2 Operations
```bash
# Build specific packages
colcon build --packages-select <package>

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# VCS operations
vcs import src < configs/<layer>.repos
vcs status src
vcs pull src
```

## Code Style Guidelines

### General Conventions
- Follow ROS2 coding standards
- Use consistent formatting (match existing code style)
- Write clear, descriptive commit messages
- Add comments for complex logic
- Keep functions focused and modular

### C++ Style
- Use `rclcpp` idioms for ROS2 code
- Follow Google C++ Style Guide
- Use smart pointers appropriately
- Prefer `const` correctness

### Python Style
- Follow PEP 8
- Use `rclpy` for ROS2 nodes
- Type hints for function signatures
- Docstrings for classes and functions

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
- Make broad refactoring changes without discussion

### ALWAYS
- Check `.agent/ROADMAP.md` before starting work to avoid conflicts
- Use feature branches for all changes
- Run builds and tests before committing
- Update documentation when changing structure
- Leave the workspace in a clean state
- Use the Antigravity Agent git identity for commits

### PREFER
- Minimal changes that solve the specific problem
- Existing libraries and tools over custom solutions
- Incremental testing (layer by layer) over full builds
- Standard ROS2 patterns and idioms
- Atomic commits with clear messages

## Git Workflow

### Branching
- Create feature branches: `feature/<description>`
- Keep branches focused on single features/fixes
- Clean up branches after merging

### Commits
- Configure agent identity:
  - Name: `Antigravity Agent`
  - Email: `roland+antigravity@ccom.unh.edu`
- Write descriptive commit messages
- Make atomic commits (one logical change per commit)
- Do not mix unrelated changes

### Pull Requests
- Create PRs for all changes
- Include clear description of changes
- Reference related issues
- Ensure builds pass before requesting review

## Workflow Integration

This repository has custom workflows in `.agent/workflows/`. Familiarize yourself with these:

- **add-repo** - Add repositories to workspace layers
- **build-all** - Build all workspace layers
- **clean** - Clean build artifacts
- **rebuild-all** - Full clean rebuild
- **submit-pr** - Create pull requests
- **check_status** - Workspace status report
- **test_all** - Run all tests
- **review_code** - Code review workflow
- **document_code** - Documentation generation

See individual workflow files for detailed instructions.

## Task Coordination

### Before Starting Work
1. Check `.agent/ROADMAP.md` for active tasks
2. Mark your task as `(Status: Active)`
3. Review related issues and documentation

### During Work
1. Make incremental progress
2. Test changes frequently
3. Update ROADMAP status as needed
4. Document significant decisions

### Finishing Work
1. Run final builds and tests
2. Update ROADMAP to `(Status: Done)`
3. Ensure workspace is clean (`git status`)
4. Create PR with clear description

## Testing Strategy

### Build Testing
- Build individual packages first: `colcon build --packages-select <pkg>`
- Build layers incrementally
- Run full build as final verification

### Runtime Testing
- Source the environment correctly
- Test nodes individually before integration
- Verify topic/service communication
- Check for runtime errors and warnings

### Regression Testing
- Ensure existing functionality still works
- Run relevant test suites
- Verify no new build warnings

## Common Patterns

### Adding a New Repository
1. Update appropriate `configs/<layer>.repos` file
2. Run `./scripts/setup.sh <layer>`
3. Build the layer: `colcon build`
4. Test the new packages

### Fixing Build Errors
1. Read the error message carefully
2. Check dependencies in `package.xml`
3. Verify CMakeLists.txt or setup.py
4. Run `rosdep install` if needed
5. Clean and rebuild if necessary

### Updating Documentation
1. Update README.md for structural changes
2. Update relevant workflow files in `.agent/workflows/`
3. Add knowledge to `.agent/knowledge/` if creating new patterns
4. Keep inline documentation synchronized with code

## Examples of Good Work

### Good Commit Message
```
Add GPS sensor support to sensor layer

- Added gps_driver package to sensors.repos
- Updated sensor layer build configuration
- Tested GPS message publishing
```

### Good PR Description
```
## Summary
Adds support for GPS sensors in the sensor workspace layer.

## Changes
- Updated configs/sensors.repos with gps_driver repository
- Verified build succeeds with new package
- Tested GPS node launches and publishes correctly

## Testing
- Built sensor layer: ✓
- Launched GPS node: ✓
- Verified /gps topic: ✓
```

## Agent Identity
When making git commits, you should identify as:
- **Name**: Antigravity Agent
- **Email**: roland+antigravity@ccom.unh.edu

This helps distinguish agent work from human contributions in the git history.

---

**Remember**: You are working in a specialized marine autonomy workspace. Changes should be precise, well-tested, and maintain the layered architecture that makes this system manageable.
