# GitHub Copilot CLI Instructions

**🚀 Quick Start**: See [`.agent/AI_CLI_QUICKSTART.md`](../.agent/AI_CLI_QUICKSTART.md) for a 5-minute setup guide.

---

## Universal Agent Rules

All AI agents (Copilot CLI, Gemini CLI, Antigravity, etc.) follow the same core workflow and rules documented in:

👉 **[`.agent/AI_RULES.md`](../.agent/AI_RULES.md)** - Single source of truth for:
- Essential documentation to read
- Standard 4-step workflow (source env → configure identity → check status → start work)
- Core rules (git hygiene, AI signatures, clean workspace, issue-first)
- Permissions and roles
- Troubleshooting common issues

---

## Copilot CLI-Specific Notes

### Environment & Versions

This workspace uses:
- **ROS 2**: Jazzy (or Humble fallback)
- **Build System**: colcon
- **VCS Tool**: vcstool
- **Python**: 3.10+

### Quick Command Reference

Common workflows are mapped in [`.agent/CLI_COMMANDS.md`](../.agent/CLI_COMMANDS.md):

```bash
/check-status      # Full workspace status
/build             # Build specific package
/build-all         # Build all workspaces
/test-all          # Run all tests
/start-feature     # Create feature branch
/submit-pr         # Create pull request
```

### Build & Test Commands

**Build workspace layers:**
```bash
# Build all layers in order
make build

# Build specific workspace
cd workspaces/core_ws && colcon build --symlink-install

# Build with output
colcon build --symlink-install --event-handlers console_direct+
```

**Run tests:**
```bash
# All tests
make test

# Specific package tests
colcon test --packages-select <package_name>
colcon test-result --verbose
```

**Lint & Validate:**
```bash
# Pre-commit checks
pre-commit run --all-files

# Validate configs
python3 .agent/scripts/validate_repos.py
```

### GitHub Integration

Copilot CLI has native GitHub access. Use it for faster operations:

```bash
gh pr list                          # List open PRs
gh issue list --assignee @me        # Your assigned issues
gh pr view 123                      # View PR details
gh issue view 46                    # View issue details
```

### ROS2 Knowledge

For ROS2 development patterns and CLI usage:
- **[`.agent/knowledge/ros2_development_patterns.md`](../.agent/knowledge/ros2_development_patterns.md)** - Package structure, colcon, vcstool
- **[`.agent/knowledge/ros2_cli_best_practices.md`](../.agent/knowledge/ros2_cli_best_practices.md)** - Runtime ROS commands

---

## First-Time Setup (3 Steps)

If this is your first session in this workspace:

```bash
# 1. Source ROS environment
source .agent/scripts/env.sh

# 2. Configure git identity (ephemeral, session-only)
source .agent/scripts/set_git_identity_env.sh "Copilot CLI Agent" "roland+copilot-cli@ccom.unh.edu"

# 3. Check workspace status
.agent/scripts/status_report.sh
```

**📚 Full details**: [`.agent/AI_CLI_QUICKSTART.md`](../.agent/AI_CLI_QUICKSTART.md)

**Expected Output After Setup:**
```bash
$ source .agent/scripts/env.sh
[ROS 2 Jazzy sourced]

$ .agent/scripts/status_report.sh
✓ ROS 2 environment: Jazzy
✓ Workspace layers: core_ws (3 packages)
✓ Git branch: main
✓ No uncommitted changes
```

---

## Role Assignment & Boundaries

By default, Copilot CLI acts as **ROS Developer**:

**Allowed:**
- ✅ Create/modify ROS packages in `workspaces/*/src/` (when working on assigned tasks)
- ✅ Write/update tests
- ✅ Update package documentation
- ✅ Modify config files in `configs/*.repos`
- ✅ Update build configurations (CMakeLists.txt, package.xml, setup.py)

**Forbidden:**
- ❌ Delete or restructure `workspaces/*/src/` without explicit instruction
- ❌ Modify packages you weren't assigned to work on
- ❌ Modify `.agent/` infrastructure (unless explicitly assigned Framework Engineer role)
- ❌ Commit build artifacts (`build/`, `install/`, `log/`)
- ❌ Commit directly to `main` branch (always use feature branches)
- ❌ Modify `.github/workflows/` without review
- ❌ Commit secrets or credentials

**Code Examples:**

Creating a new ROS2 Python package:
```bash
cd workspaces/core_ws/src
ros2 pkg create --build-type ament_python my_package \
  --dependencies rclpy std_msgs
```

Adding additional dependencies to existing package.xml:
```xml
<!-- Beyond those specified in ros2 pkg create -->
<depend>geometry_msgs</depend>
<depend>tf2_ros</depend>
```

See [`.agent/PERMISSIONS.md`](../.agent/PERMISSIONS.md) for full role definitions.

---

**Next Steps**:
- Read [`.agent/AI_CLI_QUICKSTART.md`](../.agent/AI_CLI_QUICKSTART.md) for fast onboarding
- Reference [`.agent/AI_RULES.md`](../.agent/AI_RULES.md) for complete workflow
- Use [`.agent/CLI_COMMANDS.md`](../.agent/CLI_COMMANDS.md) to discover workflows

---

**Last Updated**: 2026-01-27  
**Related**: [AI_RULES.md](../.agent/AI_RULES.md), [AI_CLI_QUICKSTART.md](../.agent/AI_CLI_QUICKSTART.md), [CLI_COMMANDS.md](../.agent/CLI_COMMANDS.md)
