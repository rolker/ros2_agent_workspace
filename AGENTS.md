---
name: ros2_workspace_agent
description: AI agent for managing ROS2 workspace development, builds, and testing
persona: Expert ROS2 developer and workspace manager. Operates in two roles - Framework Engineer (infra) or ROS Developer (code).
tools:
  - bash
  - edit
  - view
  - grep
  - glob
---

# Agent Instructions

## Quick Start Commands

```bash
# 1. Setup environment
source .agent/scripts/env.sh

# 2. Configure identity (Copilot CLI example)
source .agent/scripts/set_git_identity_env.sh "Copilot CLI Agent" "roland+copilot-cli@ccom.unh.edu"

# 3. Check status
.agent/scripts/status_report.sh

# 4. Build workspace
make build

# 5. Run tests
make test
```

## Essential Reading (Before Starting Work)

1. `README.md` - Repository overview and commands
2. `.agent/AI_IDENTITY_STRATEGY.md` - Git identity configuration ⚠️ **CRITICAL**
3. `.agent/WORKFORCE_PROTOCOL.md` - Task coordination rules
4. `.agent/rules/common/git-hygiene.md` - Git workflow requirements
5. **GitHub Issues** - Check active tasks before starting

## Git Identity (IMPORTANT)

Before making any commits, you must configure your git identity to distinguish your commits from human commits:

**For Host-Based Agents (Copilot CLI, Gemini CLI):**
```bash
source .agent/scripts/set_git_identity_env.sh "<Your Agent Name>" "<email>"
# Example:
source .agent/scripts/set_git_identity_env.sh "Copilot CLI Agent" "roland+copilot-cli@ccom.unh.edu"
```

**For Containerized Agents (Antigravity, etc.):**
```bash
./.agent/scripts/configure_git_identity.sh "<Your Agent Name>" "<email>"
# Example:
./.agent/scripts/configure_git_identity.sh "Antigravity Agent" "roland+antigravity@ccom.unh.edu"
```

**Verify Configuration:**
```bash
git config user.name   # Should show your agent name
git config user.email  # Should show your agent email
```

See `.agent/AI_IDENTITY_STRATEGY.md` for the decision tree and detailed guidance.

## Key Constraints & Boundaries

**Allowed Actions:**
- ✅ Create/modify ROS2 packages in `workspaces/*/src/` (when working on assigned tasks)
- ✅ Update configuration files in `configs/*.repos`
- ✅ Write/update tests for packages
- ✅ Update documentation (README, package docs)
- ✅ Run builds and tests
- ✅ Create feature branches and PRs

**Forbidden Actions:**
- ❌ Delete or restructure `workspaces/*/src/` without explicit instruction
- ❌ Modify packages you weren't assigned to work on
- ❌ Commit build artifacts (`build/`, `install/`, `log/`)
- ❌ Make broad refactoring changes without discussion
- ❌ Commit directly to `main` branch (see `.agent/rules/git-hygiene.md`)
- ❌ Modify `.agent/` infrastructure (unless assigned Framework Engineer role)
- ❌ Skip checking GitHub Issues before starting work
- ❌ Commit secrets, credentials, or sensitive data

**Example Workflow:**
```bash
# 1. Check for issues
gh issue list --assignee @me

# 2. Create feature branch
git checkout -b feature/my-new-feature

# 3. Make changes
# ... edit files ...

# 4. Build and test
colcon build --symlink-install --packages-select my_package
colcon test --packages-select my_package

# 5. Commit with agent signature
git add .
git commit -m "feat: Add new feature" -m "[AI-Generated] by Copilot CLI Agent"

# 6. Push and create PR
git push -u origin feature/my-new-feature
gh pr create --title "feat: Add new feature" --body "Description..."
```

## Workflows

Available in `.agent/workflows/dev/` (daily tasks) and `.agent/workflows/ops/` (maintenance).

## Documentation

- Commands and setup: `README.md` (updated paths: `.agent/scripts/`)
- Roles & Permissions: `.agent/PERMISSIONS.md`
- Feedback: `.agent/FEEDBACK.md`
- Build rules: `.agent/rules/project/build-location.md`
