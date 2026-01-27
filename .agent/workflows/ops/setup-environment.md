---
description: One-command setup for first-time CLI agent users
---

# Setup Environment Workflow

Automates the complete initial setup for CLI agents. This workflow is **idempotent** - safe to run multiple times.

## Quick Usage

```bash
# For Copilot CLI:
/setup-environment copilot

# For Gemini CLI:
/setup-environment gemini

# Auto-detect:
/setup-environment
```

## What This Workflow Does

1. ✅ Sources ROS 2 environment
2. ✅ Auto-detects or configures git identity
3. ✅ Runs initial status check
4. ✅ Verifies all prerequisites

## Manual Steps (If Not Using Workflow Command)

### Step 1: Source ROS Environment

```bash
source .agent/scripts/env.sh
```

**Verify**:
```bash
ros2 --version  # Should show ROS 2 version
colcon version  # Should show colcon version
```

### Step 2: Configure Git Identity

**Option A: Auto-detect framework (Recommended)**
```bash
source .agent/scripts/set_git_identity_env.sh --detect
```

**Option B: Specify framework**
```bash
# For Copilot CLI:
source .agent/scripts/set_git_identity_env.sh --agent copilot

# For Gemini CLI:
source .agent/scripts/set_git_identity_env.sh --agent gemini
```

**Option C: Manual identity**
```bash
source .agent/scripts/set_git_identity_env.sh "Your Agent Name" "your+email@example.com"
```

**Verify**:
```bash
git config user.name   # Should show your agent name
git config user.email  # Should show your agent email
```

### Step 3: Run Initial Status Check

```bash
.agent/scripts/status_report.sh
```

This will show:
- Git status across all repositories
- Build status of workspaces
- GitHub PR/issue status (if `gh` CLI available)
- Action items requiring attention

### Step 4: Verify Prerequisites

```bash
echo "=== Environment Verification ==="

# Check ROS
if command -v ros2 &> /dev/null; then
    echo "✅ ROS 2: $(ros2 --version)"
else
    echo "❌ ROS 2: Not found (check env.sh)"
fi

# Check colcon
if command -v colcon &> /dev/null; then
    echo "✅ colcon: $(colcon version)"
else
    echo "❌ colcon: Not found"
fi

# Check vcstool
if command -v vcs &> /dev/null; then
    echo "✅ vcstool: Installed"
else
    echo "⚠️  vcstool: Not found (optional, but recommended)"
    echo "    Install: pip install vcstool"
fi

# Check GitHub CLI (optional but recommended for CLI agents)
if command -v gh &> /dev/null; then
    echo "✅ GitHub CLI: $(gh --version | head -n1)"
else
    echo "⚠️  GitHub CLI: Not found (optional, for faster GitHub ops)"
    echo "    Install: https://cli.github.com/"
fi

# Check git identity
echo "✅ Git identity: $(git config user.name) <$(git config user.email)>"

echo ""
echo "=== Framework Detection ==="
source .agent/scripts/detect_cli_env.sh
echo "Detected framework: $AGENT_FRAMEWORK"
if [ -n "$AGENT_FRAMEWORK_VERSION" ]; then
    echo "Version: $AGENT_FRAMEWORK_VERSION"
fi
```

## Complete Setup Script

For convenience, here's a complete setup script you can run:

```bash
#!/bin/bash
# Quick setup for CLI agents

set -e  # Exit on error

echo "=== AI CLI Agent Environment Setup ==="
echo ""

# Step 1: Source ROS environment
echo "Step 1: Sourcing ROS environment..."
source .agent/scripts/env.sh
echo "✅ ROS environment loaded"
echo ""

# Step 2: Configure git identity
echo "Step 2: Configuring git identity..."
if [ $# -eq 1 ]; then
    # Framework specified
    source .agent/scripts/set_git_identity_env.sh --agent "$1"
else
    # Auto-detect
    source .agent/scripts/set_git_identity_env.sh --detect || \
    {
        echo "Auto-detection failed. Please specify framework:"
        echo "  $0 copilot"
        echo "  $0 gemini"
        exit 1
    }
fi
echo ""

# Step 3: Verify setup
echo "Step 3: Verifying setup..."
echo "  ROS 2: $(ros2 --version 2>&1 | head -n1)"
echo "  colcon: $(colcon version 2>&1 | head -n1)"
echo "  Git identity: $(git config user.name) <$(git config user.email)>"
echo "✅ Setup verified"
echo ""

# Step 4: Status check
echo "Step 4: Running initial status check..."
.agent/scripts/status_report.sh

echo ""
echo "=== Setup Complete! ==="
echo ""
echo "Next steps:"
echo "  - Check GitHub Issues for available tasks: gh issue list"
echo "  - Create a feature branch: git checkout -b feature/your-task"
echo "  - Start developing!"
echo ""
echo "Common commands:"
echo "  /check-status      - Full workspace status"
echo "  /build             - Build specific package"
echo "  /build-all         - Build all workspaces"
echo "  /test-all          - Run all tests"
echo ""
echo "See .agent/CLI_COMMANDS.md for more workflows"
```

## Troubleshooting

### "ros2: command not found"

**Cause**: ROS environment not sourced or ROS not installed

**Fix**:
```bash
# Check if ROS is installed
ls /opt/ros/

# If installed, source it
source /opt/ros/humble/setup.bash  # Adjust version if needed

# Or use the workspace env script
source .agent/scripts/env.sh
```

### "Could not auto-detect framework"

**Cause**: Running in unknown environment

**Fix**: Specify framework manually:
```bash
source .agent/scripts/set_git_identity_env.sh --agent copilot
# Or
source .agent/scripts/set_git_identity_env.sh "My Agent" "my+agent@example.com"
```

### "Permission denied" when running scripts

**Fix**:
```bash
chmod +x .agent/scripts/*.sh
```

### Git identity not persisting

**Cause**: Script executed instead of sourced

**Fix**: Use `source` keyword:
```bash
source .agent/scripts/set_git_identity_env.sh --agent copilot
# NOT: ./.agent/scripts/set_git_identity_env.sh --agent copilot
```

## When to Re-run Setup

Run this workflow again when:
- ✅ Starting a new shell session (ROS env and git identity are session-only)
- ✅ Switching between agent frameworks
- ✅ After system restart
- ❌ Not needed if already set up in current session (idempotent but adds overhead)

## Related Workflows

- `/check-status` - Verify current state after setup
- `/start-feature` - Begin working on a task
- See `.agent/AI_CLI_QUICKSTART.md` for fast CLI onboarding

---

**Last Updated**: 2026-01-27  
**Maintained By**: Framework Engineering Team
