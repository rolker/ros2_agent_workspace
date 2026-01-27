# AI CLI Agent Quick Start Guide

**For**: GitHub Copilot CLI, Gemini CLI, and similar command-line AI agent frameworks  
**Time to complete**: ~5 minutes  
**Goal**: Get productive quickly without reading hundreds of pages of documentation

---

## Who This Guide Is For

You're in the right place if:
- ✅ You're using an AI CLI tool (Copilot CLI, Gemini CLI, etc.)
- ✅ You want to start working on ROS2 packages immediately
- ✅ You share the workspace with a human user

**Not using a CLI agent?** See [AGENT_ONBOARDING.md](AGENT_ONBOARDING.md) for detailed instructions.

---

## Quick Detection: Which Agent Am I?

Run this to identify your framework:

```bash
echo "Check environment variables:"
env | grep -i "copilot\|gemini\|claude\|openai"
echo "Current user: $USER"
```

**Common patterns**:
- GitHub Copilot CLI: May have `GITHUB_*` env vars, `gh copilot` command available
- Gemini CLI: May have `GEMINI_*` or `GOOGLE_*` env vars
- When in doubt: Ask the human user which CLI tool they invoked

---

## 3-Step Setup (Do This First)

### Step 1: Source the ROS Environment (30 seconds)

```bash
source .agent/scripts/env.sh
```

**Verify it worked**:
```bash
ros2 --version  # Should show ROS 2 Humble or similar
colcon version  # Should show colcon version
```

If commands not found, see [Troubleshooting](#troubleshooting) below.

---

### Step 2: Configure Git Identity (1 minute)

**CRITICAL**: You must set your git identity to distinguish your commits from human commits.

```bash
# For Copilot CLI:
source .agent/scripts/set_git_identity_env.sh "Copilot CLI Agent" "roland+copilot-cli@ccom.unh.edu"

# For Gemini CLI:
source .agent/scripts/set_git_identity_env.sh "Gemini CLI Agent" "roland+gemini-cli@ccom.unh.edu"

# For other CLI frameworks:
source .agent/scripts/set_git_identity_env.sh "<Your Framework> Agent" "roland+<framework>@ccom.unh.edu"
```

**Verify it worked**:
```bash
git config user.name   # Should show: "<Your Framework> Agent"
git config user.email  # Should show: "roland+<framework>@ccom.unh.edu"
```

**Why this method?**
- Ephemeral (session-only, doesn't modify `.git/config`)
- Keeps user's identity intact for their commits
- Safe for shared workspaces

**📚 More details**: [AI_IDENTITY_STRATEGY.md](AI_IDENTITY_STRATEGY.md)

---

### Step 3: Check Workspace Status (1 minute)

Before starting work, see what's currently happening:

```bash
.agent/scripts/status_report.sh
```

**Or use the workflow command**:
```
/check-status
```

**What to look for**:
- ✅ **Clean git status** - Good to go
- ⚠️ **Uncommitted changes** - Ask user if they want them committed/stashed/reverted
- ⚠️ **Already on feature branch** - Verify this is the right branch for your task
- ⚠️ **Open PRs** - Check if your task overlaps with existing work

---

## ✅ You're Ready! Now What?

### Option A: Start a New Task

1. **Check GitHub Issues** for available work:
   ```
   /check-github-status
   ```
   Or manually: https://github.com/rolker/ros2_agent_workspace/issues

2. **Create a feature branch**:
   ```bash
   git checkout -b feature/TASK-46-cli-integration
   ```

3. **Work on your task** (write code, run tests, etc.)

4. **Commit your changes**:
   ```bash
   git add <files>
   git commit -m "Implement CLI integration

   - Added quickstart guide
   - Updated workflows for CLI detection
   
   Closes #46"
   ```

5. **Push and create PR** when ready:
   ```
   /submit-pr
   ```

### Option B: Use Existing Workflows

Common commands (see [CLI_COMMANDS.md](CLI_COMMANDS.md) for full list):

```bash
/build              # Build current workspace
/build-all          # Build all workspaces
/test-all           # Run all tests
/clean              # Clean build artifacts
/add-repo           # Add new ROS package
/check-status       # Comprehensive status check
```

### Option C: Direct ROS Development

Work directly with ROS tools:

```bash
# Build specific package
cd workspaces/ros2_ws
colcon build --packages-select <package_name>

# Run tests
colcon test --packages-select <package_name>

# Run a node
source install/setup.bash
ros2 run <package_name> <node_name>
```

**📚 ROS Patterns**: [knowledge/ros2_development_patterns.md](knowledge/ros2_development_patterns.md)

---

## Core Rules (Quick Reference)

### Git Hygiene 🔴 CRITICAL

- ❌ **NEVER commit to `main`** - Always use feature branches
- ✅ Branch naming: `feature/TASK-<ID>-description` or `fix/description`
- ✅ Atomic commits (one logical change per commit)
- ✅ Always stash or commit before finishing your session

**Full rules**: [rules/common/git-hygiene.md](rules/common/git-hygiene.md)

### AI Signature (Required)

All GitHub content (Issues, PRs, Comments) **must** end with:

```markdown
**🤖 Authored-By**: Copilot CLI Agent
```

(Replace with your framework name)

**Why**: Distinguishes AI-generated content from human-generated content.

### Clean Workspace

- ❌ Don't leave build artifacts in repo root
- ✅ Build in `workspaces/ros2_ws/` (or workspace-specific dir)
- ✅ Use `.agent/scratchpad/` for temporary files (logs, analysis, etc.)

---

## Essential Documentation (Read When Needed)

You don't need to read everything upfront, but bookmark these:

**Core Workflow**:
- [AI_RULES.md](AI_RULES.md) - Complete universal rules (you just read the quick version)
- [WORKFORCE_PROTOCOL.md](WORKFORCE_PROTOCOL.md) - Multi-agent coordination

**ROS Development**:
- [knowledge/ros2_development_patterns.md](knowledge/ros2_development_patterns.md) - Package structure, colcon, vcstool
- [knowledge/ros2_cli_best_practices.md](knowledge/ros2_cli_best_practices.md) - Runtime ROS commands

**When Things Break**:
- [AI_IDENTITY_STRATEGY.md](AI_IDENTITY_STRATEGY.md) - Identity configuration details
- [rules/common/git-hygiene.md](rules/common/git-hygiene.md) - Git workflow
- [PERMISSIONS.md](PERMISSIONS.md) - What you can/can't modify

---

## Troubleshooting

### `ros2: command not found`

**Cause**: ROS environment not sourced  
**Fix**:
```bash
source .agent/scripts/env.sh
```

Add this to your workflow scripts if they need ROS commands.

---

### `colcon: command not found`

**Cause**: Same as above (ROS environment not sourced)  
**Fix**: Same as above

---

### Git commits use wrong name/email

**Cause**: Identity not configured or using wrong method  
**Fix**:
```bash
# For CLI agents (ephemeral):
source .agent/scripts/set_git_identity_env.sh "Copilot CLI Agent" "roland+copilot-cli@ccom.unh.edu"

# Verify:
git config user.name
git config user.email
```

**Still wrong?** Check that you're using `source` (not just running the script), and that the script succeeded.

---

### "Uncommitted changes" warning

**What it means**: Files were modified but not committed (by you or another process)

**What to do**:
1. Check what changed:
   ```bash
   git status
   git diff
   ```

2. Ask user:
   "I see uncommitted changes in `<file>`. Should I include them in my work, stash them, or revert them?"

3. Take appropriate action based on user response

---

### Workflow script fails with permission error

**Fix**:
```bash
chmod +x .agent/scripts/<script_name>.sh
```

Then retry.

---

### "Already on branch feature/..." warning

**What it means**: You're already on a feature branch (good!) but verify it's the right one for your task.

**What to do**:
- If it's your branch: Continue working
- If it's someone else's: Create a new branch (`git checkout -b feature/your-task`)
- If unsure: Ask user

---

## FAQ

### Q: Do I need to read AGENT_ONBOARDING.md?

**A**: No! That's for non-CLI agents. This quick start covers everything CLI agents need.

### Q: What role am I? (ROS Developer, Framework Engineer, Ops Agent)

**A**: By default, CLI agents act as **ROS Developer** - you can:
- ✅ Create/modify ROS packages
- ✅ Write/update tests
- ✅ Update package documentation
- ❌ Modify workspace infrastructure (unless explicitly instructed)
- ❌ Change `.agent/` framework files (unless explicitly instructed)

See [PERMISSIONS.md](PERMISSIONS.md) for full details.

### Q: Can I modify files in `.agent/` or `.github/`?

**A**: Only if you're explicitly assigned a **Framework Engineer** task. Default role is **ROS Developer**.

### Q: How do I know which workflow to use?

**A**: See [CLI_COMMANDS.md](CLI_COMMANDS.md) for a complete mapping of workflows and when to use them.

### Q: What if my CLI framework isn't listed?

**A**: Use the generic pattern:
```bash
source .agent/scripts/set_git_identity_env.sh "<Your Framework> Agent" "roland+<framework-slug>@ccom.unh.edu"
```

Example: "MyAI CLI Agent" / "roland+myai-cli@ccom.unh.edu"

---

## What's Next?

After your first task:
- ✅ Leave the workspace clean (commit or stash changes)
- ✅ Verify status: `.agent/scripts/status_report.sh`
- ✅ Create PR if task is complete: `/submit-pr`
- ✅ Update issue status on GitHub

---

## Framework-Specific Notes

### GitHub Copilot CLI

- Native GitHub integration available (use for faster PR/issue checks)
- May have access to GitHub MCP server tools
- Check `.github/copilot-instructions.md` for platform-specific hints

### Gemini CLI

- Check `.agent/instructions/gemini-cli.instructions.md` for platform-specific hints
- May have Google Cloud integrations available

### Other CLI Frameworks

- Follow this guide as-is
- If you discover framework-specific patterns, document them in `.agent/instructions/<framework>.instructions.md`

---

**Next Steps**:
- ✅ Complete the 3-step setup above
- ✅ Pick a task from GitHub Issues or ask the user what to work on
- ✅ Use workflows in [CLI_COMMANDS.md](CLI_COMMANDS.md) for common operations
- ✅ Read [AI_RULES.md](AI_RULES.md) when you need complete details

**Questions?** Check [AI_RULES.md](AI_RULES.md) or ask the user.

---

**Last Updated**: 2026-01-27  
**Maintained By**: Framework Engineering Team  
**Related**: [AI_RULES.md](AI_RULES.md), [CLI_COMMANDS.md](CLI_COMMANDS.md), [AGENT_ONBOARDING.md](AGENT_ONBOARDING.md)
