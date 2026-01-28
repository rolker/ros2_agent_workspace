# Universal AI Agent Rules

**Purpose**: This document defines the core workflow and rules that apply to **ALL** AI agents working in this ROS2 workspace, regardless of platform (Copilot CLI, Gemini CLI, Antigravity, or custom agents).

> **🚀 Quick Start for CLI Agents**: See [`.agent/AI_CLI_QUICKSTART.md`](AI_CLI_QUICKSTART.md) for a fast-path onboarding guide.

---

## Essential Pre-Work (Read Before Starting ANY Task)

These documents are **required reading** for every agent session:

1. **[`README.md`](../README.md)** - Repository structure, commands, and getting started
2. **[`WORKFORCE_PROTOCOL.md`](WORKFORCE_PROTOCOL.md)** - Task coordination and clean handover standards
3. **[`AI_IDENTITY_STRATEGY.md`](AI_IDENTITY_STRATEGY.md)** ⚠️ **CRITICAL** - How to identify yourself in git commits
4. **[`rules/common/git-hygiene.md`](rules/common/git-hygiene.md)** - Git workflow requirements (never commit to `main`, branch naming)
5. **[`rules/common/ai-signature.md`](rules/common/ai-signature.md)** - Required signature format for GitHub Issues/PRs/Comments
6. **GitHub Issues** - Check open issues for active tasks and backlog

### Additional Role-Specific Documentation

Depending on your role, review:

- **ROS Developer**: [`knowledge/ros2_development_patterns.md`](knowledge/ros2_development_patterns.md), [`knowledge/ros2_cli_best_practices.md`](knowledge/ros2_cli_best_practices.md)
- **Framework Engineer**: [`rules/framework/`](rules/framework/), [`PERMISSIONS.md`](PERMISSIONS.md)
- **Ops Agent**: [`workflows/ops/`](workflows/ops/)

---

## Standard Workflow (4 Steps)

Every agent session should follow this sequence:

### 1. Source the Environment (1 min)

Ensure your shell has the correct ROS 2 environment paths:

```bash
source .agent/scripts/env.sh
```

**Why**: ROS commands, colcon, and vcstool require proper environment setup.

### 2. Configure Git Identity (1 min)

Before making ANY commits, configure your identity using the appropriate method:

#### Host-Based Agents (Copilot CLI, Gemini CLI)

If you're running directly on the host and sharing the working copy with the user:

```bash
source .agent/scripts/set_git_identity_env.sh "Copilot CLI Agent" "roland+copilot-cli@ccom.unh.edu"
```

**Replace with your framework**:
- Copilot CLI: `"Copilot CLI Agent"` / `"roland+copilot-cli@ccom.unh.edu"`
- Gemini CLI: `"Gemini CLI Agent"` / `"roland+gemini-cli@ccom.unh.edu"`

**Why this method?**
- ✅ Session-only (doesn't modify `.git/config`)
- ✅ User's identity remains intact for their own commits
- ✅ Perfect for shared workspaces

#### Container/Isolated Agents (Antigravity, Custom Platforms)

If you're running in a container or separate environment:

```bash
.agent/scripts/configure_git_identity.sh "Antigravity Agent" "roland+antigravity@ccom.unh.edu"
```

**Why this method?**
- ✅ Persistent (survives session restart)
- ✅ Safe (no collision with host user)
- ✅ Standard git configuration

> **📚 Full Details**: See [`AI_IDENTITY_STRATEGY.md`](AI_IDENTITY_STRATEGY.md) for the complete decision tree.

### 3. Check Workspace Status (2 min)

Before starting work, verify the current state:

```bash
.agent/scripts/status_report.sh
```

Or use the workflow:
```
/check-status
```

**What to verify**:
- Git branch status (should be clean or on a feature branch)
- Uncommitted changes (if any, determine ownership)
- Build status of repositories
- Open PRs and issues

**⚠️ Important**: If you see uncommitted changes in repos you didn't touch, **WARN THE USER** before proceeding. See [WORKFORCE_PROTOCOL.md](WORKFORCE_PROTOCOL.md#2-the-clean-handover-standard).

### 4. Start Working

Once environment is set up and status is clean:
- Pick a task from GitHub Issues (or create one if needed)
- Follow git hygiene rules (feature branches, atomic commits)
- Use workflows in `.agent/workflows/` for common tasks
- Add AI signature to all GitHub content

---

## Core Rules (Apply to All Agents)

### Git Hygiene

- **Never commit directly to `main`** - Always use feature branches
- **All changes via Pull Requests** - The `main` branch is protected; direct pushes are blocked
- **Branch naming**: `feature/ISSUE-<ID>-<description>` or `fix/<description>`
- **Atomic commits**: One logical change per commit
- **Stash or commit** uncommitted changes before finishing
- **Clean handover**: Leave workspace in clean state when session ends
- **Stay updated**: Check for updates in default branch before committing (automatic via pre-commit hook)

**Branch Update Check**: Before committing, the pre-commit hook automatically checks if your feature branch is behind the default branch and provides recommendations:
```bash
# Manual check anytime
.agent/scripts/check_branch_updates.sh

# Strict mode (blocks if behind)
.agent/scripts/check_branch_updates.sh --strict
```

**Critical**: After committing to your feature branch, you **must** create a pull request:
```bash
git push -u origin feature/my-task
gh pr create --title "..." --body "Closes #123

---
**🤖 Authored-By**: \`<Your Agent>\`
**🧠 Model**: \`<Model Name>\`"
```

**Reference**: [rules/common/git-hygiene.md](rules/common/git-hygiene.md), [workflows/ops/check-branch-updates.md](workflows/ops/check-branch-updates.md)

### AI Signature

All GitHub Issues, PRs, and Comments **must** include:

```markdown
---
**🤖 Authored-By**: `<Your Agent Name>`
**🧠 Model**: `<Your Actual Model Name>`
```

**⚠️ CRITICAL**: Use your **actual runtime identity**, not example values.

**How to get your identity:**
1. **Check environment variables** (recommended) set during initialization:
   - `$AGENT_NAME` - Your agent name
   - `$AGENT_MODEL` - Your model name
2. **Read from `.agent/.identity` file** (if it exists): `[ -f .agent/.identity ] && source .agent/.identity`
3. **Auto-detect**: Run `.agent/scripts/detect_agent_identity.sh --export`
4. **Fallback**: Use "AI Agent" / "Unknown Model" if detection fails

**Note**: Environment variables are preferred as they're always current. The `.agent/.identity` file is runtime-generated (git-ignored) and may become stale if configuration changes during a session.

**DO NOT copy example model names** (e.g., "GPT-4o", "Gemini 2.0 Flash") from documentation.

Examples (with correct introspection):
- If you're Copilot CLI running GPT-4o:
  ```markdown
  **🤖 Authored-By**: `Copilot CLI Agent`
  **🧠 Model**: `GPT-4o`
  ```
- If you're Gemini CLI running Gemini 2.0 Flash:
  ```markdown
  **🤖 Authored-By**: `Gemini CLI Agent`
  **🧠 Model**: `Gemini 2.0 Flash`
  ```
- If identity cannot be determined:
  ```markdown
  **🤖 Authored-By**: `AI Agent`
  **🧠 Model**: `Unknown Model`
  ```

**Reference**: [rules/common/ai-signature.md](rules/common/ai-signature.md)

### Workspace Cleanliness

- **Keep repository root clean** - No build artifacts, logs, or temporary files
- **Use designated directories**:
  - `.agent/scratchpad/` - Persistent temporary artifacts (logs, reports, analysis)
  - Workspace-specific build dirs (e.g., `workspaces/ros2_ws/build/`, `workspaces/ros2_ws/install/`)
- **Pre-commit hook** checks for suspicious files in source directories

**Reference**: [rules/common/clean-root.md](rules/common/clean-root.md)

### Issue-First Workflow

- **Check GitHub Issues** before starting work
- **Assign or comment** "Taking this" to claim a task
- **Reference issues** in commits and PRs (e.g., "Closes #123")
- **Don't duplicate** work that's already assigned to another agent

**Reference**: [rules/common/issue-first.md](rules/common/issue-first.md)

### Build Location

- **Always build in workspace directories**, never in repository root
- **Standard location**: `workspaces/ros2_ws/` (or workspace-specific dir)
- **Use colcon** for ROS 2 builds: `cd workspaces/ros2_ws && colcon build`

**Reference**: [rules/project/build-location.md](rules/project/build-location.md)

---

## Workflows & Commands

### Discovering Workflows

All available workflows are documented in:
- **[`.agent/AGENT_INDEX.md`](AGENT_INDEX.md)** - Complete catalog
- **[`.agent/CLI_COMMANDS.md`](CLI_COMMANDS.md)** - CLI-friendly command mapping

### Common Workflows

Located in `.agent/workflows/`:

**Operations** (`workflows/ops/`):
- `/check-status` - Comprehensive status report
- `/build` - Build specific workspace or package
- `/build-all` - Build all workspaces
- `/clean` - Clean build artifacts
- `/check-local-status` - Quick local git status

**Development** (`workflows/dev/`):
- `/start-feature` - Create feature branch and issue
- `/finish-feature` - Finalize and prepare for PR
- `/submit-pr` - Create pull request
- `/add-repo` - Add new ROS package to workspace
- `/test-all` - Run all tests

**Improvement** (`workflows/ops/`):
- `/continuous-improvement` - Identify and report infrastructure friction

---

## Session Best Practices

### Report Infrastructure Friction

At the **end of significant work sessions**, take a moment to identify and report infrastructure friction points:

**What to report:**
- Unclear or incomplete documentation
- Workflows that required workarounds
- Scripts that failed or behaved unexpectedly
- Repetitive manual steps that could be automated
- Hard-to-discover tools or features

**How to report:**
```bash
# Use the continuous improvement workflow
cat .agent/workflows/ops/continuous-improvement.md

# Draft issues using the template
cat .agent/templates/improvement_issue.md
```

**Process:**
1. Review session for friction points
2. Check for duplicate issues: `gh issue list --search "topic"`
3. Draft well-formed issues with examples and session evidence
4. Show drafts to user for approval
5. Create approved issues: `gh issue create --label "enhancement,agent-infrastructure"`

**Benefits:**
- ✅ Helps all future agents work more efficiently
- ✅ Captures improvement opportunities while context is fresh
- ✅ Builds knowledge base of real-world friction points
- ✅ Drives continuous improvement of agent infrastructure

**Reference**: [workflows/ops/continuous-improvement.md](workflows/ops/continuous-improvement.md), [CONTRIBUTING.md](../CONTRIBUTING.md#reporting-infrastructure-friction)

---

## Permissions & Roles

Agents are assigned roles with specific permissions:

- **ROS Developer**: Package development, testing, documentation
- **Framework Engineer**: Infrastructure changes, workflow updates, repo-wide refactoring
- **Ops Agent**: Status checks, builds, cleanup

**Reference**: [PERMISSIONS.md](PERMISSIONS.md)

**Note**: CLI agents typically operate as **ROS Developer** unless explicitly instructed otherwise.

---

## When Things Go Wrong

### Uncommitted Changes in Other Repos

**Stop** → **Diff** → **Ask User**:
```bash
git diff path/to/file
```

"I see uncommitted changes in `foo.cpp`. Should I include them, revert them, or stash them?"

### ROS Environment Not Sourced

Symptom: `colcon: command not found` or `ros2: command not found`

Solution:
```bash
source .agent/scripts/env.sh
```

### Git Identity Not Set

Symptom: Commits fail or use wrong identity

Solution: Follow Step 2 above (Configure Git Identity)

### Workflow Script Fails

1. Check if ROS environment is sourced
2. Verify you're in correct directory
3. Check script permissions: `chmod +x .agent/scripts/<script>.sh`
4. Review error output and consult workflow documentation

---

## Framework-Specific Guides

For platform-specific features and optimizations:

- **GitHub Copilot CLI**: [`.github/copilot-instructions.md`](../.github/copilot-instructions.md)
- **Gemini CLI**: [`.agent/instructions/gemini-cli.instructions.md`](.agent/instructions/gemini-cli.instructions.md)
- **Antigravity**: Use this document (no special instructions)

---

## Summary

✅ **Read** required documentation (README, WORKFORCE_PROTOCOL, AI_IDENTITY_STRATEGY, git-hygiene, ai-signature)  
✅ **Source** environment (`.agent/scripts/env.sh`)  
✅ **Configure** git identity (ephemeral for CLI agents, persistent for isolated agents)  
✅ **Check** workspace status (`.agent/scripts/status_report.sh` or `/check-status`)  
✅ **Follow** core rules (git hygiene, AI signature, clean workspace, issue-first)  
✅ **Use** workflows for common tasks  
✅ **Handover** cleanly (commit to feature branch, verify status)

---

**Last Updated**: 2026-01-27  
**Maintained By**: Framework Engineering Team  
**Related**: [AI_CLI_QUICKSTART.md](AI_CLI_QUICKSTART.md), [AGENT_ONBOARDING.md](AGENT_ONBOARDING.md), [WORKFORCE_PROTOCOL.md](WORKFORCE_PROTOCOL.md)
