# Gemini CLI Instructions

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

## Gemini CLI-Specific Notes

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

### Google Cloud Integration

If Gemini CLI has access to Google Cloud services, you may be able to use additional capabilities. Check your environment for available integrations.

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
source .agent/scripts/set_git_identity_env.sh "Gemini CLI Agent" "roland+gemini-cli@ccom.unh.edu"

# Or use the shorthand:
source .agent/scripts/set_git_identity_env.sh --agent gemini

# 3. Check workspace status
.agent/scripts/status_report.sh
```

**📚 Full details**: [`.agent/AI_CLI_QUICKSTART.md`](../.agent/AI_CLI_QUICKSTART.md)

---

## Role Assignment

By default, Gemini CLI acts as **ROS Developer**:
- ✅ Create/modify ROS packages
- ✅ Write/update tests
- ✅ Update package documentation
- ❌ Modify `.agent/` infrastructure (unless explicitly assigned Framework Engineer role)

See [`.agent/PERMISSIONS.md`](../.agent/PERMISSIONS.md) for full role definitions.

---

## GitHub Integration

If you have GitHub CLI (`gh`) installed, use it for faster GitHub operations:

```bash
gh pr list                          # List open PRs
gh issue list --assignee @me        # Your assigned issues
gh pr view 123                      # View PR details
gh issue view 46                    # View issue details
```

Otherwise, the workspace falls back to web API calls (slower but still functional).

---

**Next Steps**:
- Read [`.agent/AI_CLI_QUICKSTART.md`](../.agent/AI_CLI_QUICKSTART.md) for fast onboarding
- Reference [`.agent/AI_RULES.md`](../.agent/AI_RULES.md) for complete workflow
- Use [`.agent/CLI_COMMANDS.md`](../.agent/CLI_COMMANDS.md) to discover workflows

---

**Last Updated**: 2026-01-27  
**Related**: [AI_RULES.md](../.agent/AI_RULES.md), [AI_CLI_QUICKSTART.md](../.agent/AI_CLI_QUICKSTART.md), [CLI_COMMANDS.md](../.agent/CLI_COMMANDS.md)
