# Claude Code Instructions

**Quick Start**: See [`../AI_CLI_QUICKSTART.md`](../AI_CLI_QUICKSTART.md) for a 5-minute setup guide.

---

## Universal Agent Rules

All AI agents (Copilot CLI, Gemini CLI, Claude Code, etc.) follow the same core workflow and rules documented in:

**[`../AI_RULES.md`](../AI_RULES.md)** - Single source of truth for:
- Essential documentation to read
- Standard 4-step workflow (source env -> configure identity -> check status -> start work)
- Core rules (git hygiene, AI signatures, clean workspace, issue-first)
- Permissions and roles
- Troubleshooting common issues

---

## Claude Code-Specific Notes

### Quick Command Reference

Common workflows are mapped in [`../CLI_COMMANDS.md`](../CLI_COMMANDS.md):

```bash
/check-status      # Full workspace status
/build             # Build specific package
/build-all         # Build all workspaces
/test-all          # Run all tests
/start-feature     # Create feature branch
/submit-pr         # Create pull request
```

### Claude Code Capabilities

Claude Code has features that work well with this workspace:

- **Extended context window**: Can read full documentation files (AI_RULES.md, ARCHITECTURE.md) without chunking
- **Plan mode**: Natural fit for the "Plan Before Implement" workflow - use it for multi-phase features
- **Parallel tool calls**: Batch independent file reads, searches, and bash commands
- **Task agents**: Use Explore agent for navigating the layered workspace structure
- **CLAUDE.md integration**: Project-level configuration is automatically loaded

### ROS2 Knowledge

For ROS2 development patterns and CLI usage:
- **[`../knowledge/ros2_development_patterns.md`](../knowledge/ros2_development_patterns.md)** - Package structure, colcon, vcstool
- **[`../knowledge/ros2_cli_best_practices.md`](../knowledge/ros2_cli_best_practices.md)** - Runtime ROS commands

---

## First-Time Setup (3 Steps)

If this is your first session in this workspace:

```bash
# 1. Source ROS environment
source .agent/scripts/env.sh

# 2. Configure git identity (ephemeral, session-only)
source .agent/scripts/set_git_identity_env.sh "Claude Code Agent" "roland+claude-code@ccom.unh.edu"

# Or use the shorthand:
source .agent/scripts/set_git_identity_env.sh --agent claude-code

# 3. Check workspace status
.agent/scripts/status_report.sh
```

**Full details**: [`../AI_CLI_QUICKSTART.md`](../AI_CLI_QUICKSTART.md)

---

## Role Assignment

By default, Claude Code acts as **ROS Developer**:
- Create/modify ROS packages
- Write/update tests
- Update package documentation
- Modify `.agent/` infrastructure (only when explicitly assigned Framework Engineer role)

See [`../PERMISSIONS.md`](../PERMISSIONS.md) for full role definitions.

---

## GitHub Integration

Claude Code can use the GitHub CLI (`gh`) for faster operations:

```bash
gh pr list                          # List open PRs
gh issue list --assignee @me        # Your assigned issues
gh pr view 123                      # View PR details
gh issue view 46                    # View issue details
```

---

## AI Signature Format

All GitHub Issues, PRs, and Comments must include:

```markdown
---
**Authored-By**: `Claude Code Agent`
**Model**: `Claude Opus 4.5`
```

Update the model name to match your actual runtime model (Opus, Sonnet, Haiku).

---

**Next Steps**:
- Read [`../AI_CLI_QUICKSTART.md`](../AI_CLI_QUICKSTART.md) for fast onboarding
- Reference [`../AI_RULES.md`](../AI_RULES.md) for complete workflow
- Use [`../CLI_COMMANDS.md`](../CLI_COMMANDS.md) to discover workflows

---

**Last Updated**: 2026-02-10
**Related**: [AI_RULES.md](../AI_RULES.md), [AI_CLI_QUICKSTART.md](../AI_CLI_QUICKSTART.md), [CLI_COMMANDS.md](../CLI_COMMANDS.md)
