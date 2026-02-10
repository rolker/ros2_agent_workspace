# Claude Code Configuration

This workspace is configured for AI agent workflows. Claude Code should follow the universal agent rules.

## Quick Start

```bash
# 1. Source ROS2 environment
source .agent/scripts/env.sh

# 2. Configure git identity (session-only)
source .agent/scripts/set_git_identity_env.sh "Claude Code Agent" "roland+claude-code@ccom.unh.edu"

# 3. Check workspace status
.agent/scripts/status_report.sh
```

## Key Documentation

Read these before starting work:

- **[`.agent/AI_RULES.md`](.agent/AI_RULES.md)** - Universal agent workflow and rules
- **[`.agent/AI_IDENTITY_STRATEGY.md`](.agent/AI_IDENTITY_STRATEGY.md)** - Git identity configuration
- **[`.agent/CLI_COMMANDS.md`](.agent/CLI_COMMANDS.md)** - Available workflow commands
- **[`ARCHITECTURE.md`](ARCHITECTURE.md)** - System design and layering

## Build & Test Commands

```bash
# Build all layers
make build

# Build specific package
cd layers/main/core_ws && colcon build --packages-select <package>

# Run tests
make test

# Validate workspace
make validate
```

## Git Workflow

1. **Never commit to `main`** - Always use feature branches
2. **Use worktrees for isolation**:
   ```bash
   .agent/scripts/worktree_create.sh --issue <N> --type workspace
   source .agent/scripts/worktree_enter.sh --issue <N>
   ```
3. **AI signature required** in all PRs:
   ```markdown
   ---
   **Authored-By**: `Claude Code Agent`
   **Model**: `Claude Opus 4.5`
   ```

## Layered Architecture

The workspace uses a layered build system. All ROS packages are in `layers/`:

```
layers/
├── main/
│   ├── underlay_ws/   # Additional dependencies
│   ├── core_ws/       # UNH Marine Autonomy Framework
│   ├── platforms_ws/  # Platform-specific code
│   ├── sensors_ws/    # Sensor drivers
│   ├── simulation_ws/ # Simulation tools
│   └── ui_ws/         # Visualization
```

## Claude Code Strengths

Leverage these capabilities:
- **Extended context**: Read full documentation files without chunking
- **Plan mode**: Use for multi-phase feature implementation
- **Parallel tool calls**: Batch independent operations
- **Task agents**: Explore the layered workspace structure

## Common Workflows

| Command | Description |
|---------|-------------|
| `/check-status` | Full workspace status report |
| `/build-all` | Build all workspace layers |
| `/test-all` | Run all tests |
| `/start-feature` | Create feature branch for issue |
| `/submit-pr` | Create pull request |

See [`.agent/CLI_COMMANDS.md`](.agent/CLI_COMMANDS.md) for complete reference.
