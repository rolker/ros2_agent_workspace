---
name: ros2_workspace_agent
description: AI agent for managing ROS2 workspace development, builds, and testing
persona: Expert ROS2 developer and workspace manager for the UNH Marine Autonomy Framework
---

# Agent Instructions

## Quick Start

1. Read `README.md` for repository overview and commands
2. Review `.agent/WORKFORCE_PROTOCOL.md` for coordination rules
3. Check `.agent/ROADMAP.md` for active tasks before starting work
4. Follow rules in `.agent/rules/` for builds, git workflow, and file organization

## Key Constraints

- **DO NOT** modify `workspaces/*/src/` unless explicitly instructed
- **DO NOT** commit build artifacts
- **DO NOT** make broad refactoring changes without discussion
- **ALWAYS** use feature branches (see `.agent/rules/git-hygiene.md`)
- **ALWAYS** check `.agent/ROADMAP.md` before starting work

## Workflows

Available in `.agent/workflows/`: `/add-repo`, `/build-all`, `/clean`, `/submit-pr`, etc.

## Documentation

- Commands and setup: `README.md`
- ROS2 CLI patterns: `.agent/knowledge/ros2_cli_best_practices.md`
- Git identity: `.agent/AI_IDENTITY_STRATEGY.md`
- Build rules: `.agent/rules/build-location.md`, `.agent/rules/clean-root.md`
