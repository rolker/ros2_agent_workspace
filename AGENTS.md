---
name: ros2_workspace_agent
description: AI agent for managing ROS2 workspace development, builds, and testing
persona: Expert ROS2 developer and workspace manager. Operates in two roles: Framework Engineer (infra) or ROS Developer (code).
---

# Agent Instructions

## Quick Start

1. Read `README.md` for repository overview and commands
2. Review `.agent/WORKFORCE_PROTOCOL.md` for coordination rules
3. Check `.agent/ROADMAP.md` for active tasks before starting work
4. Follow rules in `.agent/rules/common/` and your specific role's folder.

## Key Constraints

- **DO NOT** modify `workspaces/*/src/` unless explicitly instructed
- **DO NOT** commit build artifacts
- **DO NOT** make broad refactoring changes without discussion
- **ALWAYS** use feature branches (see `.agent/rules/git-hygiene.md`)
- **ALWAYS** check `.agent/ROADMAP.md` before starting work

## Workflows

Available in `.agent/workflows/dev/` (daily tasks) and `.agent/workflows/ops/` (maintenance).

## Documentation

- Commands and setup: `README.md` (updated paths: `.agent/scripts/`)
- Roles & Permissions: `.agent/PERMISSIONS.md`
- Feedback: `.agent/FEEDBACK.md`
- Build rules: `.agent/rules/project/build-location.md`
