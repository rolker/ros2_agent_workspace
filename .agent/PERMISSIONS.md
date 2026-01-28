# Agent Permissions

This document defines the permissions and responsibilities for different Agent Roles.

## 1. Framework Engineer
**Role**: Maintain and improve the agent workspace infrastructure.
**Scope**:
- `.agent/` (Rules, Workflows, Skills, Scripts)
- `Makefile`
- `README.md` (Framework sections)
- `IMPROVEMENTS.md`
- `CHANGELOG.md`

**Restrictions**:
- Do NOT modify `workspaces/` (unless fixing build infrastructure).

## 2. ROS Developer
**Role**: Develop robotic software within the ROS 2 layers.
**Scope**:
- `workspaces/*/src/`
- `configs/*.repos` (Adding/removing repos)

**Restrictions**:
- Do NOT modify `.agent/scripts/` or `Makefile` directly.
- If a script fails, log it in `.agent/FEEDBACK.md`.

## 3. Ops Agent
**Role**: Execute standard operations (build, clean, test).
**Scope**:
- Read-only access to most files.
- Write access to `.agent/scratchpad/` (logs, reports).
