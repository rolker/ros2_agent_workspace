# Agent Glossary

**Purpose**: Standardize terminology across all AI agents, humans, and documentation to prevent miscommunication and ambiguity.

**Rule**: Agents MUST consult this glossary when encountering ambiguous terms in instructions, issues, or code.

---

## Core Concepts

### Project
The **target ROS 2 software** being developed - the user's intellectual property and deliverable code.
- **Examples**: `unh_marine_autonomy`, individual ROS 2 packages in `workspaces/*/src/`
- **Location**: Within `workspaces/*/src/` directories
- **Agent Permissions**: ROS Developers can modify when assigned tasks

### Project-Specific Terms
Some terminology is specific to the active project. For project-specific glossaries:
- **Pattern**: Check `workspaces/core_ws/src/<key_repo>/config/GLOSSARY.md` (if exists)
- **Example**: `workspaces/core_ws/src/unh_marine_autonomy/config/GLOSSARY.md`
- **Rule**: Project glossary takes precedence over workspace glossary for conflicting terms

### Workspace
The **scaffolding, tooling, and infrastructure** that wraps and supports Project development.
- **Example**: `ros2_agent_workspace` (this repository)
- **Includes**: `.agent/` scripts, configs, Makefiles, documentation
- **Agent Permissions**: Framework Engineers can modify infrastructure; ROS Developers typically read-only

### Repository vs Package
- **Repository**: A Git repository containing one or more ROS 2 packages (e.g., `marine_autonomy`)
- **Package**: A single ROS 2 package with `package.xml` (e.g., `marine_rviz_plugins`)
- **Distinction**: One repository can contain multiple packages

---

## ROS 2 Workspace Architecture

### Underlay
**Singular**: There is **ONE** underlay - the pre-built dependency foundation.
- **Examples**: `/opt/ros/jazzy`, optionally extended with `underlay_ws/install/`
- **Purpose**: Provides stable, known-good dependencies for all overlays
- **Agent Rule**: READ only, DO NOT MODIFY
- **Sourcing**: `source /opt/ros/jazzy/setup.bash` (or via `env.sh`)

### Overlay
**Plural**: There are **ONE OR MORE** overlays - active development layers built on top of the underlay.
- **Count**: This workspace has 5 overlay layers (core_ws, sensors_ws, platforms_ws, simulation_ws, ui_ws)
- **Purpose**: Contains project code under development
- **Agent Rule**: MODIFY when working on assigned tasks
- **Chaining**: Overlays can stack - each overlay sources the previous one
- **Example Chain**: underlay → core_ws → sensors_ws → platforms_ws → simulation_ws → ui_ws

### Workspace Layer
A thematic grouping of related packages within the multi-workspace structure.
- **Examples**: 
  - `core_ws` - Essential marine autonomy packages
  - `sensors_ws` - Sensor drivers and processing
  - `simulation_ws` - Gazebo/simulation tools
- **Build Order**: Layers build sequentially, each sourcing the previous layer's install space
- **Terminology**: Each layer is an overlay workspace

### Key Principle: Underlay (1) → Overlays (N)
```
/opt/ros/jazzy (underlay)
    ↓
workspaces/core_ws/install (overlay 1)
    ↓
workspaces/sensors_ws/install (overlay 2)
    ↓
workspaces/platforms_ws/install (overlay 3)
    ↓
workspaces/simulation_ws/install (overlay 4)
    ↓
workspaces/ui_ws/install (overlay 5)
```

---

## Agent Workflow Terms

### Issue
A GitHub Issue representing a task, bug, or feature request.
- **Role**: **Source of Truth** for what work exists and who owns it
- **Agent Rule**: Always check issues before starting work; assign yourself or comment "Taking this"

### Work Plan
A markdown file documenting the approach and progress for an issue.
- **Location**: `.agent/work-plans/PLAN_ISSUE-<number>.md`
- **Created by**: `start_issue_work.sh`
- **Purpose**: Enable handover between agents, show progress, document decisions

### Draft PR Workflow
The process of making work-in-progress visible via a draft Pull Request.
1. Create feature branch
2. Create work plan
3. Commit and push plan
4. Open draft PR immediately
5. Update plan as work progresses
6. Mark PR ready when complete

### Planning Mode
A workflow phase where agents analyze requirements and create implementation plans.
- **Trigger**: User message prefixed with `[[PLAN]]`
- **Output**: Structured plan in session workspace or work plan
- **Rule**: Must get user approval before starting implementation

### Execution Mode
A workflow phase where agents implement code and verify changes.
- **Requirements**: Must run `verify_change.sh` and create `walkthrough.md`
- **Documentation**: See `.agent/rules/EXECUTION_MODE.md`

---

## Git & Version Control

### Feature Branch
A branch for developing a specific feature or fix.
- **Naming**: `feature/ISSUE-<number>-<description>` or `feature/<description>`
- **Rule**: Never commit directly to `main`

### Worktree (Planned)
A git feature allowing multiple working directories from the same repository.
- **Purpose**: Enable parallel agent work without conflicts
- **Location**: `.worktrees/task-<number>/` (when implemented)
- **Status**: Not yet implemented (Phase 3)

---

## Agent Roles & Modes

### Framework Engineer
Role with permissions to modify workspace infrastructure.
- **Can modify**: `.agent/` scripts, workflows, build system, CI/CD
- **Cannot modify**: Project code without explicit assignment

### ROS Developer
Default role for agents working on ROS 2 packages.
- **Can modify**: ROS packages in `workspaces/*/src/` (when assigned)
- **Cannot modify**: `.agent/` infrastructure, other agents' tasks

### Sandbox Mode (Planned)
Execution mode where agent runs inside a Docker container.
- **Purpose**: Isolation, safety, clean environment
- **Status**: Not yet implemented (Phase 3)

### Host Mode
Execution mode where agent runs directly on the host system.
- **Current**: Default mode (sandbox not yet implemented)
- **Rule**: All commands should be confirmed by user

---

## Quality & Verification

### Source of Truth
The authoritative reference for information.
- **For Tasks**: GitHub Issues
- **For Code**: `main` branch in git
- **For Status**: `status_report.sh` output

### Verification
The process of validating that changes work correctly.
- **Tool**: `verify_change.sh <package_name> <test_type>`
- **Test Types**: `unit`, `lint`, `all`
- **Rule**: Required before marking PR ready

### Definition of Done
Criteria that must be met before a task is considered complete.
- **Location**: `.agent/rules/definition-of-done.md`
- **Includes**: Tests pass, code linted, documentation updated, PR approved

---

## Configuration & Setup

### .repos File
A YAML file defining git repositories to clone with vcstool.
- **Format**: vcstool repository specification
- **Location**: `configs/repos/*.repos` (in key repositories)
- **Purpose**: Declarative dependency management

### Bootstrap
The initial setup process for a fresh workspace.
- **Script**: `scripts/bootstrap.sh` or `make bootstrap`
- **Actions**: Clone repos (vcstool), install dependencies (rosdep), build workspace

### Colcon
The ROS 2 build tool.
- **Common Commands**: 
  - `colcon build --symlink-install`
  - `colcon test --packages-select <pkg>`
  - `colcon test-result --verbose`

---

## Ambiguity Resolution Examples

**"Update the project documentation"** → Clarify:
- Project code docs (package README)? → Modify `workspaces/*/src/<pkg>/README.md`
- Workspace infrastructure docs? → Modify root-level `README.md` or `.agent/` docs

**"Build the workspace"** → Clarify:
- Build all layers? → `make build`
- Build specific layer? → `cd workspaces/core_ws && colcon build`
- Build specific package? → `colcon build --packages-select <pkg>`

**"Run tests"** → Clarify:
- All workspace tests? → `make test`
- Specific package tests? → `colcon test --packages-select <pkg>`
- Verification script? → `verify_change.sh <pkg> <type>`

---

## Terms to Avoid (Ambiguous)

❌ **"The repository"** → Be specific: Which repository? The workspace repo or a package repo?

❌ **"The workspace"** in isolation → Clarify: The workspace infrastructure or a specific workspace layer (core_ws)?

❌ **"The code"** → Clarify: Agent infrastructure code or project ROS 2 packages?

❌ **"Setup"** → Clarify: Bootstrap a fresh workspace, source the environment, or configure git identity?

---

---

## Project-Specific Glossary (UNH Marine Autonomy)

This section contains terms specific to the current project being developed in this workspace.

### Project11
Legacy name for the UNH Marine Autonomy framework.
- **Refers to**: The packages within `workspaces/core_ws/src/unh_marine_autonomy/`
- **History**: Original codebase name, now rebranded to "Marine Autonomy"
- **Usage**: May appear in legacy code, documentation, or git history
- **Modern term**: Use "Marine Autonomy" or "UNH Marine Autonomy" in new code/docs

### UNH Marine Autonomy
The primary project developed in this workspace.
- **Location**: `workspaces/core_ws/src/unh_marine_autonomy/`
- **Purpose**: Autonomous marine vehicle software framework
- **Components**: Navigation, sensors, visualization, mission planning packages
- **Also known as**: "Project11" (legacy), "marine framework"

> **Note**: For other projects, create a project-specific glossary at:  
> `workspaces/core_ws/src/<project_name>/config/GLOSSARY.md`

---

**Last Updated**: 2026-01-28  
**Related**: [AI_RULES.md](AI_RULES.md), [WORKFORCE_PROTOCOL.md](WORKFORCE_PROTOCOL.md), [PROPOSAL_MULTI_AGENT_WORKFLOW.md](../PROPOSAL_MULTI_AGENT_WORKFLOW.md)
