# Agent Glossary

**Purpose**: Standardize terminology across all AI agents, humans, and documentation to prevent miscommunication and ambiguity.

**Rule**: Agents MUST consult this glossary when encountering ambiguous terms in instructions, issues, or code.

## Quick Links
[Core Concepts](#core-concepts) | [ROS 2 Architecture](#ros-2-workspace-architecture) | [Workflow Terms](#agent-workflow-terms) | [Git & Roles](#git--version-control) | [Examples](#ambiguity-resolution-examples)

---

## Core Concepts

### Project
The **target ROS 2 software** being developed - the user's intellectual property and deliverable code.
- **Examples**: `unh_marine_autonomy`, individual ROS 2 packages in `layers/*/src/`
- **Location**: Within `layers/*/src/` directories
- **Agent Permissions**: ROS Developers can modify when assigned tasks

### Project-Specific Terms
Some terminology is specific to the active project.

**For Project-Specific Glossaries:**
- **Canonical Location (in project repo)**: `config/GLOSSARY.md` inside the **project/key repository itself** (e.g., in the `unh_marine_autonomy` repo)
- **Workspace Path (after cloning)**: `layers/main/core_ws/src/<key_repo>/config/GLOSSARY.md` (e.g., `layers/main/core_ws/src/unh_marine_autonomy/config/GLOSSARY.md`)
- **Git Note**: `layers/*/src/` is gitignored in this workspace repository; commit the project glossary to the **project/key repository**, not to this workspace repo
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
- **Examples**: `/opt/ros/jazzy`, optionally extended with `layers/main/underlay_ws/install/`
- **Purpose**: Provides stable, known-good dependencies for all overlays
- **Agent Rule**: READ only, DO NOT MODIFY
- **Sourcing**: `source /opt/ros/jazzy/setup.bash` (or via `env.sh`)

### Overlay
**Plural**: There are **ONE OR MORE** overlays - active development layers built on top of the underlay.
- **Count**: This workspace has 5 overlay layers (core_ws, platforms_ws, sensors_ws, simulation_ws, ui_ws)
- **Purpose**: Contains project code under development
- **Agent Rule**: MODIFY when working on assigned tasks
- **Chaining**: Overlays can stack - each overlay sources the previous one
- **Example Chain**: underlay → core_ws → platforms_ws → sensors_ws → simulation_ws → ui_ws

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
layers/main/core_ws/install (overlay 1)
    ↓
layers/main/platforms_ws/install (overlay 2)
    ↓
layers/main/sensors_ws/install (overlay 3)
    ↓
layers/main/simulation_ws/install (overlay 4)
    ↓
layers/main/ui_ws/install (overlay 5)
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
- **Created by**: `.agent/scripts/start_issue_work.sh`
- **Purpose**: Enable handover between agents, show progress, document decisions

### Draft PR Workflow
The process of making work-in-progress visible via a draft Pull Request.
- **Details**: See [WORKFORCE_PROTOCOL.md](WORKFORCE_PROTOCOL.md)
- **Quick Summary**: Create feature branch → work plan → draft PR → implement → mark ready

### Planning Mode / Execution Mode
Agent workflow phases for structured development.
- **Planning**: Analyze requirements, create implementation plan (see `.agent/rules/PLANNING_MODE.md`)
- **Execution**: Implement code, run verification (see `.agent/rules/EXECUTION_MODE.md`)

---

## Git & Version Control

### Feature Branch
A branch for developing a specific feature or fix.
- **Details**: See `.agent/rules/common/git-hygiene.md` and `.agent/rules/common/issue-first.md` (branch naming precedence)
- **Naming**: `feature/ISSUE-<number>-<description>` (Issue-based naming supersedes older `TASK-` patterns)
- **Rule**: Never commit directly to `main`

### Worktree (Planned)
Git feature for multiple working directories from one repository.
- **Status**: Phase 3 (see [PROPOSAL_MULTI_AGENT_WORKFLOW.md](../PROPOSAL_MULTI_AGENT_WORKFLOW.md))

---

## Agent Roles & Permissions

See [PERMISSIONS.md](PERMISSIONS.md) for complete role definitions.

### Framework Engineer
Modifies workspace infrastructure (`.agent/`, workflows, build system).

### ROS Developer
Modifies ROS 2 packages in `layers/*/src/` (when assigned tasks).

---

## Quality & Verification

### Source of Truth
The authoritative reference for information.
- **For Tasks**: GitHub Issues
- **For Code**: Default branch in git (e.g., `main` in the workspace repo; may differ per project repo)
- **For Status**: `status_report.sh` output

### Verification
The process of validating that changes work correctly.
- **Tool**: `./.agent/scripts/verify_change.sh --package <name> --type <unit|lint|all>`
- **Rule**: Required before marking PR ready

### Definition of Done
Criteria for task completion. See `.agent/rules/definition-of-done.md`.

---

## Configuration & Setup

### .repos File
YAML file defining git repositories to clone with vcstool.
- **Location**: `configs/<layer>.repos` or `<key_repo>/config/repos/<layer>.repos`
- **Purpose**: Declarative dependency management

### Bootstrap
Initial environment setup for a fresh workspace.
- **Script**: `./.agent/scripts/bootstrap.sh` or `make bootstrap`
- **Actions**: Install ROS 2/dev tools, configure `rosdep`
- **Follow-up**: Clone repos (`./setup.sh`), build workspaces (`./build.sh`)

### Colcon
ROS 2 build tool. See `.agent/knowledge/ros2_development_patterns.md` for details.
- **Quick Reference**: `colcon build --symlink-install`, `colcon test --packages-select <pkg>`

---

## Ambiguity Resolution Examples

**"Update the project documentation"** → Clarify:
- Project code docs (package README)? → Modify `layers/*/src/<pkg>/README.md`
- Workspace infrastructure docs? → Modify root-level `README.md` or `.agent/` docs

**"Build the workspace"** → Clarify:
- Build all layers? → `make build`
- Build specific layer? → `cd layers/main/core_ws && colcon build`
- Build specific package? → `colcon build --packages-select <pkg>`

**"Run tests"** → Clarify:
- All workspace tests? → `make test`
- Specific package tests? → `colcon test --packages-select <pkg>`
- Verification script? → `./.agent/scripts/verify_change.sh --package <pkg> --type <unit|lint|all>`

---

## Terms to Avoid (Ambiguous)

❌ **"The repository"** → Be specific: Which repository? The workspace repo or a package repo?

❌ **"The workspace"** in isolation → Clarify: The workspace infrastructure or a specific workspace layer (core_ws)?

❌ **"The code"** → Clarify: Agent infrastructure code or project ROS 2 packages?

❌ **"Setup"** → Clarify: Bootstrap a fresh workspace, source the environment, or configure git identity?

---

## Project-Specific Glossary (UNH Marine Autonomy)

This section contains terms specific to the current project being developed in this workspace.

### Project11
Legacy name for the UNH Marine Autonomy framework.
- **Refers to**: The packages within `layers/main/core_ws/src/unh_marine_autonomy/`
- **History**: Original codebase name, now rebranded to "Marine Autonomy"
- **Usage**: May appear in legacy code, documentation, or git history
- **Modern term**: Use "Marine Autonomy" or "UNH Marine Autonomy" in new code/docs

### UNH Marine Autonomy
The primary project developed in this workspace.
- **Location**: `layers/main/core_ws/src/unh_marine_autonomy/`
- **Purpose**: Autonomous marine vehicle software framework
- **Components**: Navigation, sensors, visualization, mission planning packages
- **Also known as**: "Project11" (legacy), "marine framework"

> **Note**: For other projects, create a project-specific glossary at:  
> `layers/main/core_ws/src/<project_name>/config/GLOSSARY.md`

---

**Last Updated**: 2026-01-28  
**Related**: [AI_RULES.md](AI_RULES.md), [WORKFORCE_PROTOCOL.md](WORKFORCE_PROTOCOL.md), [PROPOSAL_MULTI_AGENT_WORKFLOW.md](../PROPOSAL_MULTI_AGENT_WORKFLOW.md)
