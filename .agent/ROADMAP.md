# Workspace Roadmap [DEPRECATED]

**⚠️ DEPRECATION NOTICE**: This file is no longer the Source of Truth. Please refer to **GitHub Issues** for active tasks and planning.

## Historical Data (For Reference Only)

### 🔥 High Priority
- [TASK-010] **CAMP Architecture Refactor** (Status: Planned)
    - **Goal**: Refactor `camp` to separate Core (Headless) from Desktop (Qt Widgets). 
    - **Context**: Split from TASK-002.

### 🤖 Agent Infrastructure
- [TASK-011] **Standardize Issues as Source of Truth** (Status: In Progress)
    - **Goal**: Enforce a rule that all agent tasks must be tracked by a GitHub Issue.
    - **Source**: Issue #45.
    - **Resolution**: Update Onboarding, Planning Mode, and create new rule file.
- [TASK-001] **Multi-Distro Support** (Status: Planned)
    - **Goal**: Support multiple ROS 2 distributions (Jazzy, Kilted, Rolling) simultaneously.
- [TASK-003] **Bootstrap Automation** (Status: Done)
    - **Goal**: Automate workspace setup (vcstool, colcon, rosdep) to support Codespaces/DevContainers.
    - **Source**: Issues #3, #4.
    - **Resolution**: Created `scripts/bootstrap.sh` matching official ROS 2 docs. Verified via Docker.
- [TASK-004] **AI Identity Strategy** (Status: Done)
    - **Goal**: Establish a mechanism for agents to commit/PR as a distinct identity to enable peer review.
    - **Source**: Issue #5.
    - **Resolution**: Implemented Git Authorship Distinction + Reduced Review Requirements.
- [TASK-008] **Framework Vision Refactor** (Status: In Progress)
    - **Goal**: Rename and modularize `project11` ecosystem to `marine` framework.
    - **Source**: Existing uncommitted work.
- [TASK-020] **Workspace Maintenance** (Status: In Progress)
    - **Goal**: Clean up minor build issues and standardize branches.
    - **Source**: Workspace Status Check.
- [TASK-019] **Enhanced Status Reporting** (Status: Done)
    - **Goal**: Enable agents to check both local workspace status and remote GitHub status (PRs/Issues) without external dependencies.
    - **Source**: User Request.

### 🚤 Project Work
- [TASK-005] **Sensor Frames Support** (Status: Planned)
    - **Goal**: Add support for sensors offset from base_link.
    - **Context**: Feature likely implemented but requires a test strategy.
    - **Source**: `mru_transform` Issue #1.
- [TASK-006] **Vertical Reference Points** (Status: Planned)
    - **Goal**: Standardize handling of tides and heave in vertical reference frames.
    - **Source**: `mru_transform` Issue #2.
- [TASK-007] **Color Map Improvements** (Status: Planned)
    - **Goal**: Support selectable color maps and dynamic range adjustment at render time.
    - **Source**: `rviz_sonar_image` Issue #2.

### Unsorted
<!-- New tasks go here -->
- [TASK-009] **Evaluate ros2_mcp** (Status: Planned)
    - **Goal**: Consider `ros2_mcp` to help with automated testing and debugging.


## Completed
- [TASK-002] **Rebranding & Consolidation** (Status: Done)
    - **Outcome**: Successfully rebranded `project11` -> `marine_autonomy`. Repositories consolidated and upstream URLs updated.
- [TASK-000] **Workspace Optimization** (Status: Done)
    - Implemented Resource Locking, Unified Testing, and Knowledge Indexing.
