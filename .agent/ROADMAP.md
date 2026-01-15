# Workspace Roadmap

## Active Tasks
- [TASK-002] **Consolidate Packages** (Status: In Progress)
    - **Goal**: Refactor `unh_marine_autonomy` and `camp` to consolidate functionality.
    - **Context**: Work is currently dirty in `configs/` and source repositories.

## Backlog

### 🤖 Agent Infrastructure
- [TASK-001] **Multi-Distro Support** (Status: Planned)
    - **Goal**: Support multiple ROS 2 distributions (Jazzy, Kilted, Rolling) simultaneously.
- [TASK-003] **Bootstrap Automation** (Status: Planned)
    - **Goal**: Automate workspace setup (vcstool, colcon, rosdep) to support Codespaces/DevContainers.
    - **Source**: Issues #3, #4.
- [TASK-004] **AI Identity Strategy** (Status: Done)
    - **Goal**: Establish a mechanism for agents to commit/PR as a distinct identity to enable peer review.
    - **Source**: Issue #5.
    - **Resolution**: Implemented Git Authorship Distinction + Reduced Review Requirements.

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

## Completed
- [TASK-000] **Workspace Optimization** (Status: Done)
    - Implemented Resource Locking, Unified Testing, and Knowledge Indexing.
