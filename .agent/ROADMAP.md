# Workspace Roadmap

## Active Tasks
<!-- No active tasks currently -->

## Backlog

### High Priority
- [TASK-001] **Multi-Distro Support** (Status: Planned)
    - **Goal**: Support multiple ROS 2 distributions (Jazzy, Kilted, Rolling) simultaneously.
    - **Architecture**: Nested Directory Structure (`workspaces/<distro>/<layer>_ws`).
    - **Required Changes**:
        1. Migration: Move existing workspaces to `workspaces/jazzy/`.
        2. Scripts: Update `env.sh`, `setup.sh`, `build.sh` to accept distro args.

### Unsorted
<!-- New tasks go here -->

## Completed
- [TASK-000] **Workspace Optimization** (Status: Done)
    - Implemented Resource Locking, Unified Testing, and Knowledge Indexing.
