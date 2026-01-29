---
trigger: always_on
---

# Git Hygiene & Multi-Agent Coordination

To prevent conflicts and "messy" workspaces when multiple agents are active:

1.  **Branch Isolation**:
    *   **NEVER** commit directly to `main`, `master`, or `jazzy`.
    *   **Repository Protection**: The `main` branch is protected by repository rules - direct pushes will be rejected.
    *   **ALWAYS** create a new branch for your task:
        *   Features: `feature/ISSUE-<ID>-<description>` (e.g., `feature/ISSUE-001-multi-distro`)
        *   Fixes: `fix/<description>`
    *   **All changes via Pull Requests**: After pushing your feature branch, create a PR to merge into `main`.
    *   *exception*: If you are just updating documentation or non-code artifacts, you may use your discretion, but a branch + PR is still preferred.

2.  **Git Worktrees for Parallel Work** (Recommended):
    For complete isolation when multiple agents work simultaneously:
    ```bash
    # Create isolated worktree
    .agent/scripts/start_issue_work.sh 42 "Agent Name" --worktree layer
    
    # Enter worktree (sources ROS environment)
    source .agent/scripts/worktree_enter.sh 42
    
    # List all active worktrees
    .agent/scripts/worktree_list.sh
    ```
    Worktrees provide separate directories so agents don't interfere with each other's builds or uncommitted changes.

## Development Workflows

### 1. Sandboxed (Recommended)
We provide a `.devcontainer` configuration that sets up a full ROS 2 Jazzy environment with all dependencies.
- **Prerequisites**: Docker Desktop (or equivalent) + VS Code + Dev Containers Extension.
- **Benefits**: No need to install ROS 2 on your host machine. Safe for AI agents.
- **Usage**: Click "Reopen in Container" in VS Code.

### 2. Native (Advanced)
If you prefer developing natively on Ubuntu 24.04:
- Run `./.agent/scripts/bootstrap.sh` to install dependencies.
- Source the environment: `source .agent/scripts/env.sh`.

## Git Hygiene
    *   **Before you finish** (i.e., before `notify_user` or ending a session):
    *   Run `status_report.sh` or `vcs status`.
    *   **Ensure there are NO uncommitted changes** (staged or unstaged) in any repository, unless you are specifically asking the user to review a "work in progress".
    *   If you have temp changes, `stash` them or `commit` them to your feature branch.

3.  **Pull Request Protocol**:
    *   Do not bundle unrelated changes into a single PR.
    *   If you find "drive-by" fixes (linting, build fixes) in a repo *unrelated* to your main task, create a **separate** branch and PR for them.
