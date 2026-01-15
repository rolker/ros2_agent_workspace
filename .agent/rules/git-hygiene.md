---
trigger: always_on
---

# Git Hygiene & Multi-Agent Coordination

To prevent conflicts and "messy" workspaces when multiple agents are active:

1.  **Branch Isolation**:
    *   **NEVER** commit directly to `main`, `master`, or `jazzy`.
    *   **ALWAYS** create a new branch for your task:
        *   Features: `feature/TASK-<ID>-<description>` (e.g., `feature/TASK-001-multi-distro`)
        *   Fixes: `fix/<description>`
    *   *exception*: If you are just updating documentation or non-code artifacts, you may use your discretion, but a branch is still preferred.

2.  **Clean Handovers**:
    *   **Before you finish** (i.e., before `notify_user` or ending a session):
    *   Run `status_report.sh` or `vcs status`.
    *   **Ensure there are NO uncommitted changes** (staged or unstaged) in any repository, unless you are specifically asking the user to review a "work in progress".
    *   If you have temp changes, `stash` them or `commit` them to your feature branch.

3.  **Pull Request Protocol**:
    *   Do not bundle unrelated changes into a single PR.
    *   If you find "drive-by" fixes (linting, build fixes) in a repo *unrelated* to your main task, create a **separate** branch and PR for them.
