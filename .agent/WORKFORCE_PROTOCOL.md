# Agent Workforce Protocol

**Goal**: Enable multiple AI agents to work accurately and safely within the same workspace without stepping on each other's toes.

## 1. Task Locking (GitHub Issues Authority)
GitHub Issues are the **Source of Truth** for what is being worked on.

*   **Before Starting**:
    *   Search for open issues or projects.
    *   **Do not** pick up an issue assigned to another user/agent unless explicitly instructed to "Join" or "Collaborate".
*   **During Work**:
    *   Assign the issue to yourself (if possible) or comment "Taking this".
*   **Finishing**:
    *   Reference the issue in your PR (e.g., "Closes #123").

## 2. The "Clean Handover" Standard
Every agent invocation is a "shift". When your shift ends (you call `notify_user` or terminate), the workspace must be in a clean state.

*   **The Golden Rule**: *Leave the campsite cleaner than you found it.*
*   **Requirement**:
    *   Run `status_report.sh` or `vcs status` before finishing.
    *   **Alert**: If you see modifications in repositories you didn't touch, **WARN THE USER**. Do not commit them blindly.
    *   **Action**: Commit your own work to a `feature/` branch.

## 3. Git Discipline
*   **Feature Branches**: Always.
*   **Atomic Commits**: One logical change per commit.
*   **Bundled Changes**: Do not mix "Refactoring the entire Navigation Stack" with "Fixing a typo in README".

## 4. Conflict Resolution
If you discover you are modifying a file that has uncommitted changes from another process:
1.  **Stop**.
2.  **Diff** the changes.
3.  **Notify User**: "I see uncommitted changes in `foo.cpp`. Should I include them, revert them, or stash them?"
