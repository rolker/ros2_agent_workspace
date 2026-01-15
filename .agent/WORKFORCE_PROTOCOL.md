# Agent Workforce Protocol

**Goal**: Enable multiple AI agents to work accurately and safely within the same workspace without stepping on each other's toes.

## 1. Task Locking (The `ROADMAP.md` Authority)
The `.agent/ROADMAP.md` is the **Source of Truth** for what is being worked on.

*   **Before Starting**:
    *   Check `.agent/ROADMAP.md` for items marked `(Status: Active)` or `(Status: In Progress)`.
    *   **Do not** pick up a task effectively "owned" by another active agent session unless explicitly instructed to "Join" or "Collaborate".
*   **During Work**:
    *   Update your specific item to `(Status: Active)`.
*   **Finishing**:
    *   Update item to `(Status: Done)`.

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
