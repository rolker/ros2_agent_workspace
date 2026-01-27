# Agent Workforce Protocol

**Goal**: Enable multiple AI agents to work accurately and safely within the same workspace without stepping on each other's toes.

## 1. Work Visibility (Draft PR Workflow)

To prevent duplicate work and enable collaboration, agents must make work-in-progress visible:

*   **Before Starting Work on an Issue**:
    *   Check for existing draft PRs (indicates another agent is working)
    *   Use helper script: `.agent/scripts/start_issue_work.sh <issue_number> "Agent Name"`
    *   This creates:
        - Feature branch `feature/ISSUE-<number>-<description>`
        - Work plan in `.agent/work-plans/PLAN_ISSUE-<number>.md`
    *   Edit the plan to document your approach
    *   Commit and push the plan
    *   Create a **draft PR** immediately with the plan

*   **During Work**:
    *   Update the work plan as you progress (check off tasks, document decisions)
    *   Commit plan updates: `.agent/scripts/update_issue_plan.sh <issue_number>`
    *   The plan serves as handover documentation if another agent needs to take over

*   **Finishing**:
    *   Update plan status to "Ready for Review"
    *   Mark draft PR as ready: `gh pr ready`

**Benefits**:
-   Draft PRs show active work on GitHub (visible to all agents/users)
-   Branch naming prevents collisions (git rejects duplicate branches)
-   Plans enable seamless handover between agents
-   Approach is reviewable before code implementation

## 2. Task Locking (GitHub Issues Authority)
GitHub Issues are the **Source of Truth** for what is being worked on.

*   **Before Starting**:
    *   Search for open issues or projects.
    *   **Do not** pick up an issue assigned to another user/agent unless explicitly instructed to "Join" or "Collaborate".
*   **During Work**:
    *   Assign the issue to yourself (if possible) or comment "Taking this".
*   **Finishing**:
    *   Reference the issue in your PR (e.g., "Closes #123").

## 2. Task Locking (GitHub Issues Authority)
GitHub Issues are the **Source of Truth** for what is being worked on.

*   **Before Starting**:
    *   Search for open issues or projects.
    *   **Check for draft PRs** - they indicate active work
    *   **Do not** pick up an issue assigned to another user/agent unless explicitly instructed to "Join" or "Collaborate".
*   **During Work**:
    *   Assign the issue to yourself (if possible) or comment "Taking this".
    *   Create draft PR with work plan (see "Work Visibility" above).
*   **Finishing**:
    *   Reference the issue in your PR (e.g., "Closes #123").
    *   Mark draft PR as ready for review.

## 3. The "Clean Handover" Standard
Every agent invocation is a "shift". When your shift ends (you call `notify_user` or terminate), the workspace must be in a clean state.

*   **The Golden Rule**: *Leave the campsite cleaner than you found it.*
*   **Requirement**:
    *   Run `status_report.sh` or `vcs status` before finishing.
    *   **Alert**: If you see modifications in repositories you didn't touch, **WARN THE USER**. Do not commit them blindly.
    *   **Action**: Commit your own work to a `feature/` branch.

## 4. Git Discipline
*   **Feature Branches**: Always.
*   **Atomic Commits**: One logical change per commit.
*   **Bundled Changes**: Do not mix "Refactoring the entire Navigation Stack" with "Fixing a typo in README".

## 5. Conflict Resolution
If you discover you are modifying a file that has uncommitted changes from another process:
1.  **Stop**.
2.  **Diff** the changes.
3.  **Notify User**: "I see uncommitted changes in `foo.cpp`. Should I include them, revert them, or stash them?"
