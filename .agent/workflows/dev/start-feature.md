---
description: Safely start a new feature branch from a clean, up-to-date default state
---

# Start Feature Workflow

Use this workflow **BEFORE** starting any coding task to ensure you are working on a fresh, up-to-date base.

## For GitHub Issues (Recommended)

When working on a GitHub issue, use the **work visibility workflow** to make your work trackable:

1.  **Use Helper Script** (Easiest)
    ```bash
    .agent/scripts/start_issue_work.sh <issue_number> "Your Agent Name"
    ```
    This automatically:
    - Ensures clean workspace
    - Pulls latest changes
    - Creates feature branch `feature/ISSUE-<number>-<description>`
    - Generates work plan from template
    
2.  **Edit the Work Plan**
    - Open `.agent/work-plans/PLAN_ISSUE-<number>.md`
    - Fill in your approach, tasks, and design decisions
    - This plan makes your work visible on GitHub!

3.  **Commit and Push Plan**
    ```bash
    git add .agent/work-plans/
    git commit -m "docs: Add work plan for Issue #<number>"
    git push -u origin HEAD
    ```

4.  **Create Draft PR**
    ```bash
    gh pr create --draft \
      --title "feat: <description>" \
      --body $'See .agent/work-plans/PLAN_ISSUE-<number>.md\n\nCloses #<number>\n\n---\n**🤖 Authored-By**: `Your Agent Name`'
    ```

**Benefits**: 
- Makes work-in-progress visible on GitHub
- Prevents other agents from duplicating work
- Provides context for handover if needed
- Creates reviewable approach before code changes

## For General Features (No Issue)

1.  **Check Status**
    -   Verify the repository is clean: `git status --porcelain`
    -   **Stop** if there are uncommitted changes.

2.  **Unlock (Return to Default)**
    -   Determine default branch (usually `main` for root, `jazzy` for project repos).
    -   `git checkout <default-branch>`

3.  **Update**
    -   `git pull`
    -   Ensure you have the latest changes from the remote.

4.  **Branch**
    -   Create your feature branch: `git checkout -b feature/<task-id>-<description>`

## See Also

- `.agent/work-plans/README.md` - Work plan workflow details
- `.agent/templates/ISSUE_PLAN.md` - Plan template
- `.agent/WORKFORCE_PROTOCOL.md` - Multi-agent coordination
