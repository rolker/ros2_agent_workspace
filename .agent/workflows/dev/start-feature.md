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

## Handling Merge Conflicts During Rebase

If your feature branch has fallen behind the default branch, you may need to rebase:

**⚠️ CRITICAL for CLI Agents**: Always use `GIT_EDITOR=true` to avoid getting stuck in interactive editors.

### Safe Rebase Process

1. **Fetch latest changes**:
   ```bash
   git fetch origin
   ```

2. **Rebase with editor disabled**:
   ```bash
   GIT_EDITOR=true git rebase origin/main
   ```

3. **If conflicts occur**:
   ```bash
   # View conflicts
   git status
   
   # Resolve conflicts manually in files
   # (Look for <<<<<<<, =======, >>>>>>> markers)
   
   # Stage resolved files
   git add <resolved-files>
   
   # Continue rebase (IMPORTANT: Use GIT_EDITOR=true)
   GIT_EDITOR=true git rebase --continue
   ```

4. **If you need to abort**:
   ```bash
   git rebase --abort
   ```

### Using Helper Functions

For convenience, source the git helpers:

```bash
source .agent/scripts/lib/git_helpers.sh

# Now use safe functions
safe_git_rebase origin/main

# After resolving conflicts
git add <files>
safe_git_rebase_continue
```

**Why this matters**: Without `GIT_EDITOR=true`, git will launch nano/vim for commit message editing, causing CLI agents to hang. See issue #130 for details.

## See Also

- `.agent/work-plans/README.md` - Work plan workflow details
- `.agent/templates/ISSUE_PLAN.md` - Plan template
- `.agent/WORKFORCE_PROTOCOL.md` - Multi-agent coordination
- `.agent/AI_RULES.md` - Git Operations for CLI Agents section
