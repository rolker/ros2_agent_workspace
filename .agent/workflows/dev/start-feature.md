---
description: safely start a new feature branch from a clean, up-to-date default state
---

# Start Feature Workflow

Use this workflow **BEFORE** starting any coding task to ensure you are working on a fresh, up-to-date base.

## Steps

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
