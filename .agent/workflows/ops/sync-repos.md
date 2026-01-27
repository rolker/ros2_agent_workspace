---
description: Safe synchronization of all workspace repositories
---

# Sync Workspace Repositories

This workflow safely manages updates for all repositories in the workspace.

## Logic
1.  **Dirty Check**: Repositories with uncommitted changes are **SKIPPED** to prevent creating conflict markers in your work-in-progress files.
2.  **Branch Awareness**:
    *   `main` / `jazzy`: Automatically performs `git pull --rebase`.
    *   Feature Branches: Performs `git fetch` only. It will notify you if you are behind origin, but will **NOT** merge automatically.

## Usage
To dry-run (see what would happen):
```bash
python3 .agent/scripts/sync_repos.py --dry-run
```

To execute:
```bash
// turbo
python3 .agent/scripts/sync_repos.py
```
