---
description: Clean up workspace after submitting a PR by returning to default branch
---

# Finish Feature Workflow

Use this workflow **IMMEDIATELY AFTER** submitting a Pull Request. This "unlocks" the repository for future tasks.

## Steps

1.  **Verify PR**
    -   Ensure the PR has been successfully created and pushed.

2.  **Unlock (Return to Default)**
    -   Run the auto-checkout script:
        ```bash
        ./.agent/scripts/checkout_default_branch.sh
        ```
    -   This script detects the default branch (main/jazzy), checks for uncommitted changes, swtiches, and pulls.

3.  **Sync**
    -   (Handled by the script above)
    -   Ensures local default branch is up-to-date.

4.  **Optional Cleanup**
    -   If the feature branch is no longer needed locally (since it's pushed): `git branch -d feature/<task-id>-<description>`
