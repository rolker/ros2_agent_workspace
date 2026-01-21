---
description: Create a feature branch, push changes, and open a Pull Request.
---

# Submit PR Workflow

Use this workflow when you are ready to submit your changes for review.

## Steps

1.  **Branching**
    -   Ensure you are on the latest `main`.
    -   Create a new branch: `git checkout -b feature/<task-id>-<short-description>`
    -   *Example*: `git checkout -b feature/TASK-001-multi-distro-support`

2.  **Validate**
    -   Run local quality checks: `make lint` or `pre-commit run --all-files`
    -   **Requirement**: You MUST fix any errors before proceeding.

3.  **Commit**
    -   Stage files: `git add <files>`
    -   Commit with a conventional message: `git commit -m "feat: <description>"`

4.  **Push**
    -   Push to origin: `git push -u origin HEAD`

5.  **Create Pull Request**
    -   **Option A: GitHub MCP (Preferred)**
        -   Use `github.create_pull_request`.
        -   Title: `feat: <description>`
        -   Body: Description of changes + "Closes #<issue-id>" if applicable.
    -   **Option B: `gh` CLI**
        -   Run: `gh pr create --fill`
    -   **Option C: Manual**
        -   The `git push` command usually outputs a URL to create a PR.
        -   **Action**: Display that URL to the user.

6.  **Request Review**
    -   If the repo is configured, comment `/copilot review` on the PR (if supported) or assign the relevant reviewers.

7.  **Finish & Unlock**
    -   Run the `finish-feature` workflow to return the repository to a clean state.
    -   Step: `git checkout <default-branch>` && `git pull`
