---
description: Create a feature branch, push changes, and open a Pull Request.
---

# Submit PR Workflow

Use this workflow when you are ready to submit your changes for review.

## For Work Started with a Plan (Draft PR)

If you started with `.agent/scripts/start_issue_work.sh` and created a draft PR:

1.  **Verify Plan is Updated**
    -   Ensure `.agent/work-plans/PLAN_ISSUE-<number>.md` reflects final state
    -   Check off completed tasks
    -   Update status to "✅ Complete" or "🚀 Ready for Review"
    
2.  **Commit Final Plan Update** (if needed)
    ```bash
    .agent/scripts/update_issue_plan.sh <issue_number> "Final plan update"
    git push
    ```

2.  **Check for Updates** (Recommended)
    -   Ensure your branch is up-to-date with the default branch:
        ```bash
        .agent/scripts/check_branch_updates.sh
        ```
    -   If updates are needed, merge or rebase as recommended
    -   Re-run tests after updating

3.  **Validate Your Changes**
    -   Run local quality checks: `make lint` or `pre-commit run --all-files`
    -   **Requirement**: You MUST fix any errors before proceeding.

4.  **Mark PR as Ready for Review**
    ```bash
    gh pr ready
    ```
    This converts the draft PR to ready-for-review status.

5.  **Request Review** (if needed)
    -   Comment `/copilot review` on the PR (if supported)
    -   Or assign reviewers manually

6.  **Return to Default Branch** (Unlock)
    ```bash
    ./.agent/scripts/checkout_default_branch.sh
    ```

## For New Work (No Draft PR Yet)

1.  **Branching**
    -   Ensure you are on the latest `main`.
    -   Create a new branch: `git checkout -b feature/<issue-id>-<short-description>`
    -   *Example*: `git checkout -b feature/ISSUE-001-multi-distro-support`

2.  **Check for Updates** (Recommended)
    -   Before committing, ensure your branch is current:
        ```bash
        .agent/scripts/check_branch_updates.sh
        ```
    -   This runs automatically with pre-commit hooks

3.  **Validate**
    -   Run local quality checks: `make lint` or `pre-commit run --all-files`
    -   **Requirement**: You MUST fix any errors before proceeding.

3.  **Commit**
    -   Stage files: `git add <files>`
    -   Commit with a conventional message: `git commit -m "feat: <description>"`

4.  **Push**
    -   Push to origin: `git push -u origin HEAD`

5.  **Create Pull Request**
    -   **MANDATORY**: Append the AI Signature (see `.agent/rules/common/ai-signature.md`) to the PR body for all methods.
    -   **Option A: GitHub MCP (Preferred)**
        -   Use `github.create_pull_request`.
        -   Title: `feat: <description>`
        -   Body: Description of changes + "Closes #<issue-id>" (Mandatory if related to issue) + AI Signature.
    -   **Option B: `gh` CLI**
        -   Run: `gh pr create --title "feat: <description>" --body "Closes #<issue-id> description of changes..."`
        -   Ensure AI Signature is included in body.
    -   **Option C: Manual**
        -   The `git push` command usually outputs a URL to create a PR.
        -   **Action**: Display that URL to the user.
        -   **Requirement**: Remind user to add "Closes #<issue-id>" and the AI Signature to the description.

6.  **Request Review**
    -   If the repo is configured, comment `/copilot review` on the PR (if supported) or assign the relevant reviewers.

7.  **Finish & Unlock**
    -   Run the `finish-feature` workflow, OR:
    -   Automatically return to the default branch to prevent working on a stale feature branch:
        ```bash
        ./.agent/scripts/checkout_default_branch.sh
        ```

## See Also

- `.agent/work-plans/README.md` - Work visibility workflow
- `.agent/workflows/dev/start-feature.md` - Starting work on issues
- `.agent/rules/common/ai-signature.md` - AI signature requirement
