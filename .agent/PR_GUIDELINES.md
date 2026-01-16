# PR and Review Guidelines

## Core Philosophy: "One Task, One PR"
*   **Granularity**: Align Pull Requests with distinct tasks or logical units of work.
*   **Size**: Aim for PRs reviewable in < 15 minutes.
*   **Focus**: Separate refactoring from feature work.

## Multi-Agent Concurrency Strategy
To prevent conflicts when multiple agents/users work in the same repo:
1.  **Sync First**: Always pull the latest `main` before branching.
2.  **Unique Branch Names**: MUST include a unique identifier (e.g., Task ID).
    *   Format: `feature/<task-id>-<description>`
    *   Example: `feature/TASK-123-update-nav-config`

## Automated Review Workflow
Every PR must go through an automated review cycle:
1.  **Submit PR**: Use the `submit-pr` workflow.
2.  **Request Copilot Review**: If the repo is configured, comment `/copilot review` on the PR (if supported) or assign the relevant reviewers.
3.  **Monitor & Respond**:
    *   **Poll**: Periodically check the PR status (e.g., every few minutes or after notification).
    *   **Review Comments**: Read and analyze all reviewer comments.
    *   **Address Feedback**:
        *   **Code Changes**: Commit fixes for valid issues.
        *   **Discussion**: Reply to comments if clarification is needed or if you disagree (politely).
    *   **Dismiss**: False positives can be dismissed with reasoning, but human reviewer comments MUST be addressed.
4.  **Merge Handoff**:
    *   Once checks pass and reviews are approved, notify the user to merge (per `definition-of-done.md`).
