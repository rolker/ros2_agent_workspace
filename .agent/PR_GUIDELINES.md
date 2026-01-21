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
    *   Once checks pass and reviews are approved, notify the user to merge (per `definition-of-done.md`).

## Repository Locking & Branch Hygiene

To ensure workspace stability and prevent lost work, we enforce a **"Branch as Lock"** policy:

### 1. The Lock Mechanism
*   **Unlocked**: A repository is "Unlocked" ONLY when it is on its default branch (`main` or `jazzy`) AND is clean (no uncommitted changes).
*   **Locked**: If a repository is on any other branch or has uncommitted changes, it is considered **LOCKED** (Active Work in Progress).

### 2. Workflow Rules
*   **Do Not Disturb**: Agents MUST NOT switch branches or modify files in a locked repository unless explicitly instructed to continue the work associated with that specific branch.
*   **Start Fresh**: Always run the `start-feature` workflow before beginning new work. This ensures you start from a clean, up-to-date default branch.
*   **Clean Up**: Always run the `finish-feature` workflow immediately after submitting a PR. This returns the repository to the "Unlocked" state, ready for the next task.
