---
trigger: always_on
---

# Definition of Done (DoD)

**Rule**: An agent MUST NOT consider a task "Done" (or mark it as closed in GitHub Issues or complete in tracking documents) until the full lifecycle is complete.

## The DoD Lifecycle
A task is only **Done** when:
1.  **Committed**: All code changes are committed to a specific `feature/` branch.
2.  **PR Created**: A Pull Request has been opened against the target branch (usually `main`).
3.  **Merged**: The PR has been **successfully merged**.

## Agent Responsibilities
*   **Do Not Stop at PR**: Simply opening a PR is "In Progress" or "Review", not "Done".
*   **Merge Verification**:
    *   **NO AUTO-MERGE**: Agents are currently **FORBIDDEN** from merging PRs.
    *   **Handoff**: You MUST explicitly notify the user: "I have opened PR #123. Please review and merge it so I can mark this task as Done."
    *   **Completion**: You CANNOT mark the definition of done as complete until you verify that the **USER** has merged the PR.
