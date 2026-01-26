# Agent Rule: PLANNING MODE

**Trigger**: When starting a new task, major refactor, or complex bug fix.
**Goal**: Produce a solid plan (`implementation_plan.md`) and get user approval.

## 1. Analysis Phase
*   **Read Context**: Use `view_file` on relevant docs (`README.md`, `ARCHITECTURE.md`) and source code.
*   **Check Existing Issues**: Use `github-mcp-server` to search for related issues/PRs.
*   **Understand Requirements**: If ambiguous, ask clarifying questions using `notify_user`.

## 2. Artifact Creation
*   **Create/Update `task.md`**: Break down the high-level goal into checklist items.
*   **Create `implementation_plan.md`**:
    *   **Goal Description**: What are we doing?
    *   **User Review Required**: Breaking changes? Risks?
    *   **Proposed Changes**: List files to modify/create.
    *   **Verification Plan**: How will we test this? (Mention `verify_change.sh`).

## 3. The "Planning Handshake"
*   **Review**: You MUST use `notify_user` to prompt the user to review `implementation_plan.md`.
*   **Iterate**: If the user gives feedback, update the plan and re-request review.
*   **Proceed**: DO NOT switch to Execution Mode until the user (or a Senior Agent) says "Approved" or "Go ahead".

## 4. Anti-Patterns
*   ❌ Writing code (except for exploration/prototyping) before the plan is approved.
*   ❌ Skipping the Verification Plan section.
*   ❌ Assuming requirements without checking.
