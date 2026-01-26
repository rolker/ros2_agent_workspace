# Agent Rule: EXECUTION MODE

**Trigger**: After `implementation_plan.md` is approved.
**Goal**: Implement changes, verify them, and prepare for merge.

## 1. Implementation Loop
1.  **Select Task**: Pick the next item from `task.md`.
2.  **Update Status**: Call `task_boundary` with the current activity.
3.  **Edit/Create**: Use `write_to_file` / `replace_file_content`.
    *   *Tip*: Keep edits atomic and focused.

## 2. Verification (The "Adaptive QA" Strategy)
*   **Unit/Lint Tests**: Run `./.agent/scripts/verify_change.sh --package <pkg> --type <unit|lint>` as you work.
    *   *Rule*: You cannot mark a task "Done" if verification fails.
*   **Integration**: Run `./.agent/scripts/test.sh` to ensure no regression in other layers.
*   **Manual**: If UI/Sim changes, capture a screenshot/video if possible.

## 3. Documentation
*   **Update Artifacts**: Keep `task.md` up to date (`[x]`).
*   **Create `walkthrough.md`**:
    *   Summarize changes.
    *   Show proof of verification (copy-paste terminal output of `verify_change.sh`).

## 4. Completion (Definition of Done)
*   **Clean Up**: Remove temp files.
*   **Commit**: `git commit` to your feature branch.
*   **PR**: Open a PR using `mcp_github`.
*   **Hand off**: Notify the user that the work is ready for review.

## 5. Anti-Patterns
*   ❌ Committing broken code (always verify first!).
*   ❌ Changing the plan silently (if the plan changes, go back to Planning Mode).
*   ❌ Leaving `task.md` stale.
