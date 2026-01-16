---
description: Comprehensive workspace status check (Local + GitHub)
---

# Workspace Status Check

1. **Local Git Status**
   - Run the `/check-local-status` workflow.

2. **Remote GitHub Status**
   - Run the `/check-github-status` workflow.

3. **Morning Report**
   - Combine the findings into a summary "Morning Report".
   - **Format**:
     - **Date**: Today's Date
     - **Workspace State**: (Clean/Dirty)
     - **Action Items**:
       - Unmerged PRs waiting for review.
       - New Issues needing triage.
       - Local changes needing commit.
