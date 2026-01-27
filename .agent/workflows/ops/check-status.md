---
description: Comprehensive workspace status check (Local + GitHub)
---

# Workspace Status Check

## Quick Start (Unified Script)

Run the unified status check script that combines local and remote checks:

```bash
python3 .agent/scripts/check_full_status.py
```

This single command performs:
1. **Local Git Status** - Checks all workspace repositories for modifications
2. **Remote GitHub Status** - Fetches open PRs and Issues

**Options:**
- `--no-local` - Skip local git status check
- `--no-remote` - Skip remote GitHub status check  
- `--batch-size N` - Number of repositories per GitHub query (default: 10)

**Examples:**
```bash
# Full status check (local + remote)
python3 .agent/scripts/check_full_status.py

# Only local status
python3 .agent/scripts/check_full_status.py --no-remote

# Only remote status
python3 .agent/scripts/check_full_status.py --no-local

# Custom batch size for GitHub queries
python3 .agent/scripts/check_full_status.py --batch-size 5
```

## Legacy Workflow (Manual Steps)

The following steps are still available if you need to run checks separately:

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

## Output Format

The unified script generates a consolidated Markdown report with:

### Local Status Section
- Root repository status (clean/modified, branch)
- Workspace repositories summary (total, clean, attention needed)
- Modified repositories table with status indicators:
  - ⚠️ Modified - Local changes
  - 🚀 Ahead - Commits ahead of remote
  - ⬇️ Behind - Commits behind remote
  - 🔀 Branch mismatch - Not on expected branch
  - ❓ Untracked - Not in .repos files
- Latest test status (if available)

### Remote Status Section
- Open Pull Requests table (repository, title, author, PR #)
- Open Issues table (repository, title, labels, issue #)

## Benefits of Unified Script

| Metric | Legacy | Unified |
|--------|--------|---------|
| **User Approvals** | 3-5 | **1** |
| **Agent Turns** | ~10 | **2** |
| **Commands** | 5 separate | **1** |
| **Reliability** | Manual coordination | **High** |

## Technical Details

### Repository Discovery
The script uses `.agent/scripts/lib/workspace.py` to:
- Scan all `.repos` files in `configs/` directory
- Extract GitHub owner/repo information from URLs
- Always include the root repository `rolker/ros2_agent_workspace`

### GitHub Query Batching
- Queries are batched to avoid command-line length limits
- Default batch size: 10 repositories per query
- Uses server-side filtering (`repo:owner/name` syntax) for efficiency
- Avoids fetching unrelated repositories

### Authentication
Requires GitHub CLI authentication:
```bash
gh auth login
```
