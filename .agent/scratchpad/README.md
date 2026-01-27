# Agent Scratchpad

This directory is for **persistent temporary artifacts** created during agent sessions.

## Purpose
Use this location for:
- Analysis artifacts (CSVs, JSON reports, markdown documents)
- Build logs and diagnostics
- Test results and coverage reports
- Session-specific metadata
- Temporary files for GitHub CLI operations (e.g., issue/PR body content)

## Git Behavior
- This directory is **git-ignored** (except this README)
- Files created here will NOT be accidentally committed
- Files persist across sessions until explicitly cleaned up

## Usage Example
```bash
# Create a temporary file for GitHub CLI
cat <<EOF_INNER > .agent/scratchpad/issue_body.md
# My Issue Title
Content here...
EOF_INNER

# Use it with gh
gh issue create --title "My Issue" --body-file .agent/scratchpad/issue_body.md
```

## Cleanup
- **Session-Level**: Clean up files when your session ends (unless needed for review)
- **Cross-Session**: Don't assume files are temporary across sessions
- Manually remove old files as needed

## Alternative: `/tmp`
For truly ephemeral files (cleaned up within the same command), use `/tmp` instead.
