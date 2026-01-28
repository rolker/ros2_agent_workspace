# Scratchpad Helper Functions - Usage Examples

This document provides practical examples for using the scratchpad helper functions to prevent file name collisions in multi-agent scenarios.

## Setup

First, source the helper library and set your AGENT_ID:

```bash
# Source the library
source .agent/scripts/lib/scratchpad_helpers.sh

# Set your agent identity (recommended)
export AGENT_ID="copilot_cli"  # or "antigravity", "gemini_cli", etc.
```

## Example 1: GitHub CLI Issue Creation

**Scenario**: Create a GitHub issue with markdown body content.

```bash
source .agent/scripts/lib/scratchpad_helpers.sh
export AGENT_ID="copilot_cli"

# Create unique file for issue body
BODY_FILE=$(scratchpad_file "issue_body" ".md")

# Write content
cat > "$BODY_FILE" << 'EOF'
### Description
Implement new feature X

### Acceptance Criteria
- [ ] Criterion 1
- [ ] Criterion 2

---
**🤖 Authored-By**: `Copilot CLI Agent`
EOF

# Use with GitHub CLI
gh issue create \
  --title "Implement feature X" \
  --label "enhancement" \
  --body-file "$BODY_FILE"

# Clean up
rm "$BODY_FILE"
```

## Example 2: Analysis Report Generation

**Scenario**: Generate an analysis report that may be reviewed later.

```bash
source .agent/scripts/lib/scratchpad_helpers.sh
export AGENT_ID="copilot_cli"

# Create unique analysis file
ANALYSIS_FILE=$(scratchpad_file "code_analysis" ".md")

# Generate analysis
echo "# Code Analysis Report" > "$ANALYSIS_FILE"
echo "" >> "$ANALYSIS_FILE"
echo "## Findings" >> "$ANALYSIS_FILE"
grep -r "TODO" src/ >> "$ANALYSIS_FILE"

# Show to user
cat "$ANALYSIS_FILE"

# Don't delete - may be needed for review
echo "Analysis saved to: $ANALYSIS_FILE"
```

## Example 3: Build Report with Timestamp

**Scenario**: Generate build report with timestamp-based naming for historical tracking.

```bash
source .agent/scripts/lib/scratchpad_helpers.sh
export AGENT_ID="copilot_cli"

# Create timestamped build report
BUILD_REPORT=$(scratchpad_file "build_report" ".json")

# Generate report
cat > "$BUILD_REPORT" << EOF
{
  "timestamp": "$(date -Iseconds)",
  "status": "success",
  "packages_built": 42
}
EOF

echo "Build report: $BUILD_REPORT"
```

## Example 4: Using Agent Subdirectories

**Scenario**: Use namespaced files for better organization.

```bash
source .agent/scripts/lib/scratchpad_helpers.sh
export AGENT_ID="copilot_cli"

# Create file in agent-specific subdirectory
# This creates .agent/scratchpad/copilot_cli/analysis_<timestamp>.md
REPORT=$(scratchpad_file_namespaced "analysis" ".md")

echo "# My Analysis" > "$REPORT"
echo "Report location: $REPORT"

# List all my files
scratchpad_list_mine
```

## Example 5: Session Cleanup

**Scenario**: Clean up files at the end of a session.

```bash
source .agent/scripts/lib/scratchpad_helpers.sh
export AGENT_ID="copilot_cli"

# Create some temporary files
FILE1=$(scratchpad_file "temp1" ".txt")
FILE2=$(scratchpad_file "temp2" ".txt")
echo "data" > "$FILE1"
echo "data" > "$FILE2"

# ... do work ...

# Clean up all files created by this agent
scratchpad_cleanup

# Or clean up only files older than 1 hour (3600 seconds)
# scratchpad_cleanup 3600
```

## Example 6: Cached GitHub Data

**Scenario**: Cache GitHub API responses to avoid rate limiting.

```bash
source .agent/scripts/lib/scratchpad_helpers.sh
export AGENT_ID="copilot_cli"

# Create unique cache file
CACHE_FILE=$(scratchpad_file "pr_cache" ".json")

# Fetch and cache
gh pr list --json number,title,state --limit 20 > "$CACHE_FILE"

# Use cached data
jq '.[] | select(.state == "OPEN")' "$CACHE_FILE"

# Clean up when done
rm "$CACHE_FILE"
```

## Example 7: Multiple Agents Working Concurrently

**Scenario**: Two agents create issues at the same time without collision.

**Agent A (Copilot CLI)**:
```bash
export AGENT_ID="copilot_cli"
source .agent/scripts/lib/scratchpad_helpers.sh

BODY_FILE=$(scratchpad_file "issue_body" ".md")
cat > "$BODY_FILE" << 'EOF'
Issue from Agent A
EOF
gh issue create --title "Agent A Issue" --body-file "$BODY_FILE"
rm "$BODY_FILE"
```

**Agent B (Antigravity) - Running simultaneously**:
```bash
export AGENT_ID="antigravity"
source .agent/scripts/lib/scratchpad_helpers.sh

BODY_FILE=$(scratchpad_file "issue_body" ".md")
cat > "$BODY_FILE" << 'EOF'
Issue from Agent B
EOF
gh issue create --title "Agent B Issue" --body-file "$BODY_FILE"
rm "$BODY_FILE"
```

**Result**: Both agents create their files without collision because filenames include:
- Agent ID (`copilot_cli` vs `antigravity`)
- Nanosecond timestamp
- Process ID

Example filenames:
- `.agent/scratchpad/copilot_cli_issue_body_1769567549030881398_4464.md`
- `.agent/scratchpad/antigravity_issue_body_1769567549030920889_5123.md`

## Best Practices

1. **Always set AGENT_ID** before using helper functions
2. **Clean up files** you create unless they're needed for review
3. **Use scratchpad_file()** for simple cases
4. **Use scratchpad_file_namespaced()** when you have many related files
5. **Delete files immediately** after one-time use (like GitHub CLI body files)
6. **Keep files with timestamps** for reports that may be reviewed later
7. **Use scratchpad_cleanup()** at the end of your session

## What NOT to Do

❌ **Don't use static filenames**:
```bash
# BAD - Will collide with other agents
cat > .agent/scratchpad/issue_body.md << 'EOF'
```

❌ **Don't forget to source the library**:
```bash
# BAD - Function not defined
BODY_FILE=$(scratchpad_file "issue" ".md")  # Error: command not found
```

❌ **Don't skip AGENT_ID**:
```bash
# BAD - Files will be named "unknown_..."
scratchpad_file "test" ".txt"  # Works but less organized
```

❌ **Don't leave temporary files**:
```bash
# BAD - Creates clutter
BODY_FILE=$(scratchpad_file "issue_body" ".md")
cat > "$BODY_FILE" << 'EOF'
...
EOF
gh issue create --body-file "$BODY_FILE"
# Missing: rm "$BODY_FILE"
```

## Troubleshooting

**Q: The helper functions aren't working**
```bash
# Make sure you sourced the library
source .agent/scripts/lib/scratchpad_helpers.sh

# Check if function exists
type scratchpad_file
```

**Q: Files aren't getting unique names**
```bash
# Check your system supports nanoseconds
date +%s%N  # Should show nanoseconds (19 digits)

# If not, it falls back to seconds + PID which is still unique
```

**Q: How do I find files created by a specific agent?**
```bash
# List files matching agent ID pattern
ls -lh .agent/scratchpad/copilot_cli_*

# Or use the helper
export AGENT_ID="copilot_cli"
source .agent/scripts/lib/scratchpad_helpers.sh
scratchpad_list_mine
```

**Q: Can I use these functions in Python or other languages?**

The functions are Bash-specific, but you can implement the same pattern:
```python
import os
import time

def scratchpad_file(base_name, extension=".txt"):
    agent_id = os.environ.get("AGENT_ID", "unknown")
    timestamp = int(time.time() * 1_000_000_000)  # nanoseconds
    pid = os.getpid()
    return f".agent/scratchpad/{agent_id}_{base_name}_{timestamp}_{pid}{extension}"

# Usage
body_file = scratchpad_file("issue_body", ".md")
```

---
**Last Updated**: 2026-01-28  
**Related**: `.agent/scratchpad/README.md`, `.agent/scratchpad/collision_analysis.md`
