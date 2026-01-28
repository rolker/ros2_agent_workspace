# Agent Scratchpad

This directory is for **persistent temporary artifacts** created during agent sessions.

## ⚠️ CRITICAL: Multi-Agent Name Collision Warning

**When multiple agents work concurrently, file name collisions WILL occur unless you follow these rules:**

1. **NEVER use static filenames** (e.g., `issue_body.md`, `analysis.txt`)
2. **ALWAYS use unique names** with timestamps, agent ID, and PID
3. **USE the helper functions** from `.agent/scripts/lib/scratchpad_helpers.sh`

**Bad (collision-prone)**:
```bash
cat > .agent/scratchpad/issue_body.md << 'EOF'
...
EOF
```

**Good (collision-safe)**:
```bash
source .agent/scripts/lib/scratchpad_helpers.sh
BODY_FILE=$(scratchpad_file "issue_body" ".md")
cat > "$BODY_FILE" << 'EOF'
...
EOF
gh issue create --body-file "$BODY_FILE"
rm "$BODY_FILE"  # Clean up after use
```

See `.agent/scratchpad/collision_analysis.md` for detailed analysis of collision scenarios.

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

## Usage Examples

### Recommended: Using Helper Functions (Collision-Safe)
```bash
# Source the helper library
source .agent/scripts/lib/scratchpad_helpers.sh

# Create a unique file for GitHub CLI
BODY_FILE=$(scratchpad_file "issue_body" ".md")
cat <<EOF > "$BODY_FILE"
# My Issue Title
Content here...
EOF

# Use it with gh
gh issue create --title "My Issue" --body-file "$BODY_FILE"

# Clean up after use
rm "$BODY_FILE"
```

### Alternative: Manual Timestamp-Based Naming
```bash
# Use nanosecond timestamp for uniqueness
BODY_FILE=".agent/scratchpad/issue_body_$(date +%s%N).md"
cat <<EOF > "$BODY_FILE"
# My Issue Title
Content here...
EOF

# Use it with gh
gh issue create --title "My Issue" --body-file "$BODY_FILE"

# Clean up after use
rm "$BODY_FILE"
```

### ❌ Anti-Pattern: Static Filenames (DO NOT USE)
```bash
# DON'T DO THIS - Multiple agents will overwrite each other's files!
cat <<EOF > .agent/scratchpad/issue_body.md
# My Issue Title
Content here...
EOF
```

## Cleanup

### Using Helper Functions
```bash
source .agent/scripts/lib/scratchpad_helpers.sh

# Clean up all files created by this agent
scratchpad_cleanup

# Clean up files older than 1 hour
scratchpad_cleanup 3600

# List files created by this agent
scratchpad_list_mine
```

### Manual Cleanup
- **Session-Level**: Clean up files when your session ends (unless needed for review)
- **Cross-Session**: Don't assume files are temporary across sessions
- Only delete files you created (check AGENT_ID prefix)
- Manually remove old files as needed

**Example**:
```bash
# Clean up only your files (if you set AGENT_ID)
rm .agent/scratchpad/${AGENT_ID}_* 2>/dev/null || true
```

## Alternative: `/tmp`
For truly ephemeral files (cleaned up within the same command), use `/tmp` instead.

