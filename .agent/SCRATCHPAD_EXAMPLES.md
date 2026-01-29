# Scratchpad Usage Examples with mktemp

This document provides practical examples for using `mktemp` with the scratchpad directory to prevent file name collisions in multi-agent scenarios.

## Why mktemp?

`mktemp` is the standard POSIX utility for creating unique temporary files:
- ✅ Atomic file creation (no race conditions)
- ✅ Built-in uniqueness guarantees
- ✅ Available on all POSIX systems
- ✅ Simple and well-understood
- ✅ No dependencies on environment variables

## Basic Usage

```bash
# Create a unique temporary file
TEMP_FILE=$(mktemp .agent/scratchpad/myfile.XXXXXX.txt)

# Use the file
echo "content" > "$TEMP_FILE"

# Clean up when done
rm "$TEMP_FILE"
```

The `XXXXXX` will be replaced with a unique random string.

## Example 1: GitHub CLI Issue Creation

**Scenario**: Create a GitHub issue with markdown body content.

```bash
# Create unique file for issue body
BODY_FILE=$(mktemp .agent/scratchpad/issue_body.XXXXXX.md)

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
# Create unique analysis file
ANALYSIS_FILE=$(mktemp .agent/scratchpad/code_analysis.XXXXXX.md)

# Generate analysis
echo "# Code Analysis Report" > "$ANALYSIS_FILE"
echo "" >> "$ANALYSIS_FILE"
echo "## Findings" >> "$ANALYSIS_FILE"
grep -r "TODO" src/ >> "$ANALYSIS_FILE"

# Show to user
cat "$ANALYSIS_FILE"

# Keep for review (don't delete immediately)
echo "Analysis saved to: $ANALYSIS_FILE"
```

## Example 3: Build Report with Timestamp

**Scenario**: Generate build report that persists for review.

```bash
# Create timestamped build report
BUILD_REPORT=$(mktemp .agent/scratchpad/build_report.XXXXXX.json)

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

## Example 4: Cached GitHub Data

**Scenario**: Cache GitHub API responses to avoid rate limiting.

```bash
# Create unique cache file
CACHE_FILE=$(mktemp .agent/scratchpad/pr_cache.XXXXXX.json)

# Fetch and cache
gh pr list --json number,title,state --limit 20 > "$CACHE_FILE"

# Use cached data
jq '.[] | select(.state == "OPEN")' "$CACHE_FILE"

# Clean up when done
rm "$CACHE_FILE"
```

## Example 5: Multiple Agents Working Concurrently

**Scenario**: Two agents create issues at the same time without collision.

**Agent A (Copilot CLI)**:
```bash
BODY_FILE=$(mktemp .agent/scratchpad/issue_body.XXXXXX.md)
cat > "$BODY_FILE" << 'EOF'
Issue from Agent A
EOF
gh issue create --title "Agent A Issue" --body-file "$BODY_FILE"
rm "$BODY_FILE"
```

**Agent B (Antigravity) - Running simultaneously**:
```bash
BODY_FILE=$(mktemp .agent/scratchpad/issue_body.XXXXXX.md)
cat > "$BODY_FILE" << 'EOF'
Issue from Agent B
EOF
gh issue create --title "Agent B Issue" --body-file "$BODY_FILE"
rm "$BODY_FILE"
```

**Result**: Both agents create their files without collision because `mktemp` generates unique filenames atomically.

Example filenames:
- `.agent/scratchpad/issue_body.a1b2c3.md`
- `.agent/scratchpad/issue_body.x7y8z9.md`

## Example 6: Using Subdirectories for Organization

**Scenario**: Organize temporary files in subdirectories.

```bash
# Ensure subdirectory exists
mkdir -p .agent/scratchpad/temp

# Create file in subdirectory
TEMP_FILE=$(mktemp .agent/scratchpad/temp/analysis.XXXXXX.md)

# Use the file
echo "Analysis data" > "$TEMP_FILE"

# Clean up
rm "$TEMP_FILE"
```

## Example 7: Bulk Cleanup

**Scenario**: Clean up old temporary files.

```bash
# Clean up files older than 1 day
find .agent/scratchpad/ -type f -mtime +1 -delete

# Clean up files older than 1 hour
find .agent/scratchpad/ -type f -mmin +60 -delete

# Clean up by pattern (be careful!)
rm .agent/scratchpad/issue_body.* 2>/dev/null || true
```

## Best Practices

1. **Always use mktemp** for temporary files in scratchpad
2. **Clean up files immediately** after one-time use (like GitHub CLI body files)
3. **Keep files with meaningful names** for reports that may be reviewed later
4. **Use descriptive prefixes** in the template (e.g., `issue_body.XXXXXX.md` not `temp.XXXXXX.md`)
5. **Include file extension** after the XXXXXX for clarity

## What NOT to Do

❌ **Don't use static filenames**:
```bash
# BAD - Will collide with other agents
cat > .agent/scratchpad/issue_body.md << 'EOF'
```

❌ **Don't forget to clean up one-time files**:
```bash
# BAD - Creates clutter
BODY_FILE=$(mktemp .agent/scratchpad/issue_body.XXXXXX.md)
cat > "$BODY_FILE" << 'EOF'
...
EOF
gh issue create --body-file "$BODY_FILE"
# Missing: rm "$BODY_FILE"
```

❌ **Don't use /tmp for persistent files**:
```bash
# BAD - /tmp may be cleared automatically
REPORT=$(mktemp /tmp/analysis.XXXXXX.md)
# This file might disappear on reboot
```

## Troubleshooting

**Q: mktemp command not found**

`mktemp` is standard on Linux and macOS. If it's not available, you're on a very unusual system.

**Q: Can I use mktemp without a template?**

Yes, but it will create files in /tmp:
```bash
TEMP_FILE=$(mktemp)  # Creates /tmp/tmp.XXXXXXXXXX
```

For scratchpad, always provide a template path.

**Q: How do I ensure the scratchpad directory exists?**

```bash
mkdir -p .agent/scratchpad
TEMP_FILE=$(mktemp .agent/scratchpad/file.XXXXXX.md)
```

---
**Last Updated**: 2026-01-29  
**Related**: `.agent/scratchpad/README.md`, `.agent/SCRATCHPAD_COLLISION_ANALYSIS.md`
