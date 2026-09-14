# Agent Scratchpad

This directory is for **persistent temporary artifacts** created during agent sessions.

## ⚠️ CRITICAL: Multi-Agent Name Collision Warning

**When multiple agents work concurrently, file name collisions WILL occur unless you follow these rules:**

1. **NEVER use static filenames** (e.g., `issue_body.md`, `analysis.txt`)
2. **ALWAYS use `mktemp`** to create unique temporary files

**Bad (collision-prone)**:
```bash
cat > .agent/scratchpad/issue_body.md << 'EOF'
...
EOF
```

**Good (collision-safe)**:
```bash
# mktemp creates unique files atomically
BODY_FILE=$(mktemp .agent/scratchpad/issue_body.XXXXXX.md)
cat > "$BODY_FILE" << 'EOF'
...
EOF
gh issue create --body-file "$BODY_FILE"
rm "$BODY_FILE"  # Clean up after use
```

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

### Recommended: Using mktemp (Collision-Safe)
```bash
# Create a unique file for GitHub CLI
BODY_FILE=$(mktemp .agent/scratchpad/issue_body.XXXXXX.md)
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

### Manual Cleanup
- **Session-Level**: Clean up files when your session ends (unless needed for review)
- **Cross-Session**: Don't assume files are temporary across sessions
- Manually remove old files as needed

**Example**:
```bash
# Clean up your specific temporary files by pattern
rm .agent/scratchpad/issue_body.* 2>/dev/null || true

# Or clean up old loose files (older than 1 day) — top level only, so it
# cannot reach into a subdirectory that is a managed cache (see below)
find .agent/scratchpad -maxdepth 1 -type f -mtime +1 -delete
```

**Not everything here is loose scratch.** Some subdirectories are caches with
their own retention rules, and an age sweep run over them deletes the half a
long-lived run still needs:

- `.agent/scratchpad/janitor/` — `/janitor-sweep`'s reports, kept for the last
  20 runs by the skill itself. Nothing else prunes them, and a deleted report
  is gone — they are the durable output of a run, not a temp file.
- `.agent/scratchpad/janitor-repos/`, `.agent/scratchpad/manifest-repo/` —
  shallow clones. These self-heal (a missing or repointed clone is re-cloned),
  so the reclaim is deleting a **whole** directory, never individual files by
  age out from under a clone another agent is reading. Delete it only while no
  sweep or audit is running on this host: `resolve_repo_checkout.sh` holds its
  per-repo lock across the clone/refresh only, so a run reads its checkout
  unlocked and a tree that vanishes mid-audit surfaces as a phantom finding.

## Alternative: `/tmp`
For truly ephemeral files (cleaned up within the same command), use `/tmp` instead.
