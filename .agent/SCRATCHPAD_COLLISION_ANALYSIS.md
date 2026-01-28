# Scratchpad Name Collision Analysis

**Date**: 2026-01-28  
**Issue**: Review scratchpad functionality for multi-agent scenarios

## Executive Summary

The current scratchpad implementation has **significant collision risks** when multiple agents work concurrently. While timestamp-based naming is recommended, it is not enforced, and many examples use static filenames that will collide.

### Key Findings

1. ✅ **Good Design**: Git-ignored directory prevents accidental commits
2. ✅ **Good Practice**: Timestamp-based naming is documented as best practice
3. ❌ **Critical Issue**: No enforcement of unique naming patterns
4. ❌ **Documentation Gap**: Most examples use static names (e.g., `issue_body.md`)
5. ❌ **Race Condition**: No file existence checks before creation
6. ❌ **Incomplete Migration**: Many scripts still use `ai_workspace/` instead of `.agent/scratchpad/`

## Current Usage Patterns

### Pattern 1: Static Filenames (Collision-Prone) ❌

Found in `.agent/rules/common/github-cli-best-practices.md`:
```bash
cat <<EOF > .agent/scratchpad/issue_body.md
...
EOF
gh issue create --title "My Issue" --body-file .agent/scratchpad/issue_body.md
```

**Risk**: If two agents create issues simultaneously, they will overwrite each other's files.

### Pattern 2: Timestamp-Based Naming (Safe) ✅

Found in `.agent/rules/common/clean-root.md`:
```bash
output_file=".agent/scratchpad/build_report_$(date +%s).json"
echo "debug info" > .agent/scratchpad/debug_output_$(date +%s).txt
```

**Benefit**: Unix timestamp provides uniqueness (assuming agents don't start in same second).

### Pattern 3: Cached Data (Collision-Prone) ❌

Found in `.agent/workflows/ops/check-status.md`:
```bash
gh pr list --json number,title,updatedAt --limit 20 > .agent/scratchpad/pr_cache.json
gh issue list --json number,title,labels,assignees --limit 20 > .agent/scratchpad/issue_cache.json
```

**Risk**: Multiple agents querying GitHub will overwrite each other's caches.

## Multi-Agent Collision Scenarios

### Scenario 1: Concurrent Issue Creation
**Actors**: Agent A and Agent B both create issues at the same time.
```
T0: Agent A writes .agent/scratchpad/issue_body.md
T1: Agent B writes .agent/scratchpad/issue_body.md (overwrites A)
T2: Agent A runs gh issue create --body-file .agent/scratchpad/issue_body.md (uses B's content!)
T3: Agent B runs gh issue create --body-file .agent/scratchpad/issue_body.md (works correctly)
```
**Result**: Agent A creates an issue with Agent B's content.

### Scenario 2: Concurrent Status Checks
**Actors**: Agent A and Agent B both run status checks.
```
T0: Agent A fetches PR data → .agent/scratchpad/pr_cache.json
T1: Agent B fetches PR data → .agent/scratchpad/pr_cache.json (overwrites A)
T2: Agent A reads pr_cache.json (gets B's data, potentially inconsistent)
```
**Result**: Agent A may see incomplete/inconsistent data.

### Scenario 3: Timestamp Collision (Edge Case)
**Actors**: Two agents start within the same second.
```bash
# Both agents execute simultaneously
agent_a: echo "A" > .agent/scratchpad/report_$(date +%s).txt
agent_b: echo "B" > .agent/scratchpad/report_$(date +%s).txt
```
**Result**: If `$(date +%s)` returns same value, one file overwrites the other.

### Scenario 4: Session Cleanup Interference
**Actors**: Agent A finishing up, Agent B actively working.
```
T0: Agent A decides to clean up .agent/scratchpad/
T1: Agent B creates .agent/scratchpad/analysis.md
T2: Agent A runs rm .agent/scratchpad/*.md
T3: Agent B tries to read .agent/scratchpad/analysis.md (file gone!)
```
**Result**: Agent B's work is lost.

## Additional Issues Not Originally Considered

### 1. Migration Status (COMPLETED)
All relevant scripts have been migrated from `ai_workspace/` to `.agent/scratchpad/`:
- `.agent/scripts/build.sh` → writes to `.agent/scratchpad/build_report.md`
- `.agent/scripts/test.sh` → writes to `.agent/scratchpad/test_report.md`
- `.agent/scripts/lock.sh` → uses `.agent/scratchpad/workspace.lock`

**Impact**: Documentation and scripts are now aligned on using `.agent/scratchpad/` as the standard location.

### 2. No File Existence Checks
No examples show checking if a file exists before creating it:
```bash
# Current pattern (unsafe)
cat > .agent/scratchpad/issue_body.md << 'EOF'
...
EOF

# Safe pattern (missing)
if [ -f .agent/scratchpad/issue_body.md ]; then
    echo "Warning: File already exists"
fi
```

### 3. No Cleanup Automation
- README says "Clean up files when your session ends"
- No helper scripts to identify "my" files vs other agent's files
- No mechanism to mark ownership or track which agent created which files

### 4. Limited Namespace Isolation
All agents write to the same flat directory. No subdirectories per:
- Agent identity (e.g., `.agent/scratchpad/copilot/`, `.agent/scratchpad/antigravity/`)
- Session ID (e.g., `.agent/scratchpad/session-abc123/`)
- Task/Issue (e.g., `.agent/scratchpad/issue-46/`)

### 5. Lock File Placement (COMPLETED)
The workspace lock has been migrated to `.agent/scratchpad/workspace.lock`:
- ✅ Good: Prevents concurrent builds/tests
- ✅ Migrated: Now uses `.agent/scratchpad/` location
- Note: Lock is binary (locked/unlocked), could be enhanced to identify which agent holds it

## Recommended Mitigations

### Priority 1: Enforce Unique Naming (CRITICAL)

**Solution: Use `mktemp` Utility**

The standard POSIX `mktemp` utility is the simplest and most reliable solution:

```bash
# mktemp creates unique files atomically with built-in race condition protection
BODY_FILE=$(mktemp .agent/scratchpad/issue_body.XXXXXX.md)
cat > "$BODY_FILE" << 'EOF'
...
EOF
gh issue create --body-file "$BODY_FILE"
rm "$BODY_FILE"
```

**Advantages**:
- Standard utility available on all POSIX systems
- Atomic file creation (no race conditions)
- Built-in uniqueness guarantees
- No custom code to maintain
- No dependencies on environment variables

**Update Documentation Examples**:
Replace all static filename examples with `mktemp`:
```bash
# BEFORE (bad)
cat > .agent/scratchpad/issue_body.md << 'EOF'

# AFTER (good) - Using mktemp
BODY_FILE=$(mktemp .agent/scratchpad/issue_body.XXXXXX.md)
cat > "$BODY_FILE" << 'EOF'
...
EOF
gh issue create --body-file "$BODY_FILE"
rm "$BODY_FILE"
```

### Priority 2: Complete Migration to .agent/scratchpad/

**Action Items**:
1. Update all scripts in `.agent/scripts/` to use `.agent/scratchpad/`
2. Deprecate `ai_workspace/` directory
3. Add migration notice to docs
4. Update all documentation references

**Files to Update**:
- `.agent/scripts/build.sh`
- `.agent/scripts/test.sh`
- `.agent/scripts/lock.sh`
- `.agent/scripts/unlock.sh`
- `.agent/scripts/health_check.sh`
- `.agent/scripts/status_report.sh`

### Priority 3: Add Namespace Isolation (OPTIONAL)

With `mktemp`, namespace isolation is less critical since each file is already unique. However, for organizational purposes, you can still use subdirectories:

**Option: Use mktemp with subdirectories**
```bash
mkdir -p .agent/scratchpad/temp
BODY_FILE=$(mktemp .agent/scratchpad/temp/issue_body.XXXXXX.md)
cat > "$BODY_FILE" << 'EOF'
...
EOF
```

### Priority 4: File Existence Checks (NOT NEEDED)

With `mktemp`, file existence checks are unnecessary because `mktemp` handles this atomically and will never create a duplicate file.

### Priority 5: Improve Lock File System (MEDIUM)

**Enhanced Lock File** (optional future improvement):
```bash
acquire_lock() {
    local lock_file=".agent/scratchpad/workspace.lock"
    local timestamp=$(date -Iseconds)
    
    if [ -f "$lock_file" ]; then
        echo "❌ Workspace locked by: $(cat $lock_file)"
        return 1
    fi
    
    echo "Time: $timestamp" > "$lock_file"
    echo "PID: $$" >> "$lock_file"
    return 0
}
```

### Priority 6: Cleanup Helpers (OPTIONAL)

Simple cleanup by file age:
```bash
# Clean up files older than 1 day
find .agent/scratchpad/ -type f -mtime +1 -delete

# Clean up files older than 1 hour
find .agent/scratchpad/ -type f -mmin +60 -delete
```

## Implementation Roadmap

### Phase 1: Documentation Updates (Immediate)
- [x] Update documentation examples to use `mktemp`
- [x] Add warning in scratchpad README about name collisions
- [x] Update all skills and workflows to use `mktemp`

### Phase 2: Migration (COMPLETED)
- [x] Update all scripts to use `.agent/scratchpad/` instead of `ai_workspace/`
- [x] Move lock file to `.agent/scratchpad/workspace.lock`
- [x] Test all scripts after migration

### Phase 3: Documentation (Ongoing)
- [x] Create collision analysis document
- [x] Add examples showing safe concurrent usage
- [ ] Create troubleshooting guide for collision scenarios

## Testing Plan

### Manual Tests
1. **Concurrent Issue Creation**: Two agents create issues simultaneously using `mktemp`
2. **Concurrent Status Checks**: Two agents run status reports simultaneously
3. **mktemp Uniqueness**: Verify mktemp creates unique files even when called rapidly

### Automated Tests (Future)
- Script to simulate concurrent agent operations
- Verify file uniqueness across parallel operations with mktemp

## Conclusion

The scratchpad is a good concept but needed hardening for true multi-agent scenarios. The solution is to use the standard **`mktemp` utility** instead of custom helper functions. This provides:

- ✅ Atomic file creation (no race conditions)
- ✅ Built-in uniqueness guarantees
- ✅ Standard POSIX utility (no custom code)
- ✅ Simple and well-understood

With `mktemp`, the scratchpad can safely support unlimited concurrent agents.

---
**Analyzed by**: Copilot CLI Agent  
**Model**: GPT-4
