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

### 1. Migration Inconsistency
Many scripts still use `ai_workspace/` instead of `.agent/scratchpad/`:
- `.agent/scripts/build.sh` → writes to `ai_workspace/build_report.md`
- `.agent/scripts/test.sh` → writes to `ai_workspace/test_report.md`
- `.agent/scripts/lock.sh` → uses `ai_workspace/workspace.lock`

**Impact**: Documentation says use `.agent/scratchpad/`, but scripts use old location.

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

### 5. Lock File Placement
The workspace lock is in `ai_workspace/workspace.lock`:
- ✅ Good: Prevents concurrent builds/tests
- ❌ Issue: Uses deprecated `ai_workspace/` location
- ❌ Issue: Lock is binary (locked/unlocked), doesn't identify which agent holds it

## Recommended Mitigations

### Priority 1: Enforce Unique Naming (CRITICAL)

**A. Provide Helper Function**
Create `.agent/scripts/lib/scratchpad_helpers.sh`:
```bash
#!/bin/bash
# Generate unique filename in scratchpad

scratchpad_file() {
    local base_name="$1"
    local extension="${2:-.txt}"
    local agent_id="${AGENT_ID:-unknown}"
    local timestamp=$(date +%s%N)  # nanoseconds for better uniqueness
    
    echo ".agent/scratchpad/${agent_id}_${base_name}_${timestamp}${extension}"
}

# Usage:
# BODY_FILE=$(scratchpad_file "issue_body" ".md")
# cat > "$BODY_FILE" << EOF
# ...
# EOF
# gh issue create --body-file "$BODY_FILE"
```

**B. Update Documentation Examples**
Replace all static filename examples with unique naming:
```bash
# BEFORE (bad)
cat > .agent/scratchpad/issue_body.md << 'EOF'

# AFTER (good) - Using nanoseconds (Linux/macOS) or seconds+random (fallback)
BODY_FILE=".agent/scratchpad/issue_body_$(date +%s%N 2>/dev/null || echo "$(date +%s)${RANDOM}${RANDOM}").md"
cat > "$BODY_FILE" << 'EOF'
...
EOF
gh issue create --body-file "$BODY_FILE"
# Cleanup after use
rm "$BODY_FILE"
```

**Note**: The helper functions in `scratchpad_helpers.sh` automatically handle the fallback from nanoseconds to seconds+random when `date +%s%N` is not available. On systems without nanosecond support, the random number and PID components provide sufficient entropy to prevent collisions.

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

### Priority 3: Add Namespace Isolation (MEDIUM)

**Option A: Agent-Specific Subdirectories**
```bash
mkdir -p .agent/scratchpad/${AGENT_ID}/
cat > .agent/scratchpad/${AGENT_ID}/issue_body.md << 'EOF'
...
EOF
```

**Option B: Session-Specific Subdirectories**
```bash
SESSION_ID=${SESSION_ID:-$(date +%s)}
mkdir -p .agent/scratchpad/session-${SESSION_ID}/
cat > .agent/scratchpad/session-${SESSION_ID}/issue_body.md << 'EOF'
...
EOF
```

**Option C: Task-Specific Subdirectories**
```bash
ISSUE_NUM=46
mkdir -p .agent/scratchpad/issue-${ISSUE_NUM}/
cat > .agent/scratchpad/issue-${ISSUE_NUM}/analysis.md << 'EOF'
...
EOF
```

**Recommended**: Combination of Agent ID + Timestamp for most cases.

### Priority 4: Add File Existence Checks (LOW)

For truly critical operations, add checks:
```bash
BODY_FILE=".agent/scratchpad/issue_body_$(date +%s%N).md"

# Paranoid check (unlikely to be needed with timestamps)
if [ -f "$BODY_FILE" ]; then
    echo "⚠️  Warning: File collision detected, retrying..."
    sleep 0.1
    BODY_FILE=".agent/scratchpad/issue_body_$(date +%s%N).md"
fi

cat > "$BODY_FILE" << 'EOF'
...
EOF
```

### Priority 5: Improve Lock File System (MEDIUM)

**Enhanced Lock File**:
```bash
# .agent/scripts/lib/lock_helpers.sh
acquire_lock() {
    local lock_file=".agent/scratchpad/workspace.lock"
    local agent_id="${AGENT_ID:-unknown}"
    local timestamp=$(date -Iseconds)
    
    if [ -f "$lock_file" ]; then
        echo "❌ Workspace locked by: $(cat $lock_file)"
        return 1
    fi
    
    echo "Agent: $agent_id" > "$lock_file"
    echo "Time: $timestamp" >> "$lock_file"
    echo "PID: $$" >> "$lock_file"
    return 0
}

release_lock() {
    local lock_file=".agent/scratchpad/workspace.lock"
    rm -f "$lock_file"
}
```

### Priority 6: Add Cleanup Helpers (LOW)

**Session Cleanup Script**:
```bash
# .agent/scripts/cleanup_scratchpad.sh
#!/bin/bash
# Clean up this agent's files from scratchpad

AGENT_ID="${AGENT_ID:-unknown}"
SESSION_ID="${SESSION_ID}"

if [ -n "$SESSION_ID" ]; then
    # Clean up by session
    rm -rf ".agent/scratchpad/session-${SESSION_ID}/"
elif [ "$AGENT_ID" != "unknown" ]; then
    # Clean up by agent (careful!)
    echo "⚠️  Cleaning up files for agent: $AGENT_ID"
    find .agent/scratchpad/ -name "${AGENT_ID}_*" -delete
else
    echo "❌ Cannot clean up: No AGENT_ID or SESSION_ID set"
    exit 1
fi
```

## Implementation Roadmap

### Phase 1: Quick Wins (Immediate)
- [ ] Create helper function for unique filenames
- [ ] Update documentation examples to use timestamps
- [ ] Add warning in scratchpad README about name collisions

### Phase 2: Migration (Short Term)
- [ ] Update all scripts to use `.agent/scratchpad/` instead of `ai_workspace/`
- [ ] Move lock file to `.agent/scratchpad/workspace.lock`
- [ ] Test all scripts after migration

### Phase 3: Enhancements (Medium Term)
- [ ] Implement namespace isolation (agent subdirectories)
- [ ] Create cleanup helper scripts
- [ ] Enhance lock file with agent identity

### Phase 4: Documentation (Ongoing)
- [ ] Update all documentation with best practices
- [ ] Add examples showing safe concurrent usage
- [ ] Create troubleshooting guide for collision scenarios

## Testing Plan

### Manual Tests
1. **Concurrent Issue Creation**: Two agents create issues simultaneously
2. **Concurrent Status Checks**: Two agents run status reports simultaneously
3. **Cleanup During Active Work**: One agent cleans up while another works
4. **Timestamp Uniqueness**: Verify nanosecond timestamps prevent collisions

### Automated Tests (Future)
- Script to simulate concurrent agent operations
- Verify file uniqueness across parallel operations
- Test lock file acquisition/release under contention

## Conclusion

The scratchpad is a good concept but needs hardening for true multi-agent scenarios. The most critical fix is **enforcing unique filenames** through helper functions and updated documentation. The migration from `ai_workspace/` should also be completed to avoid confusion.

With these mitigations, the scratchpad can safely support multiple concurrent agents.

---
**Analyzed by**: Copilot CLI Agent  
**Model**: GPT-4
