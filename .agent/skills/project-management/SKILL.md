---
name: Project Management
description: Manage project tasks using GitHub Issues as Source of Truth. Sync local ROADMAP.md from GitHub for offline reference.
---

# Project Management Skill

Use this skill when asked to "add a task", "update the roadmap", "what are we working on?", or "prioritize X".

**⚠️ IMPORTANT**: Per `.agent/WORKFORCE_PROTOCOL.md`:
- **GitHub Issues are the Source of Truth** for all task tracking
- `.agent/ROADMAP.md` is a **read-only local snapshot** synced FROM GitHub Issues
- Never manually edit ROADMAP.md - always work with GitHub Issues directly

## Primary Operations (GitHub Issues)

### 1. Add Task
**Trigger**: "Add [task] to the backlog"
**Procedure**:
1.  Create a GitHub Issue using `gh issue create`:
    ```bash
    # Create issue body in scratchpad (to preserve formatting)
    cat > .agent/scratchpad/issue_body.md << 'EOF'
    ### Description
    [Task description]
    
    ### Acceptance Criteria
    - [ ] Criterion 1
    - [ ] Criterion 2
    
    ---
    **🤖 Authored-By**: `[Agent Name]`
    EOF
    
    # Create the issue
    gh issue create --title "[Task Title]" --label "enhancement" --body-file .agent/scratchpad/issue_body.md
    ```
2.  **AI Signature Required**: Always append the AI signature to issue body (see `.agent/rules/common/ai-signature.md`)
3.  Optional: Sync ROADMAP.md from GitHub (see "Sync ROADMAP" operation below)

### 2. Update Task Status
**Trigger**: "Mark [task] as In Progress" or "Close [task]"
**Procedure**:
1.  Find the issue:
    ```bash
    gh issue list --search "task keywords"
    gh issue view <number>
    ```
2.  Update status by:
    - **Assign to self**: `gh issue edit <number> --add-assignee @me`
    - **Add labels**: `gh issue edit <number> --add-label "in-progress"`
    - **Close issue**: `gh issue close <number> --comment "Completed: [summary]"`
3.  Optional: Sync ROADMAP.md from GitHub (see "Sync ROADMAP" operation below)

### 3. Review Current Status
**Trigger**: "What is the status?" or "Review the roadmap"
**Procedure**:
1.  Query GitHub Issues:
    ```bash
    # Issues assigned to me
    gh issue list --assignee @me
    
    # All open issues
    gh issue list --state open
    
    # By label
    gh issue list --label "high-priority"
    ```
2.  Summarize:
    - Active tasks (assigned issues)
    - High-priority items (labeled appropriately)
    - Recent completions (closed issues)

## Secondary Operations (Local Sync)

### 4. Sync ROADMAP from GitHub
**Trigger**: "Update the roadmap" or after significant GitHub Issue changes
**Purpose**: Generate a read-only local snapshot of GitHub Issues for offline reference
**Procedure**:
1.  Query all open and recently closed issues:
    ```bash
    gh issue list --state open --json number,title,assignees,labels,state --limit 100 > .agent/scratchpad/issues_open.json
    gh issue list --state closed --json number,title,closedAt,labels --limit 20 > .agent/scratchpad/issues_closed.json
    ```
2.  Generate ROADMAP.md from JSON data:
    - Parse JSON to extract issue details
    - Categorize by labels (e.g., "high-priority", "in-progress")
    - Group assigned issues as "Active Tasks"
    - Group unassigned open issues as "Backlog"
    - List recently closed issues as "Completed"
    - Add timestamp in ISO 8601 format (e.g., `2026-01-28T12:00:00Z`)
3.  Prepend deprecation notice to generated file
4.  **Note**: This is currently a manual process. A future automation script could be added to `.agent/scripts/sync_roadmap.sh` to standardize the JSON parsing and formatting.
5.  **Never manually edit ROADMAP.md** - it should always be generated from GitHub data

## Best Practices

### GitHub CLI Usage
- **Always use `--body-file`** for multi-line content (see `.agent/rules/common/github-cli-best-practices.md`)
- **Include AI Signature** in all issues/PRs/comments (see `.agent/rules/common/ai-signature.md`)
- **Use scratchpad** for temporary files: `.agent/scratchpad/issue_body.md`

### Issue Creation Examples

**Feature Request**:
```bash
cat > .agent/scratchpad/issue_body.md << 'EOF'
### Description
Add support for multi-sensor fusion in the navigation stack.

### Motivation
Current implementation only supports single sensor input, limiting accuracy.

### Acceptance Criteria
- [ ] Support multiple sensor inputs
- [ ] Implement sensor fusion algorithm
- [ ] Add tests for multi-sensor scenarios

---
**🤖 Authored-By**: `Copilot CLI Agent`
EOF

gh issue create --title "[Feature] Multi-sensor fusion support" \
  --label "enhancement,high-priority" \
  --body-file .agent/scratchpad/issue_body.md
```

**Bug Report**:
```bash
cat > .agent/scratchpad/issue_body.md << 'EOF'
### Description
Navigation node crashes when receiving malformed sensor data.

### Steps to Reproduce
1. Start navigation node
2. Send malformed sensor message
3. Observe crash

### Expected Behavior
Node should handle malformed data gracefully with error logging.

---
**🤖 Authored-By**: `Copilot CLI Agent`
EOF

gh issue create --title "[Bug] Navigation crash on malformed data" \
  --label "bug,high-priority" \
  --body-file .agent/scratchpad/issue_body.md
```

### Task Workflow

1. **Before Starting Work**:
   ```bash
   # Check for available tasks
   gh issue list --assignee @me
   gh issue list --label "good-first-issue"
   
   # View task details
   gh issue view <number>
   ```

2. **Claim a Task**:
   ```bash
   # Assign to yourself
   gh issue edit <number> --add-assignee @me
   
   # Add in-progress label
   gh issue edit <number> --add-label "in-progress"
   
   # Comment on the issue
   gh issue comment <number> --body "Starting work on this issue"
   ```

3. **Complete a Task**:
   ```bash
   # Close with summary
   gh issue close <number> --comment "Completed: [brief summary of changes]"
   
   # Link PR to issue (in PR description)
   # "Closes #<number>"
   ```

## ROADMAP.md Schema (Auto-Generated)

When syncing from GitHub, the generated ROADMAP.md should follow this structure:

```markdown
# Workspace Roadmap [DEPRECATED]

**⚠️ DEPRECATION NOTICE**: This file is a read-only snapshot synced from GitHub Issues. 
Do not edit manually. Refer to **GitHub Issues** for the active task list.

**Last Synced**: 2026-01-28T12:00:00Z (ISO 8601 format)

## Active Tasks (Assigned Issues)
- [#42] Multi-sensor fusion support (Assigned: @user, Labels: enhancement, in-progress)
- [#45] Navigation crash fix (Assigned: @agent, Labels: bug, high-priority)

## Backlog (Open Unassigned Issues)
### High Priority
- [#50] Add ROS2 parameter validation (Labels: enhancement, high-priority)

### Standard Priority
- [#48] Update documentation (Labels: documentation)

## Recently Completed
- [#40] Bootstrap automation (Closed: 2026-01-20, Labels: enhancement)
- [#38] AI identity strategy (Closed: 2026-01-18, Labels: system-improvement)
```

## Migration Notes

**For Legacy ROADMAP.md Entries**:
1. Review existing task entries in ROADMAP.md (may use `[TASK-XXX]` format or other patterns)
2. For each active task, create a corresponding GitHub Issue
3. Reference the original identifier in the issue body:
   - If using TASK-ID format: "Migration from ROADMAP.md TASK-XXX"
   - If no ID present: Include the original task title and description
   - If using custom format: Note the original format in the issue
4. After migration, regenerate ROADMAP.md from GitHub Issues using the sync procedure
5. Verify all tasks are accounted for by comparing counts
6. Archive the original ROADMAP.md content if needed for historical reference

## See Also
- `.agent/WORKFORCE_PROTOCOL.md` - Multi-agent coordination using GitHub Issues
- `.agent/rules/common/issue-first.md` - Issue-first policy
- `.agent/rules/common/ai-signature.md` - Required signature format
- `.agent/rules/common/github-cli-best-practices.md` - GitHub CLI usage patterns
