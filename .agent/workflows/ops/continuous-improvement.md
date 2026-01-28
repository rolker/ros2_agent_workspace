---
description: Systematic process for AI agents to identify and report infrastructure friction
---

# Continuous Improvement Workflow

This workflow enables AI agents to systematically identify friction points during work sessions and report them as well-formed improvement issues.

## Purpose

AI agents encounter friction during sessions (unclear docs, missing workflows, confusing scripts) that could be fixed to improve all future agents' productivity. This workflow provides a standardized process to capture and report these issues with sufficient context.

## When to Run

Run this workflow:
- ✅ At the end of significant work sessions
- ✅ After completing major tasks
- ✅ When user asks "what could be better?" or "what friction did you encounter?"
- ✅ Periodically during retrospectives (e.g., weekly review)
- ✅ When you notice yourself working around the same issue repeatedly

## Prerequisites

**Required:**
- `gh` - GitHub CLI (for creating issues)
- `git` - Access to repository history
- Session context (what you worked on, what problems you encountered)

**Optional:**
- Session logs or checkpoint history
- Links to specific commits/files where friction occurred

## Workflow Steps

### Step 1: Identify Friction Points

Review your session for issues in these categories:

**Documentation Issues:**
- Unclear or incomplete instructions
- Missing examples or code snippets
- Outdated information
- Documentation doesn't match actual behavior

**Workflow Problems:**
- Commands that didn't work as expected
- Required workarounds or manual interventions
- Missing automation for repetitive tasks
- Confusing script behavior or error messages

**Environment/Setup Issues:**
- Path or environment variable problems
- Missing dependencies not caught by setup
- Version mismatches
- Configuration inconsistencies

**Discoverability Problems:**
- Useful tools/scripts that are hard to find
- Workflows that exist but aren't documented
- No clear guidance on which approach to use

**Coordination Issues:**
- Unclear handover process
- Duplicate work by multiple agents
- Missing visibility into work-in-progress

### Step 2: Check for Duplicate Issues

Before creating new issues, check if they already exist:

```bash
# List open issues (check last 50)
gh issue list --repo rolker/ros2_agent_workspace --state open --limit 50

# Search for specific topics
gh issue list --repo rolker/ros2_agent_workspace --search "documentation" --state open
gh issue list --repo rolker/ros2_agent_workspace --search "workflow" --state open
```

If an issue already exists:
- Add a comment with your additional context/examples
- Upvote or react to show importance
- Link your session evidence

### Step 3: Prioritize Issues

Assign priority based on impact and frequency:

**🔴 High Priority:**
- Blocks work or causes failures
- Affects all agents on common workflows
- Safety or correctness issues
- Data loss or security risks

**🟡 Medium Priority:**
- Reduces efficiency on common tasks
- Requires workarounds
- Affects specific roles (ROS Developer, Framework Engineer)
- Causes confusion that delays progress

**🟢 Low Priority:**
- Minor annoyances
- Rare edge cases
- Nice-to-have improvements
- Cosmetic issues

### Step 4: Draft Issues (Review Before Creating)

For each friction point, draft an issue using the template in `.agent/templates/improvement_issue.md`:

**Required sections:**
1. **Problem** - What went wrong? What was confusing?
2. **Example Friction** - Concrete command/code that failed or was unclear
3. **Proposed Solution** - Specific fix (script change, doc update, new workflow)
4. **Impact** - Who benefits? How much time/effort saved?
5. **Priority** - 🔴/🟡/🟢 with justification
6. **Session Evidence** - Link to checkpoint, session ID, or commit
7. **AI Signature** - Agent name and model

**Show the drafted issues to the user for review before creating them.**

Example draft:

```markdown
## Problem
The `sync-repos` workflow exists but isn't mentioned in CLI_COMMANDS.md, making it hard to discover.

## Example Friction
\`\`\`bash
# Agent tried:
vcs import workspaces/core_ws/src < configs/core.repos

# But sync-repos.md already exists with better error handling
.agent/scripts/sync_repos.sh core
\`\`\`

## Proposed Solution
Add `/sync-repos` to CLI_COMMANDS.md under "Development Commands" section.

## Impact
- ✅ New agents discover sync-repos on first read
- ✅ Reduces manual vcs commands errors
- ✅ ~5 minutes saved per agent per session

## Priority
🟡 Medium - Common workflow, not critical but reduces friction

## Session Evidence
Session: 66b4eea3
- Agent manually used vcs commands
- Found sync-repos.md later during exploration
- Workflow would have saved 10 minutes

---
**🤖 Reported-By**: Copilot CLI Agent (Claude 3.5 Sonnet)
```

### Step 5: Create Issues (After User Approval)

Use GitHub CLI to create issues with appropriate labels:

```bash
gh issue create --repo rolker/ros2_agent_workspace \
  --title "Make sync-repos discoverable in CLI_COMMANDS.md" \
  --label "enhancement,documentation" \
  --body-file /tmp/issue_draft.md
```

**Standard labels:**
- `enhancement` - Feature improvement
- `documentation` - Doc fixes
- `bug` - Something broken
- `agent-infrastructure` - Agent tooling/workflows
- `good-first-issue` - Easy wins

## Framework-Specific Notes

### GitHub Copilot CLI

Use `gh` CLI for all operations:
```bash
# Efficient issue creation
gh issue create --web  # Opens browser for rich editing

# Bulk issue review
gh issue list --assignee @me --json number,title,labels
```

### All Agents

Save your draft issues to `.agent/scratchpad/` before creating:
```bash
# Draft issues in scratchpad (git-ignored)
REPORT_DATE=$(date +%Y%m%d)
cat > .agent/scratchpad/friction_report_${REPORT_DATE}.md << EOF
# Friction Report - $(date)

## Issue 1: ...
[draft content]

## Issue 2: ...
[draft content]
EOF
```

## Success Criteria

This workflow is working well when:

- ✅ Agents proactively identify 2-3 improvements per major session
- ✅ Infrastructure issues are well-formed and actionable
- ✅ Friction points decrease over time (fewer new issues of same type)
- ✅ New agents onboard faster due to continuously improving docs
- ✅ Issues include concrete examples and session evidence
- ✅ No duplicate issues created (good checking process)

## Example Session Flow

1. **Complete task**: Work on Issue #46 for 2 hours
2. **Reflect**: What friction did I encounter?
   - Setup script failed on first run (missing dependency)
   - Documentation said to use `/build` but `/build-all` was needed
   - Spent 15 minutes finding the right workflow file
3. **Check duplicates**: Search existing issues - none found
4. **Draft issues**: Create 3 draft issues with examples
5. **Show user**: "I identified 3 friction points, here are the drafts..."
6. **Get approval**: User reviews and approves 2, says 3rd is expected behavior
7. **Create issues**: Use `gh issue create` for approved issues
8. **Link evidence**: Add comment with session ID and specific examples

## Templates

Use `.agent/templates/improvement_issue.md` for consistent issue formatting.

## Related Workflows

- `/check-status` - Review workspace state before reporting
- `/finish-feature` - Natural time to reflect on friction
- `.agent/FEEDBACK.md` - User feedback collection (different purpose)

## Related Issues

Examples of issues created through this process:
- #98 - Make sync-repos discoverable (Medium priority)
- #107 - Model parameter standardization (High priority)
- #108 - Testing workflows documentation (Medium priority)
- #109 - PR review workflow (Medium priority)

---

**Last Updated**: 2026-01-28  
**Maintained By**: Framework Engineering Team  
**See Also**: [CLI_COMMANDS.md](../../CLI_COMMANDS.md), [CONTRIBUTING.md](../../../CONTRIBUTING.md)
