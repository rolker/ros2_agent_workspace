# PR Status Dashboard

Provides visibility into PR pipeline status and interactive PR management.

## Features

- **Status Categorization**: Automatically categorizes PRs by review state
  - 🟡 Needs Review (unreviewed)
  - 🔴 Critical Issues (blocking comments)
  - 🟠 Minor Issues (non-blocking comments)
  - 🟢 Ready to Merge (approved, no issues)

- **Comment Classification**: Analyzes review comments for severity
  - Critical: security, bugs, errors, vulnerabilities
  - Minor: style, formatting, suggestions

- **Interactive Management**: Menu-driven actions for PR workflow
  - Launch Copilot review
  - Hand off fixes to agent
  - Merge approved PRs

## Usage

### Basic Mode (Status Display)

```bash
.agent/scripts/pr_status.sh
```

Shows categorized list of all open PRs with comment counts and timing.

### Interactive Mode

```bash
.agent/scripts/pr_status.sh --interactive
```

Provides menu-driven interface for PR actions:
1. Review a PR (launch Copilot review)
2. Fix issues on a PR (instructions for agent handoff)
3. Merge a PR
4. Refresh status

## Examples

**Basic status check:**
```bash
$ .agent/scripts/pr_status.sh

🔍 PR Status Dashboard
====================

🔴 CRITICAL ISSUES (2)
  #159  Add Promptfoo framework    (last: 2h ago)
        → 4 critical comment(s)
        → 19 minor comment(s)
  #158  Use git worktrees          (last: recently)
        → 2 critical comment(s)
        → 5 minor comment(s)

📊 Summary: 5 open PRs | 0 need review | 5 need fixes | 0 ready
```

**Interactive workflow:**
```bash
$ .agent/scripts/pr_status.sh --interactive

[Shows dashboard]

What would you like to do?
1) Review a PR (launch Copilot review)
2) Fix issues on a PR (launch agent)
3) Merge a PR
4) Refresh
q) Quit

> 2
Enter PR number: 159
To launch agent for PR #159, use:
  gh copilot 'Check comments on PR 159 and fix the issues'
```

## Comment Classification

The script automatically classifies review comments as **critical** or **minor** based on keywords:

**Critical keywords:**
- security, vulnerability, bug, error, critical
- dangerous, unsafe, leak

**Minor keywords:**
- style, formatting, typo, naming
- convention, prefer, consider, suggestion

Comments not matching either pattern default to minor.

## Requirements

- GitHub CLI (`gh`) with authentication
- `jq` for JSON parsing
- Repository access for PR data

## Integration

Designed to streamline the standard PR workflow:

1. **Scan** - Run basic mode to see what needs attention
2. **Review** - Use interactive mode to trigger reviews
3. **Fix** - Hand off issues to agents via interactive menu
4. **Merge** - Approve and merge when ready

Reduces manual PR scanning time and provides actionable next steps.

## See Also

- `.agent/workflows/dev/start-feature.md` - Feature development workflow
- `CONTRIBUTING.md` - Contribution guidelines
