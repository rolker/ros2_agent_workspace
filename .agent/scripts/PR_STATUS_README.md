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

### For Humans (Interactive)

**Basic Mode (Visual Dashboard):**
```bash
.agent/scripts/pr_status.sh
```
Shows categorized list with colors and emojis.

**Interactive Mode (Menu-Driven):**
```bash
.agent/scripts/pr_status.sh --interactive
```
Menu interface for PR actions (review, fix, merge).

### For AI Agents (Programmatic)

**Simple Text Summary:**
```bash
.agent/scripts/pr_status.sh --simple
```
Clean text output without colors/emojis, easy to parse.

**JSON Output:**
```bash
.agent/scripts/pr_status.sh --json
```
Full structured data in JSON format.

**Query Modes:**
```bash
# Get next PR with critical issues
.agent/scripts/pr_status.sh --next-critical

# Get next PR with minor issues only
.agent/scripts/pr_status.sh --next-minor

# Get next PR needing review
.agent/scripts/pr_status.sh --next-review
```

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

**Agent-friendly simple output:**
```bash
$ .agent/scripts/pr_status.sh --simple

SUMMARY: 0 need review, 5 critical, 1 minor, 0 ready

CRITICAL ISSUES:
  #163: Add read_feature_status CLI tool (1 critical, 7 minor)
  #160: feat: add repository-aware worktree naming (3 critical, 11 minor)
  #159: Add Promptfoo framework for agent instruction testing (4 critical, 19 minor)
```

**Agent query for next critical PR:**
```bash
$ .agent/scripts/pr_status.sh --next-critical

{
  "category": "critical",
  "number": "163",
  "title": "Add read_feature_status CLI tool",
  "time": "1h ago",
  "critical": 1,
  "minor": 7
}
```

**Agent JSON output:**
```bash
$ .agent/scripts/pr_status.sh --json | jq '.summary'

{
  "total": 6,
  "needs_review": 0,
  "critical": 5,
  "minor": 1,
  "ready": 0
}
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

**Agent workflow (via Copilot CLI):**
```bash
# Get simple summary
$ .agent/scripts/pr_status.sh --simple
SUMMARY: 0 need review, 5 critical, 1 minor, 0 ready

# Get next critical PR to work on
$ .agent/scripts/pr_status.sh --next-critical
{"category":"critical","number":"163","title":"Add read_feature_status CLI tool"...}

# Agent then works on PR #163
# After fixes, check again to get next PR
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

## Note

Basic PR comment triage (critical/minor classification) is now available in
`status_report.sh --pr-triage`, which operates across all workspace repos.
This standalone `pr_status.sh` is still the right tool for interactive mode,
JSON output, and `--next-critical`/`--next-minor` agent queries.

## See Also

- `status_report.sh --pr-triage` - Cross-repo PR comment classification
- `CONTRIBUTING.md` - Contribution guidelines
