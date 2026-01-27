---
description: Comprehensive workspace status check (Local + GitHub)
---

# Workspace Status Check

This workflow provides a comprehensive status report combining local git state, build status, and GitHub activity. It automatically detects if you're using a CLI framework with GitHub API access and uses faster native tools when available.

## Quick Usage

```bash
# Run the status report script
.agent/scripts/status_report.sh

# Or use this workflow step-by-step (see below)
```

## Workflow Steps

### 1. Detect Environment

Check if running under a CLI framework with enhanced capabilities:

```bash
source .agent/scripts/detect_cli_env.sh
echo "Framework: $AGENT_FRAMEWORK"
```

**Supported frameworks**:
- `copilot-cli` - Has GitHub API access via `gh` CLI
- `gemini-cli` - May have Google Cloud integrations
- `antigravity` - Standard shell-based checks
- `unknown` - Fallback to standard tools

### 2. Local Git Status

**Option A: Use status_report.sh (Recommended)**
```bash
.agent/scripts/status_report.sh
```

**Option B: Manual checks**
```bash
# Workspace repository status
git status

# All repositories in workspaces/
vcs status workspaces/*/src 2>/dev/null || echo "No workspaces cloned yet"

# Check for uncommitted changes
if [ -n "$(git status --porcelain)" ]; then
    echo "⚠️  Uncommitted changes detected"
    git status --short
fi
```

### 3. Build Status

Check if workspaces have been built:

```bash
for ws in workspaces/*_ws; do
    if [ -d "$ws/install" ]; then
        echo "✅ $(basename $ws): Built"
    else
        echo "❌ $(basename $ws): Not built"
    fi
done
```

### 4. GitHub Status

**Option A: CLI framework with GitHub API access (FAST)**

If `AGENT_FRAMEWORK=copilot-cli` or `gh` CLI is available:

```bash
if command -v gh &> /dev/null; then
    echo "=== Open Pull Requests ==="
    gh pr list --limit 20
    
    echo ""
    echo "=== Assigned Issues ==="
    gh issue list --assignee @me --limit 10
    
    echo ""
    echo "=== Recent Issues ==="
    gh issue list --limit 10 --state open
    
    # Cache results to avoid rate limiting
    gh pr list --json number,title,updatedAt --limit 20 > .agent/scratchpad/pr_cache.json 2>/dev/null || true
    gh issue list --json number,title,labels,assignees --limit 20 > .agent/scratchpad/issue_cache.json 2>/dev/null || true
else
    echo "GitHub CLI (gh) not available, skipping GitHub status"
    echo "Install with: https://cli.github.com/"
fi
```

**Option B: Fallback to web API (SLOWER)**

If GitHub CLI not available, use curl:

```bash
echo "=== GitHub Status (via API) ==="
curl -s "https://api.github.com/repos/rolker/ros2_agent_workspace/pulls?state=open" | \
    grep -E '"title"|"number"' | \
    head -20

curl -s "https://api.github.com/repos/rolker/ros2_agent_workspace/issues?state=open" | \
    grep -E '"title"|"number"' | \
    head -20
```

### 5. Generate Status Report

Combine findings into a structured report:

```bash
cat << EOF
================================================================================
WORKSPACE STATUS REPORT
Date: $(date +"%Y-%m-%d %H:%M:%S")
Framework: ${AGENT_FRAMEWORK:-unknown}
================================================================================

LOCAL STATUS:
$(git status --short)

BUILD STATUS:
$(for ws in workspaces/*_ws; do
    if [ -d "$ws/install" ]; then
        echo "  ✅ $(basename $ws)"
    else
        echo "  ❌ $(basename $ws) (not built)"
    fi
done)

GITHUB STATUS:
$(if command -v gh &> /dev/null; then
    echo "Open PRs: $(gh pr list --json number | jq '. | length')"
    echo "Open Issues: $(gh issue list --json number | jq '. | length')"
    echo "Assigned to me: $(gh issue list --assignee @me --json number | jq '. | length')"
else
    echo "(GitHub CLI not available - install 'gh' for detailed status)"
fi)

ACTION ITEMS:
$(if [ -n "$(git status --porcelain)" ]; then
    echo "  ⚠️  Uncommitted changes - review and commit"
fi)
$(if command -v gh &> /dev/null; then
    PR_COUNT=$(gh pr list --json number 2>/dev/null | jq '. | length' 2>/dev/null || echo "0")
    if [ "$PR_COUNT" -gt 0 ]; then
        echo "  📋 $PR_COUNT open PR(s) - review or merge"
    fi
    
    ISSUE_COUNT=$(gh issue list --assignee @me --json number 2>/dev/null | jq '. | length' 2>/dev/null || echo "0")
    if [ "$ISSUE_COUNT" -gt 0 ]; then
        echo "  🎯 $ISSUE_COUNT issue(s) assigned to you"
    fi
fi)

================================================================================
EOF
```

## Framework-Specific Optimizations

### GitHub Copilot CLI

Uses `gh` CLI for fast GitHub operations:
- ✅ Native PR/issue listing
- ✅ JSON output for programmatic parsing
- ✅ Respects GitHub API rate limits
- ✅ Cached results in `.agent/scratchpad/`

### Gemini CLI

Standard git/vcs checks with optional Google Cloud integrations.

### Fallback (Unknown Framework)

- Uses `git` and `vcs` commands
- Web API calls for GitHub data (slower, rate-limited)
- No caching

## Caching Strategy

To avoid GitHub API rate limits:

```bash
# Create cache directory if needed
mkdir -p .agent/scratchpad

# Cache PR data (valid for 5 minutes)
CACHE_FILE=".agent/scratchpad/pr_cache.json"
CACHE_AGE=$(($(date +%s) - $(stat -c %Y "$CACHE_FILE" 2>/dev/null || echo 0)))

if [ $CACHE_AGE -gt 300 ]; then
    # Cache older than 5 minutes, refresh
    gh pr list --json number,title,updatedAt --limit 20 > "$CACHE_FILE" 2>/dev/null
fi

# Use cached data
jq -r '.[] | "\(.number): \(.title)"' "$CACHE_FILE"
```

## Troubleshooting

### "gh: command not found"

Install GitHub CLI:
```bash
# Ubuntu/Debian
sudo apt install gh

# Or see: https://cli.github.com/
```

### "API rate limit exceeded"

Use cached results or wait:
```bash
# Check rate limit status
gh api rate_limit

# Use cached data
cat .agent/scratchpad/pr_cache.json | jq
```

### "vcs: command not found"

Install vcstool:
```bash
pip install vcstool
```

## Related Workflows

- `/check-local-status` - Fast git-only status check
- `/check-github-status` - GitHub PRs and issues only
- `/build-all` - Build all workspaces after status check

---

**Last Updated**: 2026-01-27  
**Maintained By**: Framework Engineering Team
