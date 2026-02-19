#!/bin/bash
# .agent/scripts/push_request.sh
# Container-side script to signal that commits are ready for host-side push.
#
# Writes a JSON signal file to .agent/scratchpad/push-requests/ that the
# host-side push_gateway.sh reads after the container exits.
#
# Usage:
#   push_request.sh --title "PR title" [--body "PR body" | --body-file <path>]
#
# Must be run from within the git repo that has commits to push.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

# ---------- Argument parsing ----------

PR_TITLE=""
PR_BODY=""
PR_BODY_FILE=""

show_usage() {
    cat <<'EOF'
Usage: push_request.sh --title "PR title" [OPTIONS]

Signal that commits are ready for host-side push and PR creation.

Required:
  --title <text>        Pull request title

Options:
  --body <text>         Pull request body (short text)
  --body-file <path>    Path to file containing PR body (markdown)
  -h, --help            Show this help

The script writes a JSON signal file that the host reads after container exit.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --title)
            PR_TITLE="$2"; shift 2 ;;
        --body)
            PR_BODY="$2"; shift 2 ;;
        --body-file)
            PR_BODY_FILE="$2"; shift 2 ;;
        -h|--help)
            show_usage; exit 0 ;;
        *)
            echo "Unknown option: $1" >&2
            show_usage >&2
            exit 1 ;;
    esac
done

if [ -z "$PR_TITLE" ]; then
    echo "ERROR: --title is required." >&2
    show_usage >&2
    exit 1
fi

# ---------- Gather git info ----------

ISSUE="${WORKTREE_ISSUE:-}"
if [ -z "$ISSUE" ]; then
    # Try to extract from branch name (feature/issue-N)
    BRANCH="$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "")"
    ISSUE="$(echo "$BRANCH" | grep -oP 'issue-\K\d+' || echo "")"
fi

if [ -z "$ISSUE" ]; then
    echo "ERROR: Cannot determine issue number." >&2
    echo "Set WORKTREE_ISSUE or ensure branch name contains 'issue-<N>'." >&2
    exit 1
fi

BRANCH="$(git rev-parse --abbrev-ref HEAD)"
COMMIT_SHA="$(git rev-parse HEAD)"
REPO_PATH="$(git rev-parse --show-toplevel)"

# Derive repo slug from remote URL
REPO_SLUG="$(git remote get-url origin 2>/dev/null | sed -E 's|.*github\.com[:/]||;s|\.git$||' || echo "unknown")"

# Count unpushed commits (compare with upstream if tracking exists)
UPSTREAM="$(git rev-parse --abbrev-ref '@{upstream}' 2>/dev/null || echo "")"
if [ -n "$UPSTREAM" ]; then
    COMMIT_COUNT="$(git rev-list "$UPSTREAM..HEAD" --count 2>/dev/null || echo "0")"
else
    # No upstream — all commits on this branch diverged from default
    DEFAULT_BRANCH="$(git symbolic-ref refs/remotes/origin/HEAD 2>/dev/null | sed 's|refs/remotes/origin/||' || echo "main")"
    COMMIT_COUNT="$(git rev-list "origin/$DEFAULT_BRANCH..HEAD" --count 2>/dev/null || echo "0")"
fi

if [ "$COMMIT_COUNT" = "0" ]; then
    echo "WARNING: No unpushed commits found on branch '$BRANCH'." >&2
    echo "Make sure you have committed your changes before requesting a push." >&2
    exit 1
fi

# ---------- Resolve PR body ----------

if [ -n "$PR_BODY_FILE" ]; then
    if [ ! -f "$PR_BODY_FILE" ]; then
        echo "ERROR: Body file not found: $PR_BODY_FILE" >&2
        exit 1
    fi
    PR_BODY="$(cat "$PR_BODY_FILE")"
fi

# If no body provided, generate a minimal one
if [ -z "$PR_BODY" ]; then
    PR_BODY="Closes #$ISSUE

---
**Authored-By**: \`Claude Code Agent\`
**Model**: \`${AGENT_MODEL:-Claude Opus 4.6}\`"
fi

# ---------- Write signal file ----------

# Use WORKTREE_ID as the unique key (avoids collision when multiple repos
# share the same issue number).
WORKTREE_ID="${WORKTREE_ID:-}"
if [ -z "$WORKTREE_ID" ]; then
    echo "ERROR: WORKTREE_ID not set. This script must run inside a container launched by docker_run_agent.sh." >&2
    exit 1
fi

PUSH_DIR="$ROOT_DIR/.agent/scratchpad/push-requests"
mkdir -p "$PUSH_DIR"

SIGNAL_FILE="$PUSH_DIR/$WORKTREE_ID.json"
BODY_FILE="$PUSH_DIR/$WORKTREE_ID-body.md"

# Write PR body to separate file (avoids JSON escaping issues)
printf '%s\n' "$PR_BODY" > "$BODY_FILE"

# Write JSON signal
cat > "$SIGNAL_FILE" <<ENDJSON
{
  "worktree_id": "$WORKTREE_ID",
  "issue": $ISSUE,
  "branch": "$BRANCH",
  "repo_slug": "$REPO_SLUG",
  "repo_path": "$REPO_PATH",
  "commit_sha": "$COMMIT_SHA",
  "commit_count": $COMMIT_COUNT,
  "pr_title": $(printf '%s' "$PR_TITLE" | jq -Rs .),
  "pr_body_file": "$BODY_FILE",
  "requested_at": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
  "status": "pending"
}
ENDJSON

echo ""
echo "========================================="
echo "  Push Request Created"
echo "========================================="
echo "  Issue:   #$ISSUE"
echo "  Branch:  $BRANCH"
echo "  Commits: $COMMIT_COUNT"
echo "  Title:   $PR_TITLE"
echo "  Signal:  $SIGNAL_FILE"
echo "========================================="
echo ""
echo "Exit the container to trigger the push gateway on the host."
