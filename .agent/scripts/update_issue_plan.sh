#!/bin/bash
# Update and commit work plan for an issue
# Usage: update_issue_plan.sh <issue_number> [commit_message]

set -e

ISSUE_NUMBER="$1"
COMMIT_MSG="${2:-Update work plan for Issue #$ISSUE_NUMBER}"

if [ -z "$ISSUE_NUMBER" ]; then
    echo "Usage: $0 <issue_number> [commit_message]"
    echo "Example: $0 42 'Complete Phase 1 tasks'"
    exit 1
fi

# Get repository root and change to it
REPO_ROOT="$(git rev-parse --show-toplevel)"
cd "$REPO_ROOT"

PLAN_FILE=".agent/work-plans/PLAN_ISSUE-${ISSUE_NUMBER}.md"

if [ ! -f "$PLAN_FILE" ]; then
    echo "❌ Plan file not found: $PLAN_FILE"
    echo "   Run start_issue_work.sh first to create the plan."
    exit 1
fi

# Check if plan file has any changes (including untracked)
if [[ -n "$(git status --porcelain -- "$PLAN_FILE")" ]]; then
    echo "📝 Changes detected in $PLAN_FILE"

    # Show diff summary
    echo ""
    echo "Changes:"
    # Staged changes (if any)
    git diff --stat --cached -- "$PLAN_FILE" 2>/dev/null || true
    # Unstaged changes (if any)
    git diff --stat -- "$PLAN_FILE" 2>/dev/null || true
    echo ""

    # Normalize commit message: only prepend 'docs:' if no conventional prefix is present
    # shellcheck disable=SC1073,SC1072
    if [[ "$COMMIT_MSG" =~ ^[A-Za-z]+(\([^)]*\))?:[[:space:]] ]]; then
        NORMALIZED_COMMIT_MSG="$COMMIT_MSG"
    else
        NORMALIZED_COMMIT_MSG="docs: $COMMIT_MSG"
    fi

    # Commit the update
    git add "$PLAN_FILE"
    git commit -m "$NORMALIZED_COMMIT_MSG"

    echo "✅ Plan updated and committed!"
    echo ""
    echo "🚀 Push to GitHub: git push"
else
    echo "ℹ️  No changes to plan file"
fi
