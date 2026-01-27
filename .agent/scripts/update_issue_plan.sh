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

PLAN_FILE=".agent/work-plans/PLAN_ISSUE-${ISSUE_NUMBER}.md"

if [ ! -f "$PLAN_FILE" ]; then
    echo "❌ Plan file not found: $PLAN_FILE"
    echo "   Run start_issue_work.sh first to create the plan."
    exit 1
fi

# Check if plan was modified
if ! git diff --quiet "$PLAN_FILE"; then
    echo "📝 Changes detected in $PLAN_FILE"
    
    # Show diff summary
    echo ""
    echo "Changes:"
    git diff --stat "$PLAN_FILE"
    echo ""
    
    # Commit the update
    git add "$PLAN_FILE"
    git commit -m "docs: $COMMIT_MSG"
    
    echo "✅ Plan updated and committed!"
    echo ""
    echo "🚀 Push to GitHub: git push"
else
    echo "ℹ️  No changes to plan file"
fi
