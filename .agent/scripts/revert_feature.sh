#!/bin/bash
# Revert all commits associated with a GitHub issue/feature
#
# This script finds all commits that reference a specific GitHub issue
# and creates revert commits for them (in reverse chronological order).
#
# Usage: revert_feature.sh --issue <number> [--dry-run]

set -e
set -o pipefail

usage() {
    echo "Usage: revert_feature.sh --issue <number> [--dry-run]"
    echo ""
    echo "Reverts all commits that reference the given issue number."
    echo ""
    echo "Options:"
    echo "  --issue <number>  GitHub issue number to revert"
    echo "  --dry-run         Show what would be reverted without doing it"
    echo ""
    echo "Examples:"
    echo "  # Preview what would be reverted"
    echo "  revert_feature.sh --issue 137 --dry-run"
    echo ""
    echo "  # Actually revert the feature"
    echo "  revert_feature.sh --issue 137"
    exit 1
}

# Parse arguments
ISSUE=""
DRY_RUN=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --issue)
            ISSUE="$2"
            shift 2
            ;;
        --dry-run)
            DRY_RUN=true
            shift
            ;;
        -h|--help)
            usage
            ;;
        *)
            echo "Error: Unknown option: $1"
            usage
            ;;
    esac
done

if [ -z "$ISSUE" ]; then
    echo "Error: --issue <number> is required"
    usage
fi

# Validate issue number is numeric
if ! [[ "$ISSUE" =~ ^[0-9]+$ ]]; then
    echo "Error: Issue number must be numeric, got: $ISSUE"
    exit 1
fi

echo "========================================="
echo "Feature Revert Tool"
echo "========================================="
echo "Issue: #$ISSUE"
if [ "$DRY_RUN" = true ]; then
    echo "Mode: DRY RUN (no changes will be made)"
else
    echo "Mode: LIVE (commits will be reverted)"
fi
echo ""

# Find commits mentioning this issue (search for #ISSUE, Closes #ISSUE, Fixes #ISSUE, etc.)
echo "Searching for commits referencing issue #$ISSUE..."
COMMITS=$(git log --grep="#$ISSUE" --format="%H %s" --reverse 2>/dev/null || true)

if [ -z "$COMMITS" ]; then
    echo ""
    echo "❌ No commits found referencing issue #$ISSUE"
    echo ""
    echo "Searched patterns:"
    echo "  - #$ISSUE"
    echo "  - Closes #$ISSUE"
    echo "  - Fixes #$ISSUE"
    echo "  - Resolves #$ISSUE"
    exit 1
fi

# Count commits
COMMIT_COUNT=$(echo "$COMMITS" | wc -l)

echo ""
echo "Found $COMMIT_COUNT commit(s) to revert:"
echo "========================================="
echo "$COMMITS"
echo "========================================="
echo ""

if [ "$DRY_RUN" = true ]; then
    echo "[DRY RUN] Would revert these commits (newest to oldest)"
    echo ""
    echo "To actually revert, run without --dry-run:"
    echo "  .agent/scripts/revert_feature.sh --issue $ISSUE"
    exit 0
fi

# Confirm with user
echo "⚠️  WARNING: This will create $COMMIT_COUNT revert commit(s)"
echo ""
read -p "Continue? (y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Aborted."
    exit 1
fi

echo ""
echo "Reverting commits (newest to oldest)..."
echo "========================================="

# Revert in reverse order (newest first to avoid conflicts)
# Use mapfile to avoid subshell issues with pipe
mapfile -t COMMIT_HASHES < <(git log --grep="#$ISSUE" --format=%H --reverse | tac)

REVERTED=0

for commit in "${COMMIT_HASHES[@]}"; do
    COMMIT_MSG=$(git log --format=%s -n 1 "$commit")
    echo ""
    echo "Reverting: $commit"
    echo "  Message: $COMMIT_MSG"

    if git revert --no-edit "$commit" > /dev/null 2>&1; then
        echo "  ✅ Reverted successfully"
        REVERTED=$((REVERTED + 1))
    else
        echo "  ❌ Conflict during revert"
        echo ""
        echo "========================================="
        echo "⚠️  CONFLICT DETECTED"
        echo "========================================="
        echo ""
        echo "A conflict occurred while reverting commit $commit"
        echo ""
        echo "To resolve:"
        echo "  1. Fix conflicts in the affected files"
        echo "  2. Run: git add <files>"
        echo "  3. Run: git revert --continue"
        echo ""
        echo "To abort:"
        echo "  Run: git revert --abort"
        echo ""
        exit 1
    fi
done

# Report success
echo ""
echo "========================================="
echo "✅ Successfully reverted $REVERTED commit(s) for issue #$ISSUE"
echo "========================================="
echo ""
echo "Next steps:"
echo "  1. Review the revert commits: git log"
echo "  2. Push to remote: git push"
echo "  3. Close or update issue #$ISSUE on GitHub"
