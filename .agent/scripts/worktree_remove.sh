#!/bin/bash
# .agent/scripts/worktree_remove.sh
# Remove a git worktree and clean up
#
# Usage:
#   ./worktree_remove.sh --issue <number> [--repo-slug <slug>] [--force]
#
# Examples:
#   ./worktree_remove.sh --issue 123
#   ./worktree_remove.sh --issue 123 --force
#   ./worktree_remove.sh --issue 5 --repo-slug marine_msgs
#
# This will:
#   1. Check for uncommitted changes (unless --force)
#   2. Remove the worktree directory
#   3. Prune the git worktree reference
#   4. Optionally delete the branch (prompts user)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

ISSUE_NUM=""
FORCE=false
REPO_SLUG=""

show_usage() {
    echo "Usage: $0 --issue <number> [--repo-slug <slug>] [--force]"
    echo "   or: $0 <number> [--force]"
    echo ""
    echo "Options:"
    echo "  --issue <number>        Issue number (required)"
    echo "  <number>                Issue number as positional argument"
    echo "  --repo-slug <slug>      Repository slug (optional, for disambiguation)"
    echo "  --force                 Force removal even with uncommitted changes"
    echo ""
    echo "Examples:"
    echo "  $0 --issue 123"
    echo "  $0 123"
    echo "  $0 123 --force"
    echo "  $0 --issue 5 --repo-slug marine_msgs"
}

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --issue)
            ISSUE_NUM="$2"
            shift 2
            ;;
        --repo-slug)
            REPO_SLUG="$2"
            shift 2
            ;;
        --force|-f)
            FORCE=true
            shift
            ;;
        -h|--help)
            show_usage
            exit 0
            ;;
        [0-9]*)
            # Positional argument (issue number without --issue flag)
            ISSUE_NUM="$1"
            shift
            ;;
        *)
            echo "Error: Unknown option $1"
            show_usage
            exit 1
            ;;
    esac
done

# Sanitize repo slug: replace non-alphanumeric characters (except underscore) with underscores
if [ -n "$REPO_SLUG" ]; then
    REPO_SLUG=$(echo "$REPO_SLUG" | sed 's/[^A-Za-z0-9_]/_/g')
fi

if [ -z "$ISSUE_NUM" ]; then
    echo "Error: Issue number is required"
    show_usage
    exit 1
fi

# Function to find worktree directory
find_worktree() {
    local base_dir="$1"
    local issue_num="$2"
    local repo_slug="$3"
    
    # If repo slug is specified, check exact path
    if [ -n "$repo_slug" ]; then
        local exact_path="$base_dir/issue-${repo_slug}-${issue_num}"
        if [ -d "$exact_path" ]; then
            echo "$exact_path"
            return 0
        fi
        return 1
    fi
    
    # Otherwise, search for matching worktrees (new format and legacy)
    local matches=()
    for path in "$base_dir"/issue-*-"${issue_num}"; do
        if [ -d "$path" ] && [ "$path" != "$base_dir/issue-*-${issue_num}" ]; then
            matches+=( "$path" )
        fi
    done

    # Also check legacy format: issue-{NUMBER}
    local legacy_path="$base_dir/issue-${issue_num}"
    if [ -d "$legacy_path" ]; then
        matches+=( "$legacy_path" )
    fi

    if [ "${#matches[@]}" -eq 1 ]; then
        echo "${matches[0]}"
        return 0
    elif [ "${#matches[@]}" -gt 1 ]; then
        echo "Error: Multiple worktrees found for issue ${issue_num}:" >&2
        for path in "${matches[@]}"; do
            echo "  - $(basename "$path")" >&2
        done
        echo "" >&2
        echo "Use --repo-slug to specify which one:" >&2
        for path in "${matches[@]}"; do
            local slug=$(basename "$path" | sed -E 's/^issue-(.+)-[0-9]+$/\1/')
            echo "  $0 --issue ${issue_num} --repo-slug ${slug}" >&2
        done
        return 1
    fi

    return 1
}

WORKTREE_DIR=""
WORKTREE_TYPE=""

# Check layer worktrees
if FOUND=$(find_worktree "$ROOT_DIR/layers/worktrees" "$ISSUE_NUM" "$REPO_SLUG"); then
    WORKTREE_DIR="$FOUND"
    WORKTREE_TYPE="layer"
# Check workspace worktrees
elif FOUND=$(find_worktree "$ROOT_DIR/.workspace-worktrees" "$ISSUE_NUM" "$REPO_SLUG"); then
    WORKTREE_DIR="$FOUND"
    WORKTREE_TYPE="workspace"
else
    echo "Error: No worktree found for issue #$ISSUE_NUM"
    echo ""
    echo "Checked locations:"
    if [ -n "$REPO_SLUG" ]; then
        echo "  - $ROOT_DIR/layers/worktrees/issue-${REPO_SLUG}-$ISSUE_NUM"
        echo "  - $ROOT_DIR/.workspace-worktrees/issue-${REPO_SLUG}-$ISSUE_NUM"
    else
        echo "  - $ROOT_DIR/layers/worktrees/issue-*-$ISSUE_NUM"
        echo "  - $ROOT_DIR/.workspace-worktrees/issue-*-$ISSUE_NUM"
    fi
    echo ""
    echo "List worktrees with:"
    echo "  ./.agent/scripts/worktree_list.sh"
    exit 1
fi

# Get branch name
cd "$ROOT_DIR"
BRANCH_NAME=$(git -C "$WORKTREE_DIR" branch --show-current 2>/dev/null || echo "")

echo "========================================"
echo "Removing Worktree"
echo "========================================"
echo "  Issue:  #$ISSUE_NUM"
echo "  Type:   $WORKTREE_TYPE"
echo "  Path:   $WORKTREE_DIR"
echo "  Branch: ${BRANCH_NAME:-detached HEAD}"
echo ""

# Check for uncommitted changes
if [ -d "$WORKTREE_DIR" ]; then
    cd "$WORKTREE_DIR"
    UNCOMMITTED=$(git status --porcelain 2>/dev/null)
    
    if [ -n "$UNCOMMITTED" ] && [ "$FORCE" != true ]; then
        echo "⚠️  Warning: Worktree has uncommitted changes:"
        echo ""
        git status --short
        echo ""
        echo "Use --force to remove anyway, or commit/stash your changes first."
        exit 1
    fi
    
    cd "$ROOT_DIR"
fi

# Check if we're currently in the worktree we're trying to remove
if [[ "$(pwd)" == "$WORKTREE_DIR"* ]]; then
    echo "⚠️  Warning: You are currently inside this worktree."
    echo "   Changing to workspace root first..."
    cd "$ROOT_DIR"
fi

# Remove the worktree
echo "Removing worktree..."
git worktree remove "$WORKTREE_DIR" ${FORCE:+--force}

# Prune worktree references
git worktree prune

echo ""
echo "✅ Worktree removed successfully"

# Offer to delete the branch
if [ -n "$BRANCH_NAME" ]; then
    echo ""
    # Check if branch still exists
    if git show-ref --verify --quiet "refs/heads/$BRANCH_NAME"; then
        echo "The branch '$BRANCH_NAME' still exists."
        echo ""
        echo "To delete it locally:"
        echo "  git branch -d $BRANCH_NAME"
        echo ""
        echo "To delete it on origin (if pushed):"
        echo "  git push origin --delete $BRANCH_NAME"
    fi
fi

echo ""
echo "Remaining worktrees:"
"$SCRIPT_DIR/worktree_list.sh" 2>/dev/null || git worktree list
