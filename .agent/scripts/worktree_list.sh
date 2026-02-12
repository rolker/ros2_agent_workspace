#!/bin/bash
# .agent/scripts/worktree_list.sh
# List all git worktrees for this workspace
#
# Usage:
#   ./worktree_list.sh [--verbose]
#
# Shows all active worktrees including:
#   - Issue number
#   - Type (layer/workspace)
#   - Branch name
#   - Path
#   - Status (clean/dirty)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

VERBOSE=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -v|--verbose)
            VERBOSE=true
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [--verbose]"
            echo ""
            echo "Options:"
            echo "  -v, --verbose    Show detailed status for each worktree"
            exit 0
            ;;
        *)
            echo "Error: Unknown option $1"
            exit 1
            ;;
    esac
done

cd "$ROOT_DIR"

echo "========================================"
echo "Git Worktrees"
echo "========================================"
echo ""

# Get worktree list from git
WORKTREES=$(git worktree list --porcelain)

# Count worktrees (excluding main)
LAYER_COUNT=0
WORKSPACE_COUNT=0

# Function to print a worktree entry
print_worktree() {
    local path="$1"
    local branch="$2"
    local head="$3"

    # Determine type and extract issue/repo info
    local type="main"
    local issue=""
    local repo=""

    if [[ "$path" == *"/layers/worktrees/"* ]]; then
        type="layer"
        local basename
        basename=$(basename "$path")
        # New format: issue-{REPO_SLUG}-{NUMBER}
        # Note: REPO_SLUG is sanitized to [A-Za-z0-9_] (hyphens replaced with underscores)
        if [[ "$basename" =~ ^issue-([a-zA-Z0-9_]+)-([0-9]+)$ ]]; then
            repo="${BASH_REMATCH[1]}"
            issue="${BASH_REMATCH[2]}"
        # Legacy format: issue-{NUMBER}
        elif [[ "$basename" =~ ^issue-([0-9]+)$ ]]; then
            issue="${BASH_REMATCH[1]}"
            repo="(legacy)"
        fi
        ((LAYER_COUNT++)) || true
    elif [[ "$path" == *"/.workspace-worktrees/"* ]]; then
        type="workspace"
        local basename
        basename=$(basename "$path")
        # New format: issue-{REPO_SLUG}-{NUMBER}
        # Note: REPO_SLUG is sanitized to [A-Za-z0-9_] (hyphens replaced with underscores)
        if [[ "$basename" =~ ^issue-([a-zA-Z0-9_]+)-([0-9]+)$ ]]; then
            repo="${BASH_REMATCH[1]}"
            issue="${BASH_REMATCH[2]}"
        # Legacy format: issue-{NUMBER}
        elif [[ "$basename" =~ ^issue-([0-9]+)$ ]]; then
            issue="${BASH_REMATCH[1]}"
            repo="(legacy)"
        fi
        ((WORKSPACE_COUNT++)) || true
    fi

    # Check if clean or dirty
    local status="clean"
    if [ -d "$path" ]; then
        if [ -n "$(git -C "$path" status --porcelain 2>/dev/null)" ]; then
            status="dirty"
        fi
    fi

    # Format output
    if [ "$type" == "main" ]; then
        echo "📁 Main Workspace"
        echo "   Path:   $path"
        echo "   Branch: ${branch:-detached at $head}"
        echo "   Status: $status"
    else
        local icon="📦"
        [ "$type" == "workspace" ] && icon="🔧"

        echo "$icon Issue #$issue ($type) - Repository: $repo"
        echo "   Path:   $path"
        echo "   Branch: ${branch:-detached at $head}"
        echo "   Status: $status"

        if [ "$VERBOSE" = true ] && [ -d "$path" ]; then
            echo "   Files changed: $(git -C "$path" status --porcelain 2>/dev/null | wc -l)"
        fi
    fi
    echo ""
}

# Parse and display worktrees
CURRENT_PATH=""
CURRENT_BRANCH=""
CURRENT_HEAD=""

while IFS= read -r line || [ -n "$line" ]; do
    if [[ $line == worktree* ]]; then
        # If we have a previous worktree, print it
        if [ -n "$CURRENT_PATH" ]; then
            print_worktree "$CURRENT_PATH" "$CURRENT_BRANCH" "$CURRENT_HEAD"
        fi
        CURRENT_PATH="${line#worktree }"
        CURRENT_BRANCH=""
        CURRENT_HEAD=""
    elif [[ $line == HEAD* ]]; then
        CURRENT_HEAD="${line#HEAD }"
    elif [[ $line == branch* ]]; then
        CURRENT_BRANCH="${line#branch refs/heads/}"
    fi
done <<< "$WORKTREES"

# Print last worktree
if [ -n "$CURRENT_PATH" ]; then
    print_worktree "$CURRENT_PATH" "$CURRENT_BRANCH" "$CURRENT_HEAD"
fi

# If no worktrees found, show basic git worktree list
if [ -z "$WORKTREES" ]; then
    echo "No worktrees found."
    echo ""
    echo "Create one with:"
    echo "  ./.agent/scripts/worktree_create.sh --issue <number>"
    exit 0
fi

echo "========================================"
echo "Summary"
echo "========================================"
echo "  Layer worktrees:     $LAYER_COUNT"
echo "  Workspace worktrees: $WORKSPACE_COUNT"
echo ""

# Show locations
if [ $LAYER_COUNT -gt 0 ] || [ -d "$ROOT_DIR/layers/worktrees" ]; then
    echo "Layer worktrees location:     layers/worktrees/"
fi
if [ $WORKSPACE_COUNT -gt 0 ] || [ -d "$ROOT_DIR/.workspace-worktrees" ]; then
    echo "Workspace worktrees location: .workspace-worktrees/"
fi
