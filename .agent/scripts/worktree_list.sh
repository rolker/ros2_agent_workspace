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

source "$SCRIPT_DIR/_worktree_helpers.sh"

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

# Get worktree list from git (finds main workspace + workspace worktrees)
WORKTREES=$(git worktree list --porcelain)

# Count worktrees (excluding main)
LAYER_COUNT=0
WORKSPACE_COUNT=0

# Helper: extract issue/repo from worktree directory basename
# Sets variables: WT_ISSUE, WT_REPO
extract_issue_repo() {
    local basename="$1"
    WT_ISSUE=""
    WT_REPO=""

    # New format: issue-{REPO_SLUG}-{NUMBER}
    if [[ "$basename" =~ ^issue-([a-zA-Z0-9_]+)-([0-9]+)$ ]]; then
        WT_REPO="${BASH_REMATCH[1]}"
        WT_ISSUE="${BASH_REMATCH[2]}"
    # Legacy format: issue-{NUMBER}
    elif [[ "$basename" =~ ^issue-([0-9]+)$ ]]; then
        WT_ISSUE="${BASH_REMATCH[1]}"
        WT_REPO="(legacy)"
    fi
}

# Function to print a worktree entry (for main and workspace types from git worktree list)
print_worktree() {
    local path="$1"
    local branch="$2"
    local head="$3"

    # Determine type and extract issue/repo info
    local type="main"
    local issue=""
    local repo=""

    if [[ "$path" == *"/.workspace-worktrees/"* ]]; then
        type="workspace"
        extract_issue_repo "$(basename "$path")"
        issue="$WT_ISSUE"
        repo="$WT_REPO"
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
        echo "🔧 Issue #$issue ($type) - Repository: $repo"
        echo "   Path:   $path"
        echo "   Branch: ${branch:-detached at $head}"
        echo "   Status: $status"

        if [ "$VERBOSE" = true ] && [ -d "$path" ]; then
            echo "   Files changed: $(git -C "$path" status --porcelain 2>/dev/null | wc -l)"
        fi
    fi
    echo ""
}

# Parse and display worktrees from git worktree list (main + workspace types)
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

# Print last worktree from git list
if [ -n "$CURRENT_PATH" ]; then
    print_worktree "$CURRENT_PATH" "$CURRENT_BRANCH" "$CURRENT_HEAD"
fi

# Discover layer worktrees by scanning the directory
# Layer worktrees are plain directories (not git worktrees) after the fix for #193
LAYER_WT_DIR="$ROOT_DIR/layers/worktrees"
if [ -d "$LAYER_WT_DIR" ]; then
    for layer_wt in "$LAYER_WT_DIR"/issue-*; do
        [ -d "$layer_wt" ] || continue

        extract_issue_repo "$(basename "$layer_wt")"
        local_issue="$WT_ISSUE"
        local_repo="$WT_REPO"

        # Get branch from inner package worktree
        local_branch=$(wt_layer_branch "$layer_wt" 2>/dev/null || echo "")

        # Check dirty status
        local_status="clean"
        if wt_layer_is_dirty "$layer_wt"; then
            local_status="dirty"
        fi

        echo "📦 Issue #$local_issue (layer) - Repository: $local_repo"
        echo "   Path:   $layer_wt"
        echo "   Branch: ${local_branch:-unknown}"
        echo "   Status: $local_status"

        if [ "$VERBOSE" = true ]; then
            # Count changed files across all inner package worktrees
            local_changed=0
            for ws_dir in "$layer_wt"/*_ws; do
                [ -d "$ws_dir" ] || continue
                [ -L "$ws_dir" ] && continue
                src_dir="$ws_dir/src"
                [ -d "$src_dir" ] || continue
                for pkg_dir in "$src_dir"/*; do
                    [ -d "$pkg_dir" ] || continue
                    [ -L "$pkg_dir" ] && continue
                    if git -C "$pkg_dir" rev-parse --git-dir &>/dev/null; then
                        pkg_changed=$(git -C "$pkg_dir" status --porcelain 2>/dev/null | wc -l)
                        local_changed=$((local_changed + pkg_changed))
                    fi
                done
            done
            echo "   Files changed: $local_changed"
        fi

        echo ""
        ((LAYER_COUNT++)) || true
    done
fi

# If no worktrees found at all, show help
if [ -z "$WORKTREES" ] && [ "$LAYER_COUNT" -eq 0 ]; then
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
