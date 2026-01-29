#!/bin/bash
# .agent/scripts/worktree_enter.sh
# Enter a worktree and set up the environment
#
# Usage:
#   source ./worktree_enter.sh --issue <number>
#
# This script should be SOURCED (not executed) to affect the current shell.
# It will:
#   1. Change to the worktree directory
#   2. Source the ROS 2 environment (for layer worktrees)
#   3. Set helpful environment variables
#
# Examples:
#   source ./.agent/scripts/worktree_enter.sh --issue 123

# Don't use set -e since we're sourced
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

ISSUE_NUM=""

show_usage() {
    echo "Usage: source $0 --issue <number>"
    echo ""
    echo "Options:"
    echo "  --issue <number>    Issue number (required)"
    echo ""
    echo "Note: This script must be SOURCED to affect your current shell."
    echo ""
    echo "Examples:"
    echo "  source $0 --issue 123"
}

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --issue)
            ISSUE_NUM="$2"
            shift 2
            ;;
        -h|--help)
            show_usage
            return 0 2>/dev/null || exit 0
            ;;
        *)
            echo "Error: Unknown option $1"
            show_usage
            return 1 2>/dev/null || exit 1
            ;;
    esac
done

if [ -z "$ISSUE_NUM" ]; then
    echo "Error: --issue is required"
    show_usage
    return 1 2>/dev/null || exit 1
fi

# Find the worktree - check both locations
LAYER_WORKTREE="$ROOT_DIR/layers/worktrees/issue-${ISSUE_NUM}"
WORKSPACE_WORKTREE="$ROOT_DIR/.workspace-worktrees/issue-${ISSUE_NUM}"

WORKTREE_DIR=""
WORKTREE_TYPE=""

if [ -d "$LAYER_WORKTREE" ]; then
    WORKTREE_DIR="$LAYER_WORKTREE"
    WORKTREE_TYPE="layer"
elif [ -d "$WORKSPACE_WORKTREE" ]; then
    WORKTREE_DIR="$WORKSPACE_WORKTREE"
    WORKTREE_TYPE="workspace"
else
    echo "Error: No worktree found for issue #$ISSUE_NUM"
    echo ""
    echo "Checked locations:"
    echo "  - $LAYER_WORKTREE"
    echo "  - $WORKSPACE_WORKTREE"
    echo ""
    echo "Create one with:"
    echo "  ./.agent/scripts/worktree_create.sh --issue $ISSUE_NUM"
    return 1 2>/dev/null || exit 1
fi

echo "========================================"
echo "Entering Worktree"
echo "========================================"
echo "  Issue:  #$ISSUE_NUM"
echo "  Type:   $WORKTREE_TYPE"
echo "  Path:   $WORKTREE_DIR"
echo ""

# Change to worktree directory
cd "$WORKTREE_DIR" || { echo "Error: Failed to cd to $WORKTREE_DIR"; return 1 2>/dev/null || exit 1; }

# Set environment variables
export WORKTREE_ISSUE="$ISSUE_NUM"
export WORKTREE_TYPE="$WORKTREE_TYPE"
export WORKTREE_ROOT="$WORKTREE_DIR"

# For layer worktrees, also set up scratchpad path
if [ "$WORKTREE_TYPE" == "layer" ]; then
    export WORKTREE_SCRATCHPAD="$WORKTREE_DIR/.scratchpad"
    
    # Source ROS 2 environment if available
    if [ -f "$WORKTREE_DIR/.agent/scripts/env.sh" ]; then
        echo "Sourcing ROS 2 environment..."
        source "$WORKTREE_DIR/.agent/scripts/env.sh"
    fi
fi

# Show current branch
CURRENT_BRANCH=$(git branch --show-current 2>/dev/null)
echo ""
echo "✅ Now in worktree for issue #$ISSUE_NUM"
echo "   Branch: $CURRENT_BRANCH"
echo "   PWD:    $(pwd)"
echo ""

# Show helpful commands
echo "Helpful commands:"
echo "  git status                    # Check changes"
echo "  git diff                      # See what changed"
if [ "$WORKTREE_TYPE" == "layer" ]; then
    echo "  colcon build                  # Build packages"
    echo "  colcon test                   # Run tests"
fi
echo "  ./.agent/scripts/worktree_remove.sh --issue $ISSUE_NUM  # Remove worktree"
