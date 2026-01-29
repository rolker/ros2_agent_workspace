#!/bin/bash
# .agent/scripts/worktree_create.sh
# Create a git worktree for isolated task development
#
# Usage:
#   ./worktree_create.sh --issue <number> [--type layer|workspace] [--branch <name>]
#
# Examples:
#   ./worktree_create.sh --issue 123                    # Layer worktree (default)
#   ./worktree_create.sh --issue 123 --type workspace   # Workspace worktree
#   ./worktree_create.sh --issue 123 --branch feature/custom-name
#
# Worktree Types:
#   layer     - For ROS package development (default)
#               Created in: layers/worktrees/issue-<N>/
#               Includes isolated build/install/log per layer
#
#   workspace - For infrastructure work (.agent/, configs/, docs)
#               Created in: .workspace-worktrees/issue-<N>/
#               Full workspace checkout

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

# Defaults
ISSUE_NUM=""
WORKTREE_TYPE="layer"
BRANCH_NAME=""

show_usage() {
    echo "Usage: $0 --issue <number> [--type layer|workspace] [--branch <name>]"
    echo ""
    echo "Options:"
    echo "  --issue <number>    Issue number (required)"
    echo "  --type <type>       Worktree type: 'layer' (default) or 'workspace'"
    echo "  --branch <name>     Custom branch name (default: feature/issue-<N>)"
    echo ""
    echo "Examples:"
    echo "  $0 --issue 123"
    echo "  $0 --issue 123 --type workspace"
    echo "  $0 --issue 123 --branch feature/add-new-sensor"
}

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --issue)
            ISSUE_NUM="$2"
            shift 2
            ;;
        --type)
            WORKTREE_TYPE="$2"
            shift 2
            ;;
        --branch)
            BRANCH_NAME="$2"
            shift 2
            ;;
        -h|--help)
            show_usage
            exit 0
            ;;
        *)
            echo "Error: Unknown option $1"
            show_usage
            exit 1
            ;;
    esac
done

# Validate required arguments
if [ -z "$ISSUE_NUM" ]; then
    echo "Error: --issue is required"
    show_usage
    exit 1
fi

# Validate worktree type
if [ "$WORKTREE_TYPE" != "layer" ] && [ "$WORKTREE_TYPE" != "workspace" ]; then
    echo "Error: --type must be 'layer' or 'workspace'"
    exit 1
fi

# Set default branch name if not provided
if [ -z "$BRANCH_NAME" ]; then
    BRANCH_NAME="feature/issue-${ISSUE_NUM}"
fi

# Determine worktree path based on type
if [ "$WORKTREE_TYPE" == "layer" ]; then
    WORKTREE_DIR="$ROOT_DIR/layers/worktrees/issue-${ISSUE_NUM}"
else
    WORKTREE_DIR="$ROOT_DIR/.workspace-worktrees/issue-${ISSUE_NUM}"
fi

# Check if worktree already exists
if [ -d "$WORKTREE_DIR" ]; then
    echo "Error: Worktree already exists at $WORKTREE_DIR"
    echo "Use 'worktree_enter.sh --issue $ISSUE_NUM' to enter it"
    echo "Or 'worktree_remove.sh --issue $ISSUE_NUM' to remove it"
    exit 1
fi

# Check if branch already exists
cd "$ROOT_DIR"
if git show-ref --verify --quiet "refs/heads/$BRANCH_NAME"; then
    echo "Branch '$BRANCH_NAME' already exists."
    BRANCH_EXISTS=true
else
    BRANCH_EXISTS=false
fi

echo "========================================"
echo "Creating Worktree"
echo "========================================"
echo "  Issue:      #$ISSUE_NUM"
echo "  Type:       $WORKTREE_TYPE"
echo "  Branch:     $BRANCH_NAME"
echo "  Path:       $WORKTREE_DIR"
echo ""

# Create parent directory if needed
mkdir -p "$(dirname "$WORKTREE_DIR")"

# Create the worktree
if [ "$BRANCH_EXISTS" = true ]; then
    echo "Using existing branch '$BRANCH_NAME'..."
    git worktree add "$WORKTREE_DIR" "$BRANCH_NAME"
else
    echo "Creating new branch '$BRANCH_NAME' from current HEAD..."
    git worktree add -b "$BRANCH_NAME" "$WORKTREE_DIR"
fi

# For layer worktrees, set up the layer structure
if [ "$WORKTREE_TYPE" == "layer" ]; then
    echo ""
    echo "Setting up layer worktree structure..."
    
    # Create scratchpad for this worktree
    mkdir -p "$WORKTREE_DIR/.scratchpad"
    echo "# Task Scratchpad for Issue #$ISSUE_NUM" > "$WORKTREE_DIR/.scratchpad/README.md"
    echo "" >> "$WORKTREE_DIR/.scratchpad/README.md"
    echo "This directory is for temporary files specific to this task." >> "$WORKTREE_DIR/.scratchpad/README.md"
    echo "Files here are gitignored and isolated from other worktrees." >> "$WORKTREE_DIR/.scratchpad/README.md"
    
    # Note: build/install/log dirs will be created automatically by colcon
    # when building within each layer's workspace
fi

# For workspace worktrees, create scratchpad as well
if [ "$WORKTREE_TYPE" == "workspace" ]; then
    mkdir -p "$WORKTREE_DIR/.agent/scratchpad"
fi

echo ""
echo "========================================"
echo "✅ Worktree Created Successfully"
echo "========================================"
echo ""
echo "To enter this worktree:"
echo "  source $SCRIPT_DIR/worktree_enter.sh --issue $ISSUE_NUM"
echo ""
echo "Or manually:"
echo "  cd $WORKTREE_DIR"
if [ "$WORKTREE_TYPE" == "layer" ]; then
    echo "  source .agent/scripts/env.sh"
fi
echo ""
echo "When done, remove with:"
echo "  $SCRIPT_DIR/worktree_remove.sh --issue $ISSUE_NUM"
