#!/bin/bash
# .agent/scripts/worktree_create.sh
# Create a git worktree for isolated task development
#
# Usage:
#   ./worktree_create.sh --issue <number> [--type layer|workspace] [--branch <name>] [--layer <layer_name>]
#
# Examples:
#   ./worktree_create.sh --issue 123 --type layer --layer core    # Work on core layer
#   ./worktree_create.sh --issue 123 --type workspace              # Workspace worktree
#   ./worktree_create.sh --issue 123 --type layer --layer core --branch feature/custom-name
#
# Worktree Types:
#   layer     - For ROS package development
#               Created in: layers/worktrees/issue-<N>/
#               Contains target layer (real) + symlinks to main for others
#               Requires --layer to specify which layer to work on
#
#   workspace - For infrastructure work (.agent/, configs/, docs)
#               Created in: .workspace-worktrees/issue-<N>/
#               Full workspace checkout with symlinked layers/

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

# Defaults
ISSUE_NUM=""
WORKTREE_TYPE="layer"
BRANCH_NAME=""
TARGET_LAYER=""
REPO_SLUG=""

# Available layers (same order as env.sh)
AVAILABLE_LAYERS=("underlay" "core" "platforms" "sensors" "simulation" "ui")

show_usage() {
    echo "Usage: $0 --issue <number> --type <layer|workspace> [options]"
    echo ""
    echo "Options:"
    echo "  --issue <number>    Issue number (required)"
    echo "  --type <type>       Worktree type: 'layer' or 'workspace' (required)"
    echo "  --layer <name>      Layer to work on (required for layer type)"
    echo "                      Available: ${AVAILABLE_LAYERS[*]}"
    echo "  --repo-slug <slug>  Repository slug for naming (auto-detected if not provided)"
    echo "  --branch <name>     Custom branch name (default: feature/issue-<N>)"
    echo ""
    echo "Examples:"
    echo "  $0 --issue 123 --type layer --layer core"
    echo "  $0 --issue 123 --type workspace --repo-slug workspace"
    echo "  $0 --issue 5 --type layer --layer sensors --repo-slug marine_msgs"
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
        --layer)
            TARGET_LAYER="$2"
            shift 2
            ;;
        --repo-slug)
            REPO_SLUG="$2"
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

# For layer worktrees, require --layer
if [ "$WORKTREE_TYPE" == "layer" ] && [ -z "$TARGET_LAYER" ]; then
    echo "Error: --layer is required for layer worktrees"
    echo "Available layers: ${AVAILABLE_LAYERS[*]}"
    exit 1
fi

# Validate layer name if provided
if [ -n "$TARGET_LAYER" ]; then
    VALID_LAYER=false
    for layer in "${AVAILABLE_LAYERS[@]}"; do
        if [ "$layer" == "$TARGET_LAYER" ]; then
            VALID_LAYER=true
            break
        fi
    done
    if [ "$VALID_LAYER" = false ]; then
        echo "Error: Invalid layer '$TARGET_LAYER'"
        echo "Available layers: ${AVAILABLE_LAYERS[*]}"
        exit 1
    fi
fi

# Auto-detect repo slug if not provided
if [ -z "$REPO_SLUG" ]; then
    # Try to detect from git remote URL
    if git remote get-url origin &>/dev/null; then
        REMOTE_URL=$(git remote get-url origin)
        # Extract repo name from URL (works for both HTTPS and SSH)
        # e.g., https://github.com/org/repo.git -> repo
        # e.g., git@github.com:org/repo.git -> repo
        REPO_SLUG=$(basename "$REMOTE_URL" .git)

        # If this is the main workspace repo, use "workspace" as the slug
        if [ "$REPO_SLUG" == "ros2_agent_workspace" ]; then
            REPO_SLUG="workspace"
        fi

        # Sanitize repo slug: replace hyphens and other invalid characters with underscores
        REPO_SLUG=$(echo "$REPO_SLUG" | sed 's/[^A-Za-z0-9_]/_/g')
    else
        # Fallback to "workspace" if no remote
        REPO_SLUG="workspace"
    fi
    echo "Auto-detected repository slug: $REPO_SLUG"
else
    # Sanitize explicitly provided repo slug using the same rules
    REPO_SLUG=$(echo "$REPO_SLUG" | sed 's/[^A-Za-z0-9_]/_/g')
fi

# Set default branch name if not provided
if [ -z "$BRANCH_NAME" ]; then
    BRANCH_NAME="feature/issue-${ISSUE_NUM}"
fi

# Determine worktree path based on type
if [ "$WORKTREE_TYPE" == "layer" ]; then
    WORKTREE_DIR="$ROOT_DIR/layers/worktrees/issue-${REPO_SLUG}-${ISSUE_NUM}"
else
    WORKTREE_DIR="$ROOT_DIR/.workspace-worktrees/issue-${REPO_SLUG}-${ISSUE_NUM}"
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
echo "  Repository: $REPO_SLUG"
echo "  Type:       $WORKTREE_TYPE"
[ -n "$TARGET_LAYER" ] && echo "  Layer:      $TARGET_LAYER"
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

# For layer worktrees, set up the layer structure with symlinks
if [ "$WORKTREE_TYPE" == "layer" ]; then
    echo ""
    echo "Setting up layer worktree structure..."
    
    # Create scratchpad for this worktree
    mkdir -p "$WORKTREE_DIR/.scratchpad"
    cat > "$WORKTREE_DIR/.scratchpad/README.md" << EOF
# Task Scratchpad for Issue #$ISSUE_NUM

This directory is for temporary files specific to this task.
Files here are gitignored and isolated from other worktrees.

**Working layer**: ${TARGET_LAYER}_ws
EOF
    
    # Create symlinks to main for all layers except target
    echo "Creating symlinks to main layers..."
    for layer in "${AVAILABLE_LAYERS[@]}"; do
        LAYER_WS="${layer}_ws"
        if [ "$layer" == "$TARGET_LAYER" ]; then
            # Target layer - create real directory structure
            echo "  - $LAYER_WS: creating workspace (real)"
            mkdir -p "$WORKTREE_DIR/${LAYER_WS}/src"
            
            # Import repos for this layer using vcs
            REPOS_FILE="$ROOT_DIR/layers/main/${LAYER_WS}/src/unh_marine_autonomy/config/repos/${layer}.repos"
            if [ ! -f "$REPOS_FILE" ]; then
                REPOS_FILE="$ROOT_DIR/configs/${layer}.repos"
            fi
            
            if [ -f "$REPOS_FILE" ]; then
                echo "    Importing repositories from $REPOS_FILE..."
                cd "$WORKTREE_DIR/${LAYER_WS}/src"
                vcs import < "$REPOS_FILE" || echo "    Warning: Some repos may have failed to import"
                cd "$ROOT_DIR"
            else
                echo "    Warning: No .repos file found for $layer"
            fi
        else
            # Other layers - symlink to main
            if [ -d "$ROOT_DIR/layers/main/${LAYER_WS}" ]; then
                echo "  - $LAYER_WS: symlinking to main"
                ln -s "../../main/${LAYER_WS}" "$WORKTREE_DIR/${LAYER_WS}"
            fi
        fi
    done
fi

# For workspace worktrees, symlink the layers directory
if [ "$WORKTREE_TYPE" == "workspace" ]; then
    echo ""
    echo "Setting up workspace worktree..."
    
    # Create symlink to main layers (using relative path for portability)
    # From: .workspace-worktrees/issue-N/layers/main
    # To:   layers/main (at root)
    # Path: ../../../layers/main (up through layers/ -> issue-N/ -> .workspace-worktrees/)
    echo "Creating symlink to main layers..."
    mkdir -p "$WORKTREE_DIR/layers"
    ln -s "../../../layers/main" "$WORKTREE_DIR/layers/main"
    
    # Ensure scratchpad exists
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
    echo ""
    echo "To build the $TARGET_LAYER layer:"
    echo "  cd ${TARGET_LAYER}_ws && colcon build --symlink-install"
fi
echo ""
echo "When done, remove with:"
echo "  $SCRIPT_DIR/worktree_remove.sh --issue $ISSUE_NUM"
