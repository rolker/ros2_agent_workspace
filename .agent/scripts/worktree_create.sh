#!/bin/bash
# .agent/scripts/worktree_create.sh
# Create a git worktree for isolated task development
#
# Usage:
#   ./worktree_create.sh --issue <number> [--type layer|workspace] [--branch <name>] [--layer <layer_name>] [--packages <pkg,...>]
#
# Examples:
#   ./worktree_create.sh --issue 123 --type layer --layer core --packages unh_marine_autonomy
#   ./worktree_create.sh --issue 123 --type layer --layer core --packages unh_marine_autonomy,camp
#   ./worktree_create.sh --issue 123 --type workspace
#   ./worktree_create.sh --issue 123 --type layer --layer core --packages sonar_driver --branch feature/custom-name
#
# Worktree Types:
#   layer     - For ROS package development
#               Created in: layers/worktrees/issue-<N>/
#               Uses hybrid structure for efficiency:
#                 - Git worktrees for packages being modified (--packages)
#                 - Symlinks for other packages in target layer (reuse builds)
#                 - Symlinks for other layers (reuse builds)
#               Requires --layer and --packages
#
#   workspace - For infrastructure work (.agent/, configs/, docs)
#               Created in: .workspace-worktrees/issue-<N>/
#               Full workspace checkout with symlinked layers/
#               No package modification

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

# Defaults
ISSUE_NUM=""
WORKTREE_TYPE="layer"
BRANCH_NAME=""
TARGET_LAYER=""
REPO_SLUG=""
TARGET_PACKAGES=""  # Comma-separated list of packages to modify

# Available layers (same order as env.sh)
AVAILABLE_LAYERS=("underlay" "core" "platforms" "sensors" "simulation" "ui")

show_usage() {
    echo "Usage: $0 --issue <number> --type <layer|workspace> [options]"
    echo ""
    echo "Options:"
    echo "  --issue <number>      Issue number (required)"
    echo "  --type <type>         Worktree type: 'layer' or 'workspace' (required)"
    echo "  --layer <name>        Layer to work on (required for layer type)"
    echo "                        Available: ${AVAILABLE_LAYERS[*]}"
    echo "  --packages <pkg,...>  Package(s) to modify (required for layer type)"
    echo "                        Comma-separated list for multiple packages"
    echo "  --repo-slug <slug>    Repository slug for naming (auto-detected if not provided)"
    echo "  --branch <name>       Custom branch name (default: feature/issue-<N>)"
    echo ""
    echo "Examples:"
    echo "  $0 --issue 123 --type layer --layer core --packages unh_marine_autonomy"
    echo "  $0 --issue 123 --type layer --layer core --packages unh_marine_autonomy,camp"
    echo "  $0 --issue 123 --type workspace"
    echo "  $0 --issue 5 --type layer --layer sensors --packages sonar_driver --repo-slug marine_msgs"
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
        --packages)
            TARGET_PACKAGES="$2"
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

# For layer worktrees, require --layer and --packages
if [ "$WORKTREE_TYPE" == "layer" ]; then
    if [ -z "$TARGET_LAYER" ]; then
        echo "Error: --layer is required for layer worktrees"
        echo "Available layers: ${AVAILABLE_LAYERS[*]}"
        exit 1
    fi
    if [ -z "$TARGET_PACKAGES" ]; then
        echo "Error: --packages is required for layer worktrees"
        echo "Example: --packages unh_marine_autonomy"
        echo "         --packages unh_marine_autonomy,camp  (for multiple)"
        exit 1
    fi
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
    REMOTE_URL=""

    # For layer worktrees, detect slug from the first package's git remote
    # (the issue typically lives in the same repo as the package being modified)
    if [ "$WORKTREE_TYPE" == "layer" ] && [ -n "$TARGET_PACKAGES" ]; then
        # Extract first package name from comma-separated list
        FIRST_PKG="${TARGET_PACKAGES%%,*}"
        # Trim whitespace
        FIRST_PKG="${FIRST_PKG#"${FIRST_PKG%%[![:space:]]*}"}"
        FIRST_PKG="${FIRST_PKG%"${FIRST_PKG##*[![:space:]]}"}"

        PKG_PATH="$ROOT_DIR/layers/main/${TARGET_LAYER}_ws/src/$FIRST_PKG"
        if [ -d "$PKG_PATH" ] && git -C "$PKG_PATH" remote get-url origin &>/dev/null; then
            REMOTE_URL=$(git -C "$PKG_PATH" remote get-url origin)
        fi
    fi

    # Fallback: detect from workspace repo remote (use -C to ensure we
    # always query the workspace repo, regardless of the caller's cwd)
    if [ -z "$REMOTE_URL" ]; then
        if git -C "$ROOT_DIR" remote get-url origin &>/dev/null; then
            REMOTE_URL=$(git -C "$ROOT_DIR" remote get-url origin)
        fi
    fi

    if [ -n "$REMOTE_URL" ]; then
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
        # Fallback to "workspace" if no remote detected
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
            # Target layer - create hybrid structure: git worktrees for modified packages, symlinks for others
            echo "  - $LAYER_WS: creating hybrid structure (git worktrees + symlinks)"
            mkdir -p "$WORKTREE_DIR/${LAYER_WS}/src"

            # Convert comma-separated packages to array without permanently changing IFS
            OLD_IFS=$IFS
            IFS=','
            # Read raw entries, then trim whitespace from each package name
            read -ra RAW_PACKAGE_ARRAY <<< "$TARGET_PACKAGES"
            IFS=$OLD_IFS

            PACKAGE_ARRAY=()
            for raw_pkg in "${RAW_PACKAGE_ARRAY[@]}"; do
                # Trim leading and trailing whitespace
                pkg="${raw_pkg#"${raw_pkg%%[![:space:]]*}"}"
                pkg="${pkg%"${pkg##*[![:space:]]}"}"
                # Skip empty entries (e.g., from ",,")
                if [ -n "$pkg" ]; then
                    PACKAGE_ARRAY+=("$pkg")
                fi
            done

            # Get list of all packages in main layer
            MAIN_SRC_DIR="$ROOT_DIR/layers/main/${LAYER_WS}/src"
            if [ ! -d "$MAIN_SRC_DIR" ]; then
                echo "    Error: Layer directory not found: $MAIN_SRC_DIR"
                exit 1
            fi

            # Process each package in main
            for pkg_path in "$MAIN_SRC_DIR"/*; do
                if [ ! -d "$pkg_path" ]; then
                    continue
                fi

                pkg_name=$(basename "$pkg_path")

                # Check if this package should have a worktree
                CREATE_WORKTREE=false
                for target_pkg in "${PACKAGE_ARRAY[@]}"; do
                    if [ "$pkg_name" == "$target_pkg" ]; then
                        CREATE_WORKTREE=true
                        break
                    fi
                done

                if [ "$CREATE_WORKTREE" = true ]; then
                    # Create git worktree for this package
                    echo "    - $pkg_name: creating git worktree (modified)"

                    # Check if package is a git repository
                    if [ -d "$pkg_path/.git" ]; then
                        # Create worktree from this package's git repo
                        cd "$pkg_path"
                        WORKTREE_PKG_PATH="$WORKTREE_DIR/${LAYER_WS}/src/$pkg_name"

                        # Prefer the issue branch for this package: create it if needed, or reuse if it exists
                        if git worktree add -b "$BRANCH_NAME" "$WORKTREE_PKG_PATH" 2>/dev/null; then
                            echo "      ✓ Worktree created with new branch: $BRANCH_NAME"
                        elif git worktree add "$WORKTREE_PKG_PATH" "$BRANCH_NAME" 2>/dev/null; then
                            echo "      ✓ Worktree created from existing branch: $BRANCH_NAME"
                        else
                            # Fallback: attempt to use the package's current branch, then symlink on failure
                            CURRENT_BRANCH=$(git branch --show-current 2>/dev/null || true)
                            if [ -n "$CURRENT_BRANCH" ] && git worktree add "$WORKTREE_PKG_PATH" "$CURRENT_BRANCH" 2>/dev/null; then
                                echo "      ✓ Worktree created from current branch: $CURRENT_BRANCH"
                            else
                                echo "      ✗ Failed to create worktree for $pkg_name on branch: $BRANCH_NAME"
                                echo "      Falling back to symlink"
                                ln -s "$pkg_path" "$WORKTREE_PKG_PATH"
                            fi
                        fi
                        cd "$ROOT_DIR"
                    else
                        # Not a git repo, just symlink it
                        echo "      ! Not a git repo, symlinking instead"
                        ln -s "$pkg_path" "$WORKTREE_DIR/${LAYER_WS}/src/$pkg_name"
                    fi
                else
                    # Symlink to main for unmodified packages
                    echo "    - $pkg_name: symlinking to main"
                    ln -s "$pkg_path" "$WORKTREE_DIR/${LAYER_WS}/src/$pkg_name"
                fi
            done

            # Verify all requested packages were found
            for target_pkg in "${PACKAGE_ARRAY[@]}"; do
                if [ ! -e "$WORKTREE_DIR/${LAYER_WS}/src/$target_pkg" ]; then
                    echo "    ⚠️  Warning: Package '$target_pkg' not found in layer '$layer'"
                fi
            done
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
