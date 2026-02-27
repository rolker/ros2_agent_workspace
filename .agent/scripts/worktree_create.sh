#!/bin/bash
# .agent/scripts/worktree_create.sh
# Create a git worktree for isolated task development
#
# Usage:
#   ./worktree_create.sh --issue <number> [--type layer|workspace] [--branch <name>] [--layer <layer_name>] [--packages <pkg,...>] [--plan-file <path>]
#
# Examples:
#   ./worktree_create.sh --issue 123 --type layer --layer core --packages unh_marine_autonomy
#   ./worktree_create.sh --issue 123 --type layer --layer core --packages unh_marine_autonomy,camp
#   ./worktree_create.sh --issue 123 --type workspace
#   ./worktree_create.sh --issue 123 --type workspace --plan-file /tmp/plan.md
#   ./worktree_create.sh --issue 123 --type layer --layer core --packages sonar_driver --plan-file /tmp/plan.md
#   ./worktree_create.sh --skill research --type workspace
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

# Try to fetch a specific branch from origin.
# Returns 0 on successful fetch, non-zero otherwise.
fetch_remote_branch() {
    local git_path="$1"
    local branch="$2"
    git -C "$git_path" fetch --quiet origin -- "$branch" 2>/dev/null
}

# Extract a validated owner/repo slug from a GitHub remote URL.
# Prints the slug on stdout; prints nothing for non-GitHub or malformed URLs.
extract_gh_slug() {
    local url="$1"
    local slug
    slug=$(echo "$url" | sed -E 's#.*github\.com[:/]##' | sed 's/\.git$//')
    if [[ "$slug" =~ ^[^/[:space:]]+/[^/[:space:]]+$ ]]; then
        echo "$slug"
    fi
}

# Resolve the .repos version (branch/tag) for a package name.
# Prints the version on stdout; prints nothing if not found or unknown.
resolve_repos_branch() {
    local pkg_name="$1"
    local version
    version=$(python3 -c "
import sys; sys.path.insert(0, '${SCRIPT_DIR}')
from lib.workspace import find_repo_version
v = find_repo_version('$pkg_name')
if v and v != 'unknown': print(v)
" 2>/dev/null)
    if [ -n "$version" ]; then
        echo "$version"
    fi
}

# Defaults
ISSUE_NUM=""
SKILL_NAME=""       # Skill name (alternative to --issue)
WORKTREE_TYPE="layer"
BRANCH_NAME=""
TARGET_LAYER=""
REPO_SLUG=""
TARGET_PACKAGES=""  # Comma-separated list of packages to modify
PLAN_FILE=""        # Path to approved plan file (implies draft PR creation)

# Skills allowed to create worktrees without a GitHub issue
ALLOWED_SKILLS=("research" "gather-project-knowledge")

# Available layers (same order as env.sh)
AVAILABLE_LAYERS=("underlay" "core" "platforms" "sensors" "simulation" "ui")

show_usage() {
    echo "Usage: $0 (--issue <number> | --skill <name>) --type <layer|workspace> [options]"
    echo ""
    echo "Options:"
    echo "  --issue <number>      Issue number (required, unless --skill is used)"
    echo "  --skill <name>        Skill name (alternative to --issue; allowed: ${ALLOWED_SKILLS[*]})"
    echo "  --type <type>         Worktree type: 'layer' or 'workspace' (required)"
    echo "  --layer <name>        Layer to work on (required for layer type)"
    echo "                        Available: ${AVAILABLE_LAYERS[*]}"
    echo "  --packages <pkg,...>  Package(s) to modify (required for layer type)"
    echo "                        Comma-separated list for multiple packages"
    echo "  --repo-slug <slug>    Repository slug for naming (auto-detected if not provided)"
    echo "  --branch <name>       Custom branch name (default: feature/issue-<N> or skill/<id>)"
    echo "  --plan-file <path>    Path to approved plan file; creates draft PR and posts plan as comment"
    echo ""
    echo "Examples:"
    echo "  $0 --issue 123 --type layer --layer core --packages unh_marine_autonomy"
    echo "  $0 --issue 123 --type layer --layer core --packages unh_marine_autonomy,camp"
    echo "  $0 --issue 123 --type workspace"
    echo "  $0 --skill research --type workspace"
    echo "  $0 --issue 5 --type layer --layer sensors --packages sonar_driver --repo-slug marine_msgs"
}

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --issue)
            ISSUE_NUM="$2"
            shift 2
            ;;
        --skill)
            SKILL_NAME="$2"
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
        --plan-file)
            PLAN_FILE="$2"
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

# Validate required arguments: --issue XOR --skill
if [ -n "$ISSUE_NUM" ] && [ -n "$SKILL_NAME" ]; then
    echo "Error: --issue and --skill are mutually exclusive"
    show_usage
    exit 1
fi
if [ -z "$ISSUE_NUM" ] && [ -z "$SKILL_NAME" ]; then
    echo "Error: either --issue or --skill is required"
    show_usage
    exit 1
fi

# Validate skill name against allowlist
if [ -n "$SKILL_NAME" ]; then
    VALID_SKILL=false
    for allowed in "${ALLOWED_SKILLS[@]}"; do
        if [ "$allowed" == "$SKILL_NAME" ]; then
            VALID_SKILL=true
            break
        fi
    done
    if [ "$VALID_SKILL" = false ]; then
        echo "Error: Skill '$SKILL_NAME' is not in the allowlist"
        echo "Allowed skills: ${ALLOWED_SKILLS[*]}"
        exit 1
    fi
    # Generate synthetic ID
    SYNTHETIC_ID="${SKILL_NAME}-$(date +"%Y%m%d-%H%M%S")"
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
        # Extract owner/repo slug for GitHub CLI (e.g., "org/repo")
        GH_REPO_SLUG=$(extract_gh_slug "$REMOTE_URL")

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
        GH_REPO_SLUG=""
        REPO_SLUG="workspace"
    fi
    echo "Auto-detected repository slug: $REPO_SLUG"
else
    # --repo-slug controls worktree naming; still auto-detect GH_REPO_SLUG
    # for GitHub API calls (gh issue view, etc.)
    GH_REPO_SLUG=""
    if [ "$WORKTREE_TYPE" == "layer" ] && [ -n "$TARGET_PACKAGES" ]; then
        FIRST_PKG="${TARGET_PACKAGES%%,*}"
        FIRST_PKG="${FIRST_PKG#"${FIRST_PKG%%[![:space:]]*}"}"
        FIRST_PKG="${FIRST_PKG%"${FIRST_PKG##*[![:space:]]}"}"
        PKG_PATH="$ROOT_DIR/layers/main/${TARGET_LAYER}_ws/src/$FIRST_PKG"
        if [ -d "$PKG_PATH" ] && git -C "$PKG_PATH" remote get-url origin &>/dev/null; then
            REMOTE_URL=$(git -C "$PKG_PATH" remote get-url origin)
            GH_REPO_SLUG=$(extract_gh_slug "$REMOTE_URL")
        fi
    elif git -C "$ROOT_DIR" remote get-url origin &>/dev/null; then
        REMOTE_URL=$(git -C "$ROOT_DIR" remote get-url origin)
        GH_REPO_SLUG=$(extract_gh_slug "$REMOTE_URL")
    fi
    REPO_SLUG=$(echo "$REPO_SLUG" | sed 's/[^A-Za-z0-9_]/_/g')
fi

# Validate issue exists and fetch title (after GH_REPO_SLUG is resolved)
# Skip in skill mode — there is no issue to validate
ISSUE_TITLE=""
ISSUE_STATE=""
if [ -n "$ISSUE_NUM" ]; then
    if command -v gh &>/dev/null; then
        if [ -n "$GH_REPO_SLUG" ]; then
            _ISSUE_INFO=$(gh issue view "$ISSUE_NUM" --repo "$GH_REPO_SLUG" --json title,state --jq '.title + "||" + .state' 2>/dev/null || echo "")
        else
            _ISSUE_INFO=$(gh issue view "$ISSUE_NUM" --json title,state --jq '.title + "||" + .state' 2>/dev/null || echo "")
        fi
        if [[ "$_ISSUE_INFO" == *"||"* ]]; then
            ISSUE_TITLE="${_ISSUE_INFO%||*}"
            ISSUE_STATE="${_ISSUE_INFO##*||}"
        fi
    fi
    if [ -n "$ISSUE_TITLE" ]; then
        echo "📋 Issue #$ISSUE_NUM: $ISSUE_TITLE"
        if [ "$ISSUE_STATE" = "CLOSED" ]; then
            echo "   ⚠️  Warning: Issue #$ISSUE_NUM is CLOSED"
        fi
    else
        echo "⚠️  Could not fetch issue #$ISSUE_NUM title (offline or issue does not exist)"
        echo "   Proceeding anyway — verify the issue number is correct."
    fi
else
    echo "🔧 Skill worktree: $SKILL_NAME (ID: $SYNTHETIC_ID)"
fi
echo ""

# Set default branch name if not provided
if [ -z "$BRANCH_NAME" ]; then
    if [ -n "$SKILL_NAME" ]; then
        BRANCH_NAME="skill/${SYNTHETIC_ID}"
    else
        BRANCH_NAME="feature/issue-${ISSUE_NUM}"
    fi
fi

# Determine worktree path based on type and mode (issue vs skill)
if [ -n "$SKILL_NAME" ]; then
    DIR_PREFIX="skill-${REPO_SLUG}-${SYNTHETIC_ID}"
else
    DIR_PREFIX="issue-${REPO_SLUG}-${ISSUE_NUM}"
fi
if [ "$WORKTREE_TYPE" == "layer" ]; then
    WORKTREE_DIR="$ROOT_DIR/layers/worktrees/${DIR_PREFIX}"
else
    WORKTREE_DIR="$ROOT_DIR/.workspace-worktrees/${DIR_PREFIX}"
fi

# Check if worktree already exists
if [ -d "$WORKTREE_DIR" ]; then
    echo "Error: Worktree already exists at $WORKTREE_DIR"
    if [ -n "$SKILL_NAME" ]; then
        echo "Use 'worktree_enter.sh --skill $SKILL_NAME' to enter it"
        echo "Or 'worktree_remove.sh --skill $SKILL_NAME' to remove it"
    else
        echo "Use 'worktree_enter.sh --issue $ISSUE_NUM' to enter it"
        echo "Or 'worktree_remove.sh --issue $ISSUE_NUM' to remove it"
    fi
    exit 1
fi

cd "$ROOT_DIR"

echo "========================================"
echo "Creating Worktree"
echo "========================================"
if [ -n "$SKILL_NAME" ]; then
    echo "  Skill:      $SKILL_NAME"
    echo "  ID:         $SYNTHETIC_ID"
else
    echo "  Issue:      #$ISSUE_NUM"
fi
echo "  Repository: $REPO_SLUG"
echo "  Type:       $WORKTREE_TYPE"
[ -n "$TARGET_LAYER" ] && echo "  Layer:      $TARGET_LAYER"
echo "  Branch:     $BRANCH_NAME"
echo "  Path:       $WORKTREE_DIR"
echo ""

# Create parent directory if needed
mkdir -p "$(dirname "$WORKTREE_DIR")"

# Create the worktree directory
if [ "$WORKTREE_TYPE" == "workspace" ]; then
    # Workspace worktrees are full git worktrees of the workspace repo
    if git show-ref --verify --quiet "refs/heads/$BRANCH_NAME"; then
        echo "Using existing local branch '$BRANCH_NAME'..."
        git worktree add "$WORKTREE_DIR" "$BRANCH_NAME"
    elif fetch_remote_branch "$ROOT_DIR" "$BRANCH_NAME"; then
        echo "Tracking remote branch 'origin/$BRANCH_NAME'..."
        git worktree add --track -b "$BRANCH_NAME" "$WORKTREE_DIR" "origin/$BRANCH_NAME"
    else
        echo "Creating new branch '$BRANCH_NAME' from current HEAD..."
        git worktree add -b "$BRANCH_NAME" "$WORKTREE_DIR"
    fi
else
    # Layer worktrees are plain directories containing per-package git worktrees.
    # The branch is created in the individual package repos, not the workspace repo.
    mkdir -p "$WORKTREE_DIR"
fi

# For layer worktrees, set up the layer structure with symlinks
if [ "$WORKTREE_TYPE" == "layer" ]; then
    echo ""
    echo "Setting up layer worktree structure..."

    # Create scratchpad for this worktree
    mkdir -p "$WORKTREE_DIR/.scratchpad"
    if [ -n "$SKILL_NAME" ]; then
        cat > "$WORKTREE_DIR/.scratchpad/README.md" << EOF
# Task Scratchpad for Skill: $SKILL_NAME ($SYNTHETIC_ID)

This directory is for temporary files specific to this task.
Files here are gitignored and isolated from other worktrees.

**Working layer**: ${TARGET_LAYER}_ws
EOF
    else
        cat > "$WORKTREE_DIR/.scratchpad/README.md" << EOF
# Task Scratchpad for Issue #$ISSUE_NUM

This directory is for temporary files specific to this task.
Files here are gitignored and isolated from other worktrees.

**Working layer**: ${TARGET_LAYER}_ws
EOF
    fi

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

                        # Resolve the .repos base branch for this package
                        REPOS_BRANCH=$(resolve_repos_branch "$pkg_name")
                        if [ -n "$REPOS_BRANCH" ]; then
                            git fetch --quiet origin "$REPOS_BRANCH" 2>/dev/null || true
                        fi

                        # Prefer the issue branch for this package: local first, then remote, then new from .repos branch, then new from HEAD
                        if git worktree add "$WORKTREE_PKG_PATH" "$BRANCH_NAME" 2>/dev/null; then
                            echo "      ✓ Worktree created from existing local branch: $BRANCH_NAME"
                        elif fetch_remote_branch "$pkg_path" "$BRANCH_NAME" && \
                             git worktree add --track -b "$BRANCH_NAME" "$WORKTREE_PKG_PATH" "origin/$BRANCH_NAME" 2>/dev/null; then
                            echo "      ✓ Worktree created tracking remote branch: origin/$BRANCH_NAME"
                        elif [ -n "$REPOS_BRANCH" ] && \
                             git worktree add -b "$BRANCH_NAME" "$WORKTREE_PKG_PATH" "origin/$REPOS_BRANCH" 2>/dev/null; then
                            echo "      ✓ Worktree created with new branch: $BRANCH_NAME (from $REPOS_BRANCH)"
                        elif git worktree add -b "$BRANCH_NAME" "$WORKTREE_PKG_PATH" 2>/dev/null; then
                            echo "      ✓ Worktree created with new branch: $BRANCH_NAME"
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
if [ -n "$SKILL_NAME" ]; then
    echo "  Skill: $SKILL_NAME (ID: $SYNTHETIC_ID)"
elif [ -n "$ISSUE_TITLE" ]; then
    echo "  Issue #$ISSUE_NUM: $ISSUE_TITLE"
fi
echo ""

# --plan-file: push branch, create draft PR, post plan as comment
if [ -n "$PLAN_FILE" ]; then
    if [ -n "$SKILL_NAME" ]; then
        echo "Creating draft PR for skill: $SKILL_NAME..."
    else
        echo "Creating draft PR for issue #$ISSUE_NUM..."
    fi
    echo ""

    # Use issue title fetched earlier; fall back to generic
    if [ -n "$SKILL_NAME" ]; then
        ISSUE_TITLE="Skill update: $SKILL_NAME"
    elif [ -z "$ISSUE_TITLE" ]; then
        echo "  ⚠️  Could not fetch issue title; using generic title"
        ISSUE_TITLE="Issue #$ISSUE_NUM"
    fi

    # Validate plan file exists
    HAS_PLAN=false
    if [ ! -f "$PLAN_FILE" ]; then
        echo "  ⚠️  Plan file not found: $PLAN_FILE (creating PR without plan comment)"
    else
        HAS_PLAN=true
    fi

    # Auto-detect agent identity if not already set in environment
    if [ -z "$AGENT_NAME" ] || [ -z "$AGENT_MODEL" ]; then
        if [ -f "$SCRIPT_DIR/framework_config.sh" ]; then
            source "$SCRIPT_DIR/framework_config.sh"
        fi
        if [ -f "$SCRIPT_DIR/detect_cli_env.sh" ]; then
            source "$SCRIPT_DIR/detect_cli_env.sh" || true
        fi
        if [ -n "$AGENT_FRAMEWORK" ] && [ "$AGENT_FRAMEWORK" != "unknown" ]; then
            FRAMEWORK_KEY="${AGENT_FRAMEWORK%-cli}"
            FRAMEWORK_KEY="${FRAMEWORK_KEY,,}"
            : "${AGENT_NAME:=${FRAMEWORK_NAMES[$FRAMEWORK_KEY]:-AI Agent}}"
            : "${AGENT_MODEL:=${FRAMEWORK_MODELS[$FRAMEWORK_KEY]:-Unknown}}"
        fi
    fi

    # Agent name/model for signatures — fall back to generic if still unset
    DRAFT_AGENT_NAME="${AGENT_NAME:-AI Agent}"
    DRAFT_AGENT_MODEL="${AGENT_MODEL:-Unknown}"

    # Helper: create a draft PR and optionally post the plan as a comment.
    # Arguments:
    #   $1 - git directory to push from
    #   $2 - issue reference (e.g. "#123" or "owner/repo#123")
    #   $3 - (optional) repo slug for gh pr create
    #   $4 - (optional) base branch for gh pr create
    create_draft_pr() {
        local git_dir="$1"
        local issue_ref="$2"
        local repo_flag="${3:-}"
        local base_flag="${4:-}"

        cd "$git_dir"

        # Push branch (empty commit if needed to create the remote ref)
        if ! git rev-parse --verify "origin/$BRANCH_NAME" &>/dev/null; then
            if [ -n "$issue_ref" ]; then
                git commit --allow-empty -m "chore: start work on $issue_ref" || true
            else
                git commit --allow-empty -m "chore: start skill update ($SKILL_NAME)" || true
            fi
        fi
        if ! git push -u origin "$BRANCH_NAME" 2>/dev/null; then
            echo "  ⚠️  Push failed — skipping draft PR (non-fatal)"
            return 1
        fi

        # Check if a PR already exists for this branch
        local existing_pr=""
        if [ -n "$repo_flag" ]; then
            existing_pr=$(gh pr list --repo "$repo_flag" --head "$BRANCH_NAME" --json url --jq '.[0].url' 2>/dev/null || echo "")
        else
            existing_pr=$(gh pr list --head "$BRANCH_NAME" --json url --jq '.[0].url' 2>/dev/null || echo "")
        fi
        if [ -n "$existing_pr" ]; then
            echo "  ℹ PR already exists: $existing_pr"
            # Post plan as comment on existing PR if available
            if [ "$HAS_PLAN" = true ]; then
                if gh pr comment "$existing_pr" --body-file "$PLAN_FILE" >/dev/null 2>&1; then
                    echo "  ✓ Plan posted as comment on existing PR"
                fi
            fi
            return 0
        fi

        # Create draft PR
        BODY_FILE=$(mktemp /tmp/gh_body.XXXXXX.md)
        if [ -n "$SKILL_NAME" ]; then
            cat > "$BODY_FILE" << PREOF
## Summary

$ISSUE_TITLE

Automated update from the \`$SKILL_NAME\` skill.

---
**Authored-By**: \`${DRAFT_AGENT_NAME}\`
**Model**: \`${DRAFT_AGENT_MODEL}\`
PREOF
        else
            cat > "$BODY_FILE" << PREOF
## Summary

$ISSUE_TITLE

Closes $issue_ref

---
**Authored-By**: \`${DRAFT_AGENT_NAME}\`
**Model**: \`${DRAFT_AGENT_MODEL}\`
PREOF
        fi

        local gh_args=(pr create --draft --title "$ISSUE_TITLE" --body-file "$BODY_FILE")
        [ -n "$repo_flag" ] && gh_args+=(--repo "$repo_flag")
        [ -n "$base_flag" ] && gh_args+=(--base "$base_flag")

        GH_STDERR=$(mktemp /tmp/gh_stderr.XXXXXX)
        local pr_url
        pr_url=$(gh "${gh_args[@]}" 2>"$GH_STDERR") && PR_CREATED=true || PR_CREATED=false

        if [ "$PR_CREATED" = true ]; then
            echo "  ✓ Draft PR created: $pr_url"
            # Post plan as PR comment
            if [ "$HAS_PLAN" = true ]; then
                if gh pr comment "$pr_url" --body-file "$PLAN_FILE" >/dev/null 2>&1; then
                    echo "  ✓ Plan posted as PR comment"
                else
                    echo "  ⚠️  Failed to post plan comment (non-fatal)"
                fi
            fi
        else
            echo "  ⚠️  Draft PR creation failed (non-fatal)"
            cat "$GH_STDERR" >&2
        fi
        rm -f "$GH_STDERR" "$BODY_FILE"
    }

    if [ "$WORKTREE_TYPE" == "workspace" ]; then
        if [ -n "$SKILL_NAME" ]; then
            create_draft_pr "$WORKTREE_DIR" ""
        else
            create_draft_pr "$WORKTREE_DIR" "#$ISSUE_NUM"
        fi
        cd "$ROOT_DIR"

    elif [ "$WORKTREE_TYPE" == "layer" ]; then
        for pkg_dir in "$WORKTREE_DIR"/"${TARGET_LAYER}_ws"/src/*; do
            # Skip symlinks (unmodified packages) and non-git dirs
            [ -L "$pkg_dir" ] && continue
            [ ! -e "$pkg_dir/.git" ] && continue

            pkg_name=$(basename "$pkg_dir")
            echo "  Creating draft PR for package: $pkg_name"

            # Determine repo slug from remote
            PKG_REMOTE_URL=$(git -C "$pkg_dir" remote get-url origin 2>/dev/null || echo "")
            PKG_REPO_SLUG=""
            if [ -n "$PKG_REMOTE_URL" ]; then
                PKG_REPO_SLUG=$(extract_gh_slug "$PKG_REMOTE_URL")
            fi

            # Build issue reference — use cross-repo format when PR repo != issue repo
            if [ -n "$GH_REPO_SLUG" ] && [ -n "$PKG_REPO_SLUG" ] && [ "$GH_REPO_SLUG" != "$PKG_REPO_SLUG" ]; then
                ISSUE_REF="$GH_REPO_SLUG#$ISSUE_NUM"
            else
                ISSUE_REF="#$ISSUE_NUM"
            fi

            # Resolve the .repos base branch for the PR target
            PKG_BASE_BRANCH=$(resolve_repos_branch "$pkg_name")

            create_draft_pr "$pkg_dir" "$ISSUE_REF" "$PKG_REPO_SLUG" "$PKG_BASE_BRANCH"
        done
        cd "$ROOT_DIR"
    fi

    echo ""
fi

if [ -n "$SKILL_NAME" ]; then
    echo "To enter this worktree:"
    echo "  source $SCRIPT_DIR/worktree_enter.sh --skill $SKILL_NAME"
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
    echo "  $SCRIPT_DIR/worktree_remove.sh --skill $SKILL_NAME"
else
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
fi
