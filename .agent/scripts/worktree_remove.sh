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

# Capture caller's physical working directory before any cd operations.
# We need this to detect if the caller is inside the worktree being removed.
# Since this script is executed (not sourced), we cannot change the caller's
# cwd — so we must refuse to proceed if they're inside the target worktree.
# Use pwd -P to resolve symlinks so the comparison is reliable.
CALLER_PWD="$(pwd -P)"

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
            local slug
            slug=$(basename "$path" | sed -E 's/^issue-(.+)-[0-9]+$/\1/')
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

# Refuse to proceed if the caller's shell is inside the worktree.
# This script runs as a subprocess, so it cannot change the caller's cwd.
# Removing the directory while the caller is inside it leaves their shell broken.
# Canonicalize WORKTREE_DIR so it matches the physical CALLER_PWD.
WORKTREE_DIR=$(cd "$WORKTREE_DIR" && pwd -P)
if [[ "$CALLER_PWD" == "$WORKTREE_DIR" || "$CALLER_PWD" == "$WORKTREE_DIR/"* ]]; then
    echo "❌ Error: Your shell is currently inside this worktree."
    echo ""
    echo "   Run this first:  cd $ROOT_DIR"
    echo "   Then re-run:     $0 --issue $ISSUE_NUM"
    exit 1
fi

# For layer worktrees, perform all safety checks first (inner packages,
# unrecognized content, workspace-level), then clean up layer infrastructure
# only after every check has passed.  This avoids leaving the worktree in a
# partially-destroyed state if a check causes an early exit.
INNER_WORKTREE_REPOS=()  # Parent repos of inner worktrees (for pruning later)

if [ "$WORKTREE_TYPE" == "layer" ] && [ -d "$WORKTREE_DIR" ]; then
    HAS_ISSUES=false

    # --- Phase 1: Check inner package worktrees and detect unrecognized content ---
    for ws_dir in "$WORKTREE_DIR"/*_ws; do
        [ -d "$ws_dir" ] || continue
        [ -L "$ws_dir" ] && continue  # skip symlinked layers
        SRC_DIR="$ws_dir/src"
        if [ ! -d "$SRC_DIR" ]; then
            # Non-symlink workspace directory without expected src/ layout
            if [ "$FORCE" != true ]; then
                echo "⚠️  Warning: Unrecognized workspace directory '$(basename "$ws_dir")' (no src/ directory)"
                HAS_ISSUES=true
            fi
            continue
        fi

        for pkg_dir in "$SRC_DIR"/*; do
            [ -e "$pkg_dir" ] || continue
            [ -L "$pkg_dir" ] && continue  # skip symlinked packages

            if [ -d "$pkg_dir" ] && git -C "$pkg_dir" rev-parse --git-dir &>/dev/null; then
                # Known inner git worktree — check for uncommitted changes
                PKG_NAME=$(basename "$pkg_dir")
                PKG_STATUS=$(git -C "$pkg_dir" status --porcelain 2>/dev/null)
                if [ -n "$PKG_STATUS" ] && [ "$FORCE" != true ]; then
                    echo "⚠️  Warning: Package '$PKG_NAME' has uncommitted changes:"
                    echo ""
                    git -C "$pkg_dir" status --short
                    echo ""
                    HAS_ISSUES=true
                fi

                # Track the parent repo for pruning later
                MAIN_REPO=$(
                    cd "$pkg_dir" && {
                        cd "$(git rev-parse --git-common-dir)" &&
                        pwd -P | sed 's|/\.git$||'
                    } 2>/dev/null
                )
                if [ -n "$MAIN_REPO" ] && [ -d "$MAIN_REPO" ]; then
                    INNER_WORKTREE_REPOS+=("$MAIN_REPO")
                fi
            else
                # Not a symlink and not a git repo — unrecognized user content
                if [ "$FORCE" != true ]; then
                    echo "⚠️  Warning: Unrecognized content in $(basename "$ws_dir")/src/: $(basename "$pkg_dir")"
                    HAS_ISSUES=true
                fi
            fi
        done

        # Check for unexpected user content outside known infrastructure dirs
        KNOWN_INFRA="^(src|build|install|log)$"
        for entry in "$ws_dir"/*; do
            [ -e "$entry" ] || continue
            entry_name=$(basename "$entry")
            if ! echo "$entry_name" | grep -qE "$KNOWN_INFRA"; then
                if [ "$FORCE" != true ]; then
                    echo "⚠️  Warning: Unexpected content in $(basename "$ws_dir")/: $entry_name"
                    HAS_ISSUES=true
                fi
            fi
        done
    done

    # --- Phase 2: Check workspace-level uncommitted changes ---
    # Filter out known layer infrastructure so only genuine changes are flagged.
    # Treat any top-level *_ws directory as infrastructure (Phase 1 already
    # validated each workspace dir individually for unexpected content).
    INFRA_PATTERN='^(\?\? |.. )(\.scratchpad/|[^/]*_ws(/|$))'
    cd "$WORKTREE_DIR"
    WORKSPACE_UNCOMMITTED=$(git status --porcelain 2>/dev/null | grep -v -E "$INFRA_PATTERN" || true)
    cd "$ROOT_DIR"

    if [ -n "$WORKSPACE_UNCOMMITTED" ] && [ "$FORCE" != true ]; then
        echo "⚠️  Warning: Worktree has uncommitted changes:"
        echo ""
        echo "$WORKSPACE_UNCOMMITTED"
        echo ""
        HAS_ISSUES=true
    fi

    if [ "$HAS_ISSUES" = true ]; then
        echo "Use --force to remove anyway, or commit/stash your changes first."
        exit 1
    fi

    # --- Phase 3: All checks passed — perform destructive cleanup ---
    for ws_dir in "$WORKTREE_DIR"/*_ws; do
        [ -e "$ws_dir" ] || continue
        if [ -L "$ws_dir" ]; then
            rm "$ws_dir"
        elif [ -d "$ws_dir" ]; then
            rm -rf "$ws_dir"
        fi
    done
    rm -rf "$WORKTREE_DIR/.scratchpad"

# Non-layer worktrees: simple uncommitted changes check
elif [ -d "$WORKTREE_DIR" ]; then
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

# Remove the worktree
echo "Removing worktree..."
if [ "$FORCE" = true ]; then
    git worktree remove --force "$WORKTREE_DIR"
else
    git worktree remove "$WORKTREE_DIR"
fi

# Prune worktree references (workspace repo + inner package repos)
git worktree prune
if [ "${#INNER_WORKTREE_REPOS[@]}" -gt 0 ]; then
    declare -A SEEN_REPOS=()
    for repo_path in "${INNER_WORKTREE_REPOS[@]}"; do
        if [ -n "$repo_path" ] && [ -z "${SEEN_REPOS[$repo_path]+_}" ]; then
            SEEN_REPOS["$repo_path"]=1
            git -C "$repo_path" worktree prune 2>/dev/null || true
        fi
    done
fi

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
