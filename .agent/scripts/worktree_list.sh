#!/bin/bash
# .agent/scripts/worktree_list.sh
# List all git worktrees for this workspace
#
# Usage:
#   ./worktree_list.sh [--verbose] [--json]
#
# Shows all active worktrees including:
#   - Issue number
#   - Type (layer/workspace)
#   - Branch name
#   - Path
#   - Status (clean/dirty)
#
# Options:
#   --json    Output structured JSON to stdout (diagnostics to stderr)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

source "$SCRIPT_DIR/_worktree_helpers.sh"

VERBOSE=false
JSON_OUTPUT=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -v|--verbose)
            VERBOSE=true
            shift
            ;;
        --json)
            JSON_OUTPUT=true
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [--verbose] [--json]"
            echo ""
            echo "Options:"
            echo "  -v, --verbose    Show detailed status for each worktree"
            echo "      --json       Output structured JSON to stdout"
            exit 0
            ;;
        *)
            echo "Error: Unknown option $1" >&2
            exit 1
            ;;
    esac
done

# JSON array accumulator
JSON_ENTRIES=()
DIRTY_COUNT=0

# Helper: escape a string for JSON (handles quotes, backslashes, newlines)
json_escape() {
    local s="$1"
    s="${s//\\/\\\\}"
    s="${s//\"/\\\"}"
    s="${s//$'\n'/\\n}"
    s="${s//$'\t'/\\t}"
    printf '%s' "$s"
}

# Helper: build a JSON object for one worktree
# Arguments: type issue skill path branch status repo layer files_changed
build_json_entry() {
    local type="$1" issue="$2" skill="$3" path="$4" branch="$5"
    local status="$6" repo="$7" layer="$8" files_changed="$9"

    local issue_val="null"
    [ -n "$issue" ] && issue_val="$issue"

    local skill_val="null"
    [ -n "$skill" ] && skill_val="\"$(json_escape "$skill")\""

    local repo_val="null"
    [ -n "$repo" ] && repo_val="\"$(json_escape "$repo")\""

    local layer_val="null"
    [ -n "$layer" ] && layer_val="\"$(json_escape "$layer")\""

    local branch_val="null"
    [ -n "$branch" ] && branch_val="\"$(json_escape "$branch")\""

    printf '{"type":"%s","issue":%s,"skill":%s,"path":"%s","branch":%s,"status":"%s","repo":%s,"layer":%s,"files_changed":%s}' \
        "$type" "$issue_val" "$skill_val" \
        "$(json_escape "$path")" "$branch_val" "$status" \
        "$repo_val" "$layer_val" "${files_changed:-0}"
}

cd "$ROOT_DIR"

if [ "$JSON_OUTPUT" = false ]; then
    echo "========================================"
    echo "Git Worktrees"
    echo "========================================"
    echo ""
fi

# Get worktree list from git (finds main workspace + workspace worktrees)
WORKTREES=$(git worktree list --porcelain)

# Count worktrees (excluding main)
LAYER_COUNT=0
WORKSPACE_COUNT=0

# Helper: extract issue/repo/skill from worktree directory basename
# Sets variables: WT_ISSUE, WT_REPO, WT_SKILL
extract_issue_repo() {
    local basename="$1"
    WT_ISSUE=""
    WT_REPO=""
    WT_SKILL=""

    # Skill format: skill-{REPO_SLUG}-{SKILL_NAME}-{TIMESTAMP}
    # Timestamp may be YYYYMMDD-HHMMSS or YYYYMMDD-HHMMSS-NNNNNNNNN
    if [[ "$basename" =~ ^skill-([a-zA-Z0-9_]+)-([a-zA-Z0-9_-]+)-([0-9]{8}-[0-9]{6}(-[0-9]+)?)$ ]]; then
        WT_REPO="${BASH_REMATCH[1]}"
        WT_SKILL="${BASH_REMATCH[2]}"
    # New format: issue-{REPO_SLUG}-{NUMBER}
    elif [[ "$basename" =~ ^issue-([a-zA-Z0-9_]+)-([0-9]+)$ ]]; then
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

    local skill=""
    if [[ "$path" == *"/.workspace-worktrees/"* ]]; then
        type="workspace"
        extract_issue_repo "$(basename "$path")"
        issue="$WT_ISSUE"
        repo="$WT_REPO"
        skill="$WT_SKILL"
        ((WORKSPACE_COUNT++)) || true
    fi

    # Check if clean or dirty
    local status="clean"
    local files_changed=0
    if [ -d "$path" ]; then
        local porcelain
        porcelain="$(git -C "$path" status --porcelain 2>/dev/null || true)"
        if [ -n "$porcelain" ]; then
            status="dirty"
        fi
        if [ -n "$porcelain" ]; then
            files_changed=$(echo "$porcelain" | wc -l)
            files_changed=$((files_changed + 0))  # strip whitespace
        fi
    fi

    # Track dirty count (exclude main — summary.total excludes main too)
    if [ "$status" = "dirty" ] && [ "$type" != "main" ]; then
        ((DIRTY_COUNT++)) || true
    fi

    # Collect JSON entry (always, for both modes)
    local display_branch="${branch:-detached at $head}"
    JSON_ENTRIES+=("$(build_json_entry "$type" "$issue" "$skill" "$path" "$display_branch" "$status" "$repo" "" "$files_changed")")

    # Format text output
    if [ "$JSON_OUTPUT" = false ]; then
        if [ "$type" == "main" ]; then
            echo "[main] Main Workspace"
            echo "   Path:   $path"
            echo "   Branch: $display_branch"
            echo "   Status: $status"
        elif [ -n "$skill" ]; then
            echo "[workspace] Skill: $skill ($type) - Repository: $repo"
            echo "   Path:   $path"
            echo "   Branch: $display_branch"
            echo "   Status: $status"

            if [ "$VERBOSE" = true ] && [ -d "$path" ]; then
                echo "   Files changed: $files_changed"
            fi
        else
            echo "[workspace] Issue #$issue ($type) - Repository: $repo"
            echo "   Path:   $path"
            echo "   Branch: $display_branch"
            echo "   Status: $status"

            if [ "$VERBOSE" = true ] && [ -d "$path" ]; then
                echo "   Files changed: $files_changed"
            fi
        fi
        echo ""
    fi
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
    for layer_wt in "$LAYER_WT_DIR"/issue-* "$LAYER_WT_DIR"/skill-*; do
        [ -d "$layer_wt" ] || continue

        extract_issue_repo "$(basename "$layer_wt")"
        local_issue="$WT_ISSUE"
        local_repo="$WT_REPO"
        local_skill="$WT_SKILL"

        # Get branch from inner package worktree
        local_branch=$(wt_layer_branch "$layer_wt" 2>/dev/null || echo "")

        # Check dirty status and count changed files in a single pass
        # (avoids running git status twice per repo)
        local_status="clean"
        local_changed=0
        if [ "$VERBOSE" = true ] || [ "$JSON_OUTPUT" = true ]; then
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
            if [ "$local_changed" -gt 0 ]; then
                local_status="dirty"
                ((DIRTY_COUNT++)) || true
            fi
        else
            # Text mode without --verbose: just check dirty, skip counting
            if wt_layer_is_dirty "$layer_wt"; then
                local_status="dirty"
                ((DIRTY_COUNT++)) || true
            fi
        fi

        # Determine layer name from the non-symlinked *_ws directory
        local_layer=""
        for ws_dir in "$layer_wt"/*_ws; do
            [ -d "$ws_dir" ] || continue
            [ -L "$ws_dir" ] && continue
            local_layer="$(basename "$ws_dir" _ws)"
            break
        done

        # Collect JSON entry
        JSON_ENTRIES+=("$(build_json_entry "layer" "$local_issue" "$local_skill" "$layer_wt" "${local_branch:-}" "$local_status" "$local_repo" "$local_layer" "$local_changed")")

        # Format text output
        if [ "$JSON_OUTPUT" = false ]; then
            if [ -n "$local_skill" ]; then
                echo "[layer] Skill: $local_skill (layer) - Repository: $local_repo"
            else
                echo "[layer] Issue #$local_issue (layer) - Repository: $local_repo"
            fi
            echo "   Path:   $layer_wt"
            echo "   Branch: ${local_branch:-unknown}"
            echo "   Status: $local_status"

            if [ "$VERBOSE" = true ]; then
                echo "   Files changed: $local_changed"
            fi

            echo ""
        fi
        ((LAYER_COUNT++)) || true
    done
fi

# --- Output ---

if [ "$JSON_OUTPUT" = true ]; then
    # Exclude main worktree from total so layer + workspace == total
    TOTAL=$(( LAYER_COUNT + WORKSPACE_COUNT ))

    # Build JSON array
    printf '{"worktrees":['
    first=true
    for entry in "${JSON_ENTRIES[@]}"; do
        if [ "$first" = true ]; then
            first=false
        else
            printf ','
        fi
        printf '%s' "$entry"
    done
    printf '],"summary":{"total":%d,"layer":%d,"workspace":%d,"dirty":%d}}\n' \
        "$TOTAL" "$LAYER_COUNT" "$WORKSPACE_COUNT" "$DIRTY_COUNT"
    exit 0
fi

# Text output continues below

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
