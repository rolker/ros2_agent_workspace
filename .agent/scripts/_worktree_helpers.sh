#!/bin/bash
# .agent/scripts/_worktree_helpers.sh
# Shared helper functions for worktree scripts
#
# Source this file from other worktree scripts:
#   source "$SCRIPT_DIR/_worktree_helpers.sh"

# Get branch name from the first inner package git worktree in a layer worktree.
# Returns empty string if no inner git worktree is found.
# Usage: branch=$(wt_layer_branch "$worktree_dir")
wt_layer_branch() {
    local worktree_dir="$1"

    for ws_dir in "$worktree_dir"/*_ws; do
        [ -d "$ws_dir" ] || continue
        [ -L "$ws_dir" ] && continue  # skip symlinked layers

        local src_dir="$ws_dir/src"
        [ -d "$src_dir" ] || continue

        for pkg_dir in "$src_dir"/*; do
            [ -d "$pkg_dir" ] || continue
            [ -L "$pkg_dir" ] && continue  # skip symlinked packages

            if git -C "$pkg_dir" rev-parse --git-dir &>/dev/null; then
                local branch
                branch=$(git -C "$pkg_dir" branch --show-current 2>/dev/null)
                if [ -n "$branch" ]; then
                    echo "$branch"
                    return 0
                fi
            fi
        done
    done

    return 1
}

# Check if a layer worktree has uncommitted changes in any inner package git worktree.
# Ignores symlinked layers/packages and infrastructure directories.
# Returns 0 (true) if dirty, 1 (false) if clean.
# Usage: if wt_layer_is_dirty "$worktree_dir"; then ...
wt_layer_is_dirty() {
    local worktree_dir="$1"

    for ws_dir in "$worktree_dir"/*_ws; do
        [ -d "$ws_dir" ] || continue
        [ -L "$ws_dir" ] && continue  # skip symlinked layers

        local src_dir="$ws_dir/src"
        [ -d "$src_dir" ] || continue

        for pkg_dir in "$src_dir"/*; do
            [ -d "$pkg_dir" ] || continue
            [ -L "$pkg_dir" ] && continue  # skip symlinked packages

            if git -C "$pkg_dir" rev-parse --git-dir &>/dev/null; then
                if [ -n "$(git -C "$pkg_dir" status --porcelain 2>/dev/null)" ]; then
                    return 0  # dirty
                fi
            fi
        done
    done

    return 1  # clean
}

# Find the most recent skill worktree matching a skill name.
# Skill worktree dirs are named: skill-{REPO_SLUG}-{SKILL}-{TIMESTAMP}
# Usage: path=$(find_worktree_by_skill "$base_dir" "$skill_name" ["$repo_slug"])
# Optional repo_slug filters to a specific repository.
find_worktree_by_skill() {
    local base_dir="$1"
    local skill="$2"
    local repo_slug="${3:-}"

    local matches=()
    local pattern
    if [ -n "$repo_slug" ]; then
        pattern="$base_dir/skill-${repo_slug}-${skill}-*"
    else
        pattern="$base_dir/skill-*-${skill}-*"
    fi
    for path in $pattern; do
        if [ -d "$path" ] && [ "$path" != "$pattern" ]; then
            matches+=( "$path" )
        fi
    done

    if [ "${#matches[@]}" -eq 0 ]; then
        return 1
    fi

    if [ "${#matches[@]}" -gt 1 ]; then
        echo "Warning: multiple skill worktrees found for '$skill'; using most recent" >&2
    fi

    # Return the most recent (last in sorted order, since timestamp is in the name)
    local sorted
    sorted=$(printf '%s\n' "${matches[@]}" | sort)
    echo "$sorted" | tail -n1
    return 0
}
