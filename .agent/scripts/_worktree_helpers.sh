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

# Get the directory of the first inner package git worktree in a layer worktree.
# Use this to find the package repo whose origin / git-bug data should be
# queried for issue metadata (the issue lives in the package's repo, not
# the workspace).
# Prints the package dir on stdout (with exit status 0); prints nothing
# and returns a non-zero status if no inner git worktree is found.
# Usage: pkg_dir=$(wt_layer_pkg_dir "$worktree_dir")
wt_layer_pkg_dir() {
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
                echo "$pkg_dir"
                return 0
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

# Extract a validated owner/repo slug from a GitHub remote URL.
# Prints the slug on stdout; prints nothing for non-GitHub or malformed URLs.
#
# Supported URL forms:
#   https://github.com/OWNER/REPO[.git]
#   https://github.com:PORT/OWNER/REPO[.git]
#   git@github.com:OWNER/REPO[.git]                        (SCP form)
#   ssh://[user@]github.com[:PORT]/OWNER/REPO[.git]
#   ssh://[user@]ssh.github.com:443/OWNER/REPO[.git]       (SSH-over-443)
#
# Rejects substring/lookalike hosts (e.g. mygithub.com, gist.github.com)
# and `github.com` appearing inside the URL path by anchoring the match
# at the start of the string and requiring the host to be at a true URL
# host position — start, after a single `[user@]` auth section, or after
# the `://` protocol delimiter.
#
# Usage: slug=$(extract_gh_slug "$url")
extract_gh_slug() {
    local url="$1"
    # Strip a single trailing .git so the regexes don't have to.
    local cleaned="${url%.git}"
    # Two anchored patterns covering the officially supported remote URL
    # forms. Anchoring at ^ rejects lookalike hosts and `@github.com`
    # appearing inside a URL path (e.g.
    # `git@example.com:foo@github.com/owner/repo`).
    #
    # Form 1: explicit scheme (https, http, ssh).
    #   ^(https?|ssh)://[user@]?(ssh\.)?github.com[:port]?/OWNER/REPO$
    local re_url='^(https?|ssh)://([^@/[:space:]]+@)?(ssh\.)?github\.com(:[0-9]+)?/([^/[:space:]]+)/([^/[:space:]]+)$'
    # Form 2: SCP-style `[user@]host:path` (no scheme, no slash between
    # host and path).
    #   ^[user@]?(ssh\.)?github.com:OWNER/REPO$
    local re_scp='^([^@/[:space:]]+@)?(ssh\.)?github\.com:([^/[:space:]]+)/([^/[:space:]]+)$'
    if [[ "$cleaned" =~ $re_url ]]; then
        echo "${BASH_REMATCH[5]}/${BASH_REMATCH[6]}"
    elif [[ "$cleaned" =~ $re_scp ]]; then
        echo "${BASH_REMATCH[3]}/${BASH_REMATCH[4]}"
    fi
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
    # Use an array for the glob to avoid word-splitting issues
    local -a glob_patterns
    if [ -n "$repo_slug" ]; then
        glob_patterns=( "$base_dir"/skill-"${repo_slug}"-"${skill}"-* )
    else
        glob_patterns=( "$base_dir"/skill-*-"${skill}"-* )
    fi
    for path in "${glob_patterns[@]}"; do
        # When glob doesn't match, bash returns the literal pattern
        if [ -d "$path" ]; then
            matches+=( "$path" )
        fi
    done

    if [ "${#matches[@]}" -eq 0 ]; then
        return 1
    fi

    if [ "${#matches[@]}" -gt 1 ]; then
        echo "Warning: multiple skill worktrees found for '$skill'; using most recent" >&2
    fi

    # Find the most recent by comparing the timestamp suffix in the basename,
    # not the full path (which includes repo_slug and can sort incorrectly)
    local latest_path="" latest_ts=""
    for path in "${matches[@]}"; do
        local basename="${path##*/}"
        # Basename format: skill-{REPO_SLUG}-{SKILL}-{TIMESTAMP}
        # Extract timestamp: everything after the last occurrence of -{skill}-
        local ts="${basename##*-"${skill}"-}"
        if [ -z "$latest_ts" ] || [[ "$ts" > "$latest_ts" ]]; then
            latest_ts="$ts"
            latest_path="$path"
        fi
    done

    echo "$latest_path"
    return 0
}
