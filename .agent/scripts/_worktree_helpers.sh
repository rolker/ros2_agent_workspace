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

# Check whether `dir` is still a git worktree that git's OWN worktree
# registry recognizes for its repo (main checkout or any linked worktree)
# (#659). Fails closed: any git failure — not a git checkout at all, a
# deregistered admin dir behind a dangling `.git` gitdir pointer (the shape a
# merge_pr.sh exit-3 worktree-removal failure or an interrupted removal
# leaves), an unreadable `worktree list` — counts as "not registered", and
# the caller should EXCLUDE the directory rather than include it.
#
# `git worktree list` enumerates every worktree of the repository (main +
# linked) regardless of which one it is invoked FROM, so running it from
# `dir` itself is sufficient — there is no need to separately locate the
# repo's main checkout. This also collapses two checks into one command: if
# `dir`'s `.git` gitdir pointer can't be resolved (its admin directory under
# the main checkout's `.git/worktrees/<name>` was removed), the command
# itself fails rather than merely omitting `dir` from its output — "is this a
# live git checkout at all" and "is it still registered" fail the same way.
#
# Usage: if wt_is_registered "$dir"; then ... fi
wt_is_registered() {
    local dir="$1"
    local real_dir
    real_dir="$(CDPATH='' cd -- "$dir" 2>/dev/null && pwd -P)" || return 1

    local list
    list="$(git -C "$dir" worktree list --porcelain 2>/dev/null)" || return 1

    local line path
    while IFS= read -r line; do
        case "$line" in
            "worktree "*)
                path="${line#worktree }"
                # Resolve for a like-for-like compare: `worktree list` may
                # print the path in a different (but equivalent) form.
                path="$(CDPATH='' cd -- "$path" 2>/dev/null && pwd -P)" || continue
                [ "$path" = "$real_dir" ] && return 0
                ;;
        esac
    done <<< "$list"
    return 1
}

# Discover every project repo's root rosdep.yaml under BOTH layers/main and
# registered layer worktrees (#659). Prints one absolute path per line
# (unsorted — callers sort as needed); "Note:" lines for a skipped worktree
# entry go to stderr, never stdout.
#
# layers/main is globbed unconditionally, exactly as before #659. Under
# layers/worktrees/*, a `*_ws` directory or a `src/*` package entry that is a
# SYMLINK is skipped, following the wt_layer_* convention above:
# worktree_create.sh symlinks every sibling package not in the worktree's own
# --packages, and every non-target layer wholesale, straight back to
# layers/main — walking through those transparently (bash's `*` glob follows
# symlinked directory components) would rediscover layers/main content a
# second time under a different path, once per active worktree that happens
# to include that layer/package.
#
# A surviving package directory's rosdep.yaml is admitted only when
# wt_is_registered confirms git still knows about that worktree; otherwise
# it is skipped with a named reason (never silently included — see
# wt_is_registered above).
#
# Requires `shopt -s nullglob` to already be set in the calling shell (every
# caller sets it before its own glob use; this function relies on that
# rather than toggling the option itself, to avoid a surprising global
# side effect from a plain function call).
#
# Usage: wt_discover_local_rosdep_yamls "$root_dir"
wt_discover_local_rosdep_yamls() {
    local root_dir="$1"
    local yaml

    for yaml in "$root_dir"/layers/main/*_ws/src/*/rosdep.yaml; do
        echo "$yaml"
    done

    local wt_dir ws_dir src_dir pkg_dir
    for wt_dir in "$root_dir"/layers/worktrees/*; do
        [ -d "$wt_dir" ] || continue

        for ws_dir in "$wt_dir"/*_ws; do
            [ -d "$ws_dir" ] || continue
            [ -L "$ws_dir" ] && continue  # skip symlinked layers

            src_dir="$ws_dir/src"
            [ -d "$src_dir" ] || continue

            for pkg_dir in "$src_dir"/*; do
                [ -d "$pkg_dir" ] || continue
                [ -L "$pkg_dir" ] && continue  # skip symlinked packages

                yaml="$pkg_dir/rosdep.yaml"
                [ -f "$yaml" ] || continue

                if wt_is_registered "$pkg_dir"; then
                    echo "$yaml"
                else
                    echo "Note: '$yaml' skipped — '$pkg_dir' is not a" \
                         "git-registered worktree (leftover or deregistered" \
                         "directory)." >&2
                fi
            done
        done
    done
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
