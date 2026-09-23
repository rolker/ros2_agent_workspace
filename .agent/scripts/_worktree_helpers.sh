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

# Check whether `dir` is a git worktree that belongs to `main_repo_dir` —
# i.e. is `main_repo_dir` itself, or a LINKED worktree of that SAME repo,
# per `main_repo_dir`'s own worktree registry (#659). This is an identity
# check, not just an "is dir some live git checkout" check: a standalone git
# repo (rogue or coincidental) placed at the worktree path shape, or a
# worktree of a DIFFERENT repo, satisfies neither test below and is
# excluded, even though `git -C dir worktree list --porcelain` on its own
# would trivially list `dir` itself (any git repo is a trivial "worktree" of
# itself). Two independent checks, both required:
#
#   1. `dir` and `main_repo_dir` share the same `--git-common-dir` — the one
#      admin directory a main checkout and its linked worktrees all point
#      into. A standalone repo (rogue or otherwise) has its OWN commondir
#      and never matches this, regardless of what its own `worktree list`
#      says about itself.
#   2. `main_repo_dir`'s OWN `worktree list --porcelain` — queried from
#      `main_repo_dir`, never from `dir` — still names `dir`. This is the
#      authoritative registry: querying it from `main_repo_dir` rather than
#      `dir` matters only defensively (both should agree, since they share
#      a commondir per check 1), but anchoring the query at the trusted side
#      keeps the check meaningful even if check 1's comparison were ever
#      weakened.
#
# Fails closed on every other condition too: `main_repo_dir` missing or not
# a readable git checkout — including the case #659's checkpoint asked to
# be decided explicitly: the corresponding `layers/main/*_ws/src/<repo>`
# repo does not exist at all, e.g. a stale worktree path outliving a rename
# or removal of the main repo — a deregistered admin dir behind a dangling
# `.git` gitdir pointer on either side (the shape a `merge_pr.sh` exit-3
# worktree-removal failure or an interrupted removal leaves), or an
# unreadable `worktree list`. Any of these counts as "not registered to
# this repo", and the caller should EXCLUDE the directory rather than
# include it.
#
# Usage: if wt_is_registered "$dir" "$main_repo_dir"; then ... fi
wt_is_registered() {
    local dir="$1"
    local main_repo_dir="$2"

    local real_dir
    real_dir="$(CDPATH='' cd -- "$dir" 2>/dev/null && pwd -P)" || return 1

    local main_real
    main_real="$(CDPATH='' cd -- "$main_repo_dir" 2>/dev/null && pwd -P)" || return 1
    git -C "$main_real" rev-parse --git-dir &>/dev/null || return 1

    # Check 1: same git-common-dir. `--path-format=absolute` makes the
    # comparison well-defined regardless of which directory git would
    # otherwise print the path relative to (established pattern in this
    # workspace: dashboard.sh, resolve_repo_checkout.sh, workspace_root.sh).
    local main_common dir_common
    main_common="$(git -C "$main_real" rev-parse --path-format=absolute --git-common-dir 2>/dev/null)" || return 1
    dir_common="$(git -C "$dir" rev-parse --path-format=absolute --git-common-dir 2>/dev/null)" || return 1
    [ -n "$main_common" ] && [ -n "$dir_common" ] || return 1
    main_common="$(CDPATH='' cd -- "$main_common" 2>/dev/null && pwd -P)" || return 1
    dir_common="$(CDPATH='' cd -- "$dir_common" 2>/dev/null && pwd -P)" || return 1
    [ "$dir_common" = "$main_common" ] || return 1

    # Check 2: main_repo_dir's own registry still names dir.
    local list line path
    list="$(git -C "$main_real" worktree list --porcelain 2>/dev/null)" || return 1
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
# wt_is_registered confirms it is a linked worktree of the CORRESPONDING
# `layers/main/<ws>/src/<pkg>` repo — same layer-workspace name and package
# name, exactly the checkout worktree_create.sh's `git worktree add` (run
# from that same `layers/main` package directory) would have produced.
# Otherwise it is skipped with a named reason (never silently included —
# see wt_is_registered above), covering two distinct cases: the
# corresponding `layers/main` repo doesn't exist at all, or it exists but
# does not register `pkg_dir` as one of its own worktrees (a rogue/coincidental
# standalone git repo at the worktree path shape, a worktree of some OTHER
# repo, or a genuinely leftover/deregistered directory).
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

                local main_pkg_dir
                main_pkg_dir="$root_dir/layers/main/$(basename "$ws_dir")/src/$(basename "$pkg_dir")"

                if [ ! -d "$main_pkg_dir" ]; then
                    echo "Note: '$yaml' skipped — no corresponding" \
                         "'$main_pkg_dir' repo exists (stale worktree path" \
                         "outliving a rename/removal of the main repo)." >&2
                elif wt_is_registered "$pkg_dir" "$main_pkg_dir"; then
                    echo "$yaml"
                else
                    echo "Note: '$yaml' skipped — '$pkg_dir' is not a" \
                         "git-registered linked worktree of" \
                         "'$main_pkg_dir' (leftover/deregistered directory," \
                         "or a standalone/unrelated git repo at this path)." >&2
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
