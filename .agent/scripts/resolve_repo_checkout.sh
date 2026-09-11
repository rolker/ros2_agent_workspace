#!/bin/bash
# .agent/scripts/resolve_repo_checkout.sh
# Resolve a project repo to a usable on-disk checkout, without assuming that
# the layer tree (`layers/`) exists.
#
# Why: skills that audit a project repo (`audit-project`, and the
# `janitor-sweep` that chains it) located repos with
# `find layers/main/*/src/<repo>` — a hard dependency on a fully set-up layer
# tree. `layers/` is gitignored and absent in every worktree, fresh clone and
# container, so those callers silently had nothing to audit. This script gives
# them one resolution rule: use the layer checkout when there is one, otherwise
# shallow-clone the URL the workspace manifests already declare.
#
# Usage:
#   .agent/scripts/resolve_repo_checkout.sh <repo-name>
#
# Output (stdout, on success only):
#   <absolute-path><TAB><layer|clone>
#
# The mode field matters to callers: `clone` means there is no layer checkout,
# so layer-dependent checks (a `colcon test` run, "is it in the expected
# layer?") must report SKIPPED rather than OK.
#
# Exit codes — every failure is named, and none of them is an empty success
# (#609's false-green lesson):
#   0  resolved
#   2  usage error
#   3  no repo manifest configured at all (configs/manifest absent or holding
#      no .repos) — list_overlay_repos.py prints an empty list at exit 0 in
#      this state, which would otherwise read as "repo not found"
#   4  repo not listed in any manifest that WAS read
#   5  clone or refresh failed
#   6  manifest unreadable — list_overlay_repos.py itself failed
#
# Clones land in <main-workspace-root>/.agent/scratchpad/janitor-repos/<repo>
# (gitignored). The cache is anchored at the MAIN workspace root, not the
# current worktree, so every worktree on a host shares one cache.

set -uo pipefail

# Executed, not sourced.
if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
    echo "resolve_repo_checkout.sh must be executed, not sourced" >&2
    return 2 2>/dev/null || exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# .agent/scripts/ -> .agent/ -> workspace root
TREE_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

usage() {
    echo "usage: resolve_repo_checkout.sh <repo-name>" >&2
}

if [[ $# -ne 1 ]]; then
    usage
    exit 2
fi

REPO_NAME="$1"
if [[ -z "${REPO_NAME//[[:space:]]/}" || "$REPO_NAME" == -* ]]; then
    usage
    exit 2
fi

# Main workspace root: from a worktree, the common git dir points at the main
# checkout's .git. Falling back to the tree root covers a non-git sandbox and
# a plain (non-worktree) clone alike.
MAIN_ROOT="$TREE_ROOT"
if common_dir=$(git -C "$TREE_ROOT" rev-parse --path-format=absolute --git-common-dir 2>/dev/null); then
    candidate="$(dirname "$common_dir")"
    [[ -d "$candidate" ]] && MAIN_ROOT="$candidate"
fi

# --- (a) an existing layer checkout wins -------------------------------------
for candidate in "$MAIN_ROOT"/layers/main/*/src/"$REPO_NAME"; do
    if [[ -d "$candidate" ]]; then
        printf '%s\t%s\n' "$(cd "$candidate" && pwd)" "layer"
        exit 0
    fi
done

# --- (b) otherwise resolve the URL from the workspace manifests ---------------
LIST_SCRIPT="$MAIN_ROOT/.agent/scripts/list_overlay_repos.py"
if [[ ! -f "$LIST_SCRIPT" ]]; then
    echo "resolve_repo_checkout.sh: cannot find $LIST_SCRIPT" >&2
    exit 6
fi

if ! repos_json=$(python3 "$LIST_SCRIPT" --format json 2>&1); then
    echo "resolve_repo_checkout.sh: could not read the repo manifests: $repos_json" >&2
    exit 6
fi

# Zero repos enumerated is NOT "repo not found" — it means nothing is
# configured to look through. Reported as its own failure so a caller cannot
# render it as "nothing to audit".
repo_count=$(printf '%s' "$repos_json" | python3 -c 'import json,sys; print(len(json.load(sys.stdin)))' 2>/dev/null)
if [[ -z "$repo_count" ]]; then
    echo "resolve_repo_checkout.sh: manifest listing was not valid JSON" >&2
    exit 6
fi
if [[ "$repo_count" -eq 0 ]]; then
    echo "resolve_repo_checkout.sh: no repo manifest configured under $MAIN_ROOT/configs — run 'make setup-all'" >&2
    exit 3
fi

repo_url=$(printf '%s' "$repos_json" | python3 -c '
import json, sys
name = sys.argv[1]
for repo in json.load(sys.stdin):
    if repo.get("name") == name:
        print(repo.get("url", ""))
        break
' "$REPO_NAME")

if [[ -z "$repo_url" ]]; then
    echo "resolve_repo_checkout.sh: '$REPO_NAME' is not listed in any of the $repo_count configured repos" >&2
    exit 4
fi

# --- (c) shallow clone / refresh ---------------------------------------------
# --depth 1 only: a --filter=blob:none clone that still checks out a working
# tree refetches every blob immediately, so the filter would buy nothing here.
CACHE_DIR="$MAIN_ROOT/.agent/scratchpad/janitor-repos"
TARGET="$CACHE_DIR/$REPO_NAME"

if ! mkdir -p "$CACHE_DIR" 2>/dev/null; then
    echo "resolve_repo_checkout.sh: cannot create clone cache at $CACHE_DIR" >&2
    exit 5
fi

if [[ -d "$TARGET/.git" ]]; then
    if ! refresh_out=$(git -C "$TARGET" fetch --depth 1 origin 2>&1) \
       || ! reset_out=$(git -C "$TARGET" reset --hard FETCH_HEAD 2>&1); then
        echo "resolve_repo_checkout.sh: could not refresh the existing clone at $TARGET: ${reset_out:-$refresh_out}" >&2
        exit 5
    fi
else
    rm -rf "$TARGET"
    if ! clone_out=$(git clone --depth 1 "$repo_url" "$TARGET" 2>&1); then
        echo "resolve_repo_checkout.sh: clone of $repo_url failed: $clone_out" >&2
        rm -rf "$TARGET"
        exit 5
    fi
fi

printf '%s\t%s\n' "$TARGET" "clone"
exit 0
