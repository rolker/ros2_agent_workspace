#!/bin/bash
# .agent/scripts/workspace_root.sh
# Print the workspace root the caller should anchor everything at.
#
# Why: three skills (`audit-project`, `janitor-sweep`, `issue-triage`) opened
# with
#
#     ROOT=$(git rev-parse --path-format=absolute --git-common-dir) && ROOT=$(dirname "$ROOT")
#
# which answers with *whatever repo you are standing in*. From the workspace or
# a workspace worktree that is the workspace's main checkout — right. From a
# LAYER worktree you are inside the project repo, so it answers with the
# project repo's root, where `.agent/scripts/` and `configs/` do not exist:
# `$ROOT/.agent/scripts/resolve_repo_checkout.sh` is then unfindable, and
# `case "$REPO_PATH" in "$ROOT"/layers/*/src/*` misreads a genuine layer
# checkout as mode `clone`. Both failures are silent-ish and both were
# "documented" by telling the operator to remember. This resolves it instead.
#
# The rule, in order:
#   1. $WORKSPACE_ROOT, when set — validated, never taken on faith.
#   2. This script's own location: `.agent/scripts/` -> `.agent/` -> the root.
#      A skill that found this file found the workspace it belongs to.
#   3. From that root, hop to the MAIN checkout when it is a git worktree —
#      `layers/` and the `configs/manifest` symlink exist only there, and every
#      caller's contract says "anchored at the main workspace root, never the
#      current worktree". resolve_repo_checkout.sh makes the same `git
#      --git-common-dir` hop; here it is additionally gated on the destination
#      being a workspace root, so a hop that lands somewhere unusable keeps
#      the root found above rather than replacing it with one.
#      Note this redirects even an explicitly-set $WORKSPACE_ROOT when that
#      root is a worktree: the callers' contract is the main checkout, and a
#      worktree cannot satisfy it — the variable chooses WHICH workspace, not
#      which checkout of it.
#
# A workspace root is recognised by holding both `.agent/scripts/` and
# `configs/` — the two directories every caller addresses through it.
#
# Usage (executed; callers walk up for it because a script cannot be called
# from a directory that has not been found yet). This is the snippet the three
# skills carry verbatim — it starts at $WORKSPACE_ROOT when set, so that
# variable is a usable remedy from a directory with no workspace above it, and
# it stops at the FIRST copy found rather than walking past a workspace whose
# script refused:
#   d="${WORKSPACE_ROOT:-$(pwd)}"; ROOT=""
#   while [ "$d" != "/" ]; do
#       if [ -f "$d/.agent/scripts/workspace_root.sh" ]; then
#           ROOT=$(bash "$d/.agent/scripts/workspace_root.sh") || ROOT=""
#           break
#       fi
#       d=$(dirname "$d")
#   done
#
# Exit codes:
#   0  the root is on stdout
#   1  no usable workspace root: $WORKSPACE_ROOT was set but is not one, or
#      this script is not sitting in a workspace's `.agent/scripts/`
# Nothing is printed on stdout on a failure path (#609): a caller reads a root
# or nothing, never an empty string at exit 0.

set -uo pipefail

if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
    echo "workspace_root.sh must be executed, not sourced: ROOT=\$(.../workspace_root.sh)" >&2
    return 2 2>/dev/null || exit 2
fi

is_workspace_root() {
    [[ -d "$1/.agent/scripts" && -d "$1/configs" ]]
}

if [[ -n "${WORKSPACE_ROOT:-}" ]]; then
    if ! is_workspace_root "$WORKSPACE_ROOT"; then
        echo "workspace_root.sh: WORKSPACE_ROOT='$WORKSPACE_ROOT' has no .agent/scripts/ and configs/ — it is not a workspace root" >&2
        exit 1
    fi
    if ! root=$(cd "$WORKSPACE_ROOT" && pwd); then
        echo "workspace_root.sh: cannot enter WORKSPACE_ROOT='$WORKSPACE_ROOT'" >&2
        exit 1
    fi
else
    script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    root="$(cd "$script_dir/../.." && pwd)"
    if ! is_workspace_root "$root"; then
        echo "workspace_root.sh: $root (two levels above this script) has no .agent/scripts/ and configs/ — this copy of the script is not in a workspace" >&2
        exit 1
    fi
fi

# From a worktree, the common git dir points at the main checkout's .git. The
# main checkout is where `layers/` and the `configs/manifest` symlink live, so
# that — not the worktree — is what the callers anchor at. A non-git sandbox or
# a plain clone keeps the root found above.
if common_dir=$(git -C "$root" rev-parse --path-format=absolute --git-common-dir 2>/dev/null); then
    main_root="$(dirname "$common_dir")"
    if [[ -d "$main_root" ]] && is_workspace_root "$main_root"; then
        root="$main_root"
    fi
fi

printf '%s\n' "$root"
