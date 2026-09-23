#!/bin/bash
# .agent/scripts/_resolve_default_branch.sh
# Shared helper for resolving a repository's default branch.
#
# Source this file from other scripts:
#   source "$SCRIPT_DIR/_resolve_default_branch.sh"
#
# Exposes: resolve_default_branch [<repo-root>]
#   On success: echoes a ref usable for `git diff <ref>...HEAD` on stdout.
#               Prefers a local branch name (e.g. `main`); falls back to
#               `origin/<branch>` when the local branch is absent. No
#               trailing whitespace.
#   On failure: prints error to stderr and returns 1.
#
#   <repo-root> defaults to `git rev-parse --show-toplevel` of the current
#   working directory.
#
# Resolution order:
#   1. Per-project manifest hook (inert today; wires up when #172 lands).
#   2. `git symbolic-ref refs/remotes/origin/HEAD` (the configured default).
#   3. Literal `main` as last-ditch fallback.
#
# After picking a name, the helper verifies the ref is reachable locally
# or on `origin/`. Callers receive a ref that is safe to feed straight
# into `git diff <ref>...HEAD`.
#
# Rationale: workspace and project repos default to `main` today, but
# the workspace-improvements-cascade-to-projects principle and #172's
# upcoming per-project manifest mean we need a single resolution point
# rather than hardcoding `main` in every consumer.

resolve_default_branch() {
    local repo_root="${1:-}"
    if [ -z "$repo_root" ]; then
        repo_root=$(git rev-parse --show-toplevel 2>/dev/null) || {
            echo "ERROR: resolve_default_branch: not inside a git repository" >&2
            return 1
        }
    else
        # Validate explicit repo_root up-front so a bad path produces
        # a clear error instead of falling through to the
        # "main fallback didn't work" branch later.
        git -C "$repo_root" rev-parse --git-dir >/dev/null 2>&1 || {
            echo "ERROR: resolve_default_branch: not a git repository: $repo_root" >&2
            return 1
        }
    fi

    local branch=""
    local resolved_via=""

    # --- Step 1: per-project manifest hook ---
    # When #172's per-project manifest schema is decided, the read goes
    # here. The block is intentionally empty so resolution falls through
    # to git's symbolic-ref detection until then. Wire-up will look like:
    #
    #   if [ -f "$repo_root/.agent/manifest.yaml" ]; then
    #       branch=$(yq -r '.default_branch // ""' "$repo_root/.agent/manifest.yaml")
    #   fi

    # --- Step 2: git's configured default ---
    # Two-step capture (instead of `git ... | sed`) so callers running
    # under `set -e -o pipefail` (e.g. cross_model_review.sh) don't abort
    # here when origin/HEAD is unset — the failing symbolic-ref would
    # make the pipeline non-zero and short-circuit the step-3 fallback.
    # Parameter expansion replaces sed for the prefix strip.
    if [ -z "$branch" ]; then
        local symref=""
        symref=$(git -C "$repo_root" symbolic-ref refs/remotes/origin/HEAD 2>/dev/null) || symref=""
        if [ -n "$symref" ]; then
            branch="${symref#refs/remotes/origin/}"
            resolved_via="git symbolic-ref"
        fi
    fi

    # --- Step 3: hardcoded fallback ---
    if [ -z "$branch" ]; then
        branch="main"
        resolved_via="hardcoded fallback"
    fi

    # --- Verify the chosen ref is reachable ---
    if git -C "$repo_root" rev-parse --verify --quiet "$branch" >/dev/null 2>&1; then
        echo "$branch"
        return 0
    fi
    if git -C "$repo_root" rev-parse --verify --quiet "origin/$branch" >/dev/null 2>&1; then
        echo "origin/$branch"
        return 0
    fi

    {
        echo "ERROR: resolve_default_branch: '${branch}' (resolved via ${resolved_via})"
        echo "  is not reachable as a local ref or as origin/${branch}."
        echo ""
        if [ "$resolved_via" = "hardcoded fallback" ]; then
            echo "  Repo at '${repo_root}' has no origin/HEAD configured AND no 'main' branch."
            echo "  Either run: git remote set-head origin -a"
            echo "  Or pass an explicit base via the caller's --branch <ref> flag."
        else
            echo "  origin/HEAD points at '${branch}' but that ref does not resolve."
            echo "  Try: git fetch origin"
            echo "  Or pass an explicit base via the caller's --branch <ref> flag."
        fi
    } >&2
    return 1
}

# Warn if the file is executed directly — it only defines a function and
# must be sourced.
if [ "${BASH_SOURCE[0]}" = "${0}" ]; then
    echo "ERROR: _resolve_default_branch.sh must be sourced, not executed directly." >&2
    echo "Usage: source .agent/scripts/_resolve_default_branch.sh" >&2
    exit 1
fi
