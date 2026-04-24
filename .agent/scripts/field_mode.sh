#!/bin/bash
# .agent/scripts/field_mode.sh
# Detect whether the current repo is in "field mode" vs. "dev mode".
#
# Field mode = the repo's origin is NOT github.com (e.g., gitcloud, a private
# Forgejo, or another non-github remote). In field mode, agents may edit
# tracked files in the main tree and commit directly to the default branch
# without opening a PR. See issue #445 for rationale.
#
# Dev mode = the repo's origin is github.com. Dev mode enforces the worktree
# + PR workflow.
#
# Usage:
#   # As CLI (exits 0 for field mode, 1 for dev mode):
#   .agent/scripts/field_mode.sh
#   .agent/scripts/field_mode.sh --describe   # human-readable summary
#
#   # Sourced:
#   source .agent/scripts/field_mode.sh
#   if is_field_mode; then
#     echo "field mode — direct main-tree edits permitted"
#   fi

# is_field_mode [repo_dir]
# Exit 0 if the given repo (default: $PWD) is in field mode, else 1.
#
# A repo is in dev mode iff its origin host is exactly github.com.
# The host is matched only when preceded by start-of-string, '/', or '@'
# AND followed by ':' or '/', so hostnames like 'mygithub.com' or
# 'github.company.internal' are NOT misclassified as dev mode.
is_field_mode() {
    local repo_dir="${1:-$PWD}"
    local origin_url
    origin_url=$(git -C "$repo_dir" remote get-url origin 2>/dev/null) || return 1
    [[ ! "$origin_url" =~ (^|/|@)github\.com[:/] ]]
}

# describe_mode [repo_dir]
# Print a human-readable mode summary to stdout.
#
# Uses `|| true` on the git call so that callers running with `set -e`
# don't exit before the empty-string check; we want to print a message
# and return 1 in that case, not silently abort the parent shell.
describe_mode() {
    local repo_dir="${1:-$PWD}"
    local origin_url
    origin_url=$(git -C "$repo_dir" remote get-url origin 2>/dev/null || true)
    if [ -z "$origin_url" ]; then
        echo "not a git repo or no 'origin' remote: $repo_dir"
        return 1
    fi

    if is_field_mode "$repo_dir"; then
        echo "field mode  (origin: $origin_url)"
    else
        echo "dev mode    (origin: $origin_url)"
    fi
}

# If invoked directly (not sourced), run as CLI
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    case "${1:-}" in
        --describe)
            shift
            describe_mode "$@"
            ;;
        *)
            is_field_mode "$@"
            ;;
    esac
fi
