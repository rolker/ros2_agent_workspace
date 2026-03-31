#!/bin/bash
# Shared helpers for looking up GitHub issues in the local git-bug cache.
#
# Source this file in scripts that need git-bug lookups:
#   source "$(dirname "${BASH_SOURCE[0]}")/gitbug_helpers.sh"
#
# All functions return empty string on failure — callers handle fallback.

# gitbug_lookup <repo_dir> <github_issue_num> <field>
#
# Look up a GitHub issue by number in the local git-bug cache.
# Searches the metadata.github-url field for a matching /issues/<N> suffix.
#
# Arguments:
#   repo_dir           - Git repo root containing the git-bug data
#   github_issue_num   - GitHub issue number (e.g., 418)
#   field              - One of: title, status, human_id
#
# Returns: field value on stdout, or empty string if not found.
#
# Requires: git-bug, python3 (both checked at call time)
gitbug_lookup() {
    local repo_dir="$1"
    local issue_num="$2"
    local field="$3"

    if ! command -v git-bug &>/dev/null || ! command -v python3 &>/dev/null; then
        return 1
    fi

    # Validate issue_num is numeric to prevent injection
    if ! [[ "$issue_num" =~ ^[0-9]+$ ]]; then
        return 1
    fi

    # Validate field is one of the allowed values
    case "$field" in
        title|status|human_id) ;;
        *) return 1 ;;
    esac

    local json_output
    json_output=$(git -C "$repo_dir" bug bug --format json 2>/dev/null) || return 1

    python3 -c "
import json, sys
try:
    data = json.loads(sys.stdin.read())
except (json.JSONDecodeError, ValueError):
    sys.exit(1)
num = sys.argv[1]
field = sys.argv[2]
for bug in data:
    url = bug.get('metadata', {}).get('github-url', '')
    if url.endswith('/issues/' + num):
        if field == 'title':
            print(bug.get('title', ''))
        elif field == 'status':
            print(bug.get('status', ''))
        elif field == 'human_id':
            print(bug.get('human_id', ''))
        sys.exit(0)
sys.exit(1)
" "$issue_num" "$field" <<< "$json_output" 2>/dev/null
}

# gitbug_has_bridge <repo_dir>
#
# Check if the repo has a git-bug bridge configured.
# Returns 0 if a bridge exists, 1 otherwise.
gitbug_has_bridge() {
    local repo_dir="$1"
    local output
    output=$(git -C "$repo_dir" bug bridge 2>/dev/null) || return 1
    [ -n "$output" ]
}

# gitbug_count_open <repo_dir>
#
# Count open bugs in the local git-bug cache.
# Prints the count on stdout, or returns 1 on failure.
gitbug_count_open() {
    local repo_dir="$1"
    local output
    output=$(git -C "$repo_dir" bug bug "status:open" 2>/dev/null) || return 1
    if [ -z "$output" ]; then
        echo "0"
    else
        printf '%s\n' "$output" | grep -c . || echo "0"
    fi
}
