#!/bin/bash
# .agent/scripts/progress_append.sh
# Prompt-free progress.md entry appender + committer (#594).
#
# Usage:
#   .agent/scripts/progress_append.sh [-C <dir>] <issue-number> \
#       [--title "<issue title>"] [--name "<agent name>" --email "<agent email>"] < entry.md
#
# Reads ONE progress entry from stdin — the first non-blank line must be an
# ADR-0013 "## <Entry Type>" heading — appends it to
# <repo-root>/.agent/work-plans/issue-<N>/progress.md (creating the file with
# frontmatter, and the heading from --title, if absent), then stages and
# commits ONLY that file as the agent identity with the fixed message
# "progress: <entry type> for #<N>".
#
# Scope discipline (what makes this script allowlist-safe, like dlog.sh):
#   - the target path is derived from the issue number, never taken from input
#   - the commit message is fixed apart from the entry-type slot
#   - `git commit -- <file>` commits only that path; a dirty index is untouched
#   - identity comes from $AGENT_NAME/$AGENT_EMAIL (set_git_identity_env.sh)
#     or --name/--email, and the script FAILS LOUD when unset — it never falls
#     back to human git config (which would trip check_pr_authors.py on
#     agent-convention branches)
#
# -C <dir> targets another worktree (the `git -C` pattern the skills use);
# default is the repo containing the current directory.
#
# Exit codes: 0 ok, 2 usage/validation error, 3 git failure.

if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
    echo "Error: This script should be executed, not sourced."
    echo "  Run: ${BASH_SOURCE[0]} $*"
    return 1
fi
set -uo pipefail

usage() {
    echo "Usage: $0 [-C <dir>] <issue-number> [--title <title>] [--name <name> --email <email>] < entry.md" >&2
    exit 2
}

DIR="."
TITLE=""
NAME="${AGENT_NAME:-}"
EMAIL="${AGENT_EMAIL:-}"
ISSUE=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        -C)      [[ $# -ge 2 ]] || usage; DIR="$2"; shift 2 ;;
        --title) [[ $# -ge 2 ]] || usage; TITLE="$2"; shift 2 ;;
        --name)  [[ $# -ge 2 ]] || usage; NAME="$2"; shift 2 ;;
        --email) [[ $# -ge 2 ]] || usage; EMAIL="$2"; shift 2 ;;
        -*)      usage ;;
        *)       [[ -n "$ISSUE" ]] && usage; ISSUE="$1"; shift ;;
    esac
done

[[ "$ISSUE" =~ ^[0-9]+$ ]] || { echo "error: issue number must be numeric (got '${ISSUE:-}')" >&2; exit 2; }
if [[ -z "$NAME" || -z "$EMAIL" ]]; then
    echo "error: agent identity unset — source set_git_identity_env.sh (exports AGENT_NAME/AGENT_EMAIL)" >&2
    echo "       or pass --name/--email. Refusing to fall back to human git config." >&2
    exit 2
fi

ROOT=$(git -C "$DIR" rev-parse --show-toplevel 2>/dev/null) \
    || { echo "error: '$DIR' is not inside a git repository" >&2; exit 2; }

ENTRY=$(cat)
FIRST=$(printf '%s\n' "$ENTRY" | sed -n '/[^[:space:]]/{p;q;}')
[[ -n "$FIRST" ]] || { echo "error: empty entry on stdin" >&2; exit 2; }
if [[ ! "$FIRST" =~ ^##\ [A-Za-z] ]]; then
    echo "error: entry must start with an ADR-0013 '## <Entry Type>' heading (got: $FIRST)" >&2
    exit 2
fi
ENTRY_TYPE="${FIRST#\#\# }"
# Strip any trailing whitespace run (a CR from a CRLF heading, or a padded
# heading) so it can't leak `\r`/double-space into the commit subject.
ENTRY_TYPE="${ENTRY_TYPE%"${ENTRY_TYPE##*[![:space:]]}"}"
TYPE_MSG=$(printf '%s' "$ENTRY_TYPE" | tr '[:upper:]' '[:lower:]')

FILE_REL=".agent/work-plans/issue-$ISSUE/progress.md"
FILE="$ROOT/$FILE_REL"
mkdir -p "$(dirname "$FILE")"
if [[ ! -f "$FILE" ]]; then
    {
        printf -- '---\nissue: %s\n---\n\n' "$ISSUE"
        if [[ -n "$TITLE" ]]; then
            printf '# Issue #%s — %s\n' "$ISSUE" "$TITLE"
        else
            printf '# Issue #%s\n' "$ISSUE"
        fi
    } > "$FILE"
fi
{ printf '\n'; printf '%s\n' "$ENTRY"; } >> "$FILE"

git -C "$ROOT" add -- "$FILE_REL" || exit 3
# Force the validated identity onto BOTH author and committer via the GIT_*
# env vars: ambient GIT_AUTHOR_*/GIT_COMMITTER_* (exported by
# set_git_identity_env.sh) outrank `-c user.name/email`, so `-c` alone can be
# silently overridden by a stale/human identity — the very case
# check_pr_authors.py trips on. Setting the four vars here makes the override
# authoritative; the `-c` flags remain as a fallback when GIT_* are unset.
# Note: `git commit -- <pathspec>` commits only that path; if a future
# pre-commit hook mutated progress.md and exited 0, the hook's edit would stay
# unstaged and the tree could desync from HEAD. No such hook exists today.
if ! GIT_AUTHOR_NAME="$NAME" GIT_AUTHOR_EMAIL="$EMAIL" \
     GIT_COMMITTER_NAME="$NAME" GIT_COMMITTER_EMAIL="$EMAIL" \
     git -C "$ROOT" -c user.name="$NAME" -c user.email="$EMAIL" \
        commit -m "progress: $TYPE_MSG for #$ISSUE" -- "$FILE_REL"; then
    echo "error: commit failed (pre-commit hook?) — $FILE_REL is appended and staged; fix and re-commit" >&2
    exit 3
fi
echo "appended + committed '$ENTRY_TYPE' entry to $FILE_REL"
