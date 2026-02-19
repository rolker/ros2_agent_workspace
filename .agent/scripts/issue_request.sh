#!/bin/bash
# .agent/scripts/issue_request.sh
# Container-side script to queue GitHub issue creation for host-side processing.
#
# Writes a JSON signal file to .agent/scratchpad/issue-requests/<worktree_id>/
# that the host-side push_gateway.sh reads after the container exits.
#
# Usage:
#   issue_request.sh --repo <owner/repo> --title "title" [--body "body"] [--labels "l1,l2"]
#   issue_request.sh --repo <owner/repo> --title "title" --body-file <path>

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

# ---------- Argument parsing ----------

REPO_SLUG=""
ISSUE_TITLE=""
ISSUE_BODY=""
ISSUE_BODY_FILE=""
LABELS=""

show_usage() {
    cat <<'EOF'
Usage: issue_request.sh --repo <owner/repo> --title "title" [OPTIONS]

Queue a GitHub issue for host-side creation after container exit.

Required:
  --repo <owner/repo>   Target repository (e.g., rolker/unh_marine_autonomy)
  --title <text>        Issue title

Options:
  --body <text>         Issue body (short text)
  --body-file <path>    Path to file containing issue body (markdown)
  --labels <l1,l2>      Comma-separated labels
  -h, --help            Show this help

The script writes a JSON signal file that the host reads after container exit.
Multiple issues can be queued per session — each gets a sequential number.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --repo)
            REPO_SLUG="$2"; shift 2 ;;
        --title)
            ISSUE_TITLE="$2"; shift 2 ;;
        --body)
            ISSUE_BODY="$2"; shift 2 ;;
        --body-file)
            ISSUE_BODY_FILE="$2"; shift 2 ;;
        --labels)
            LABELS="$2"; shift 2 ;;
        -h|--help)
            show_usage; exit 0 ;;
        *)
            echo "Unknown option: $1" >&2
            show_usage >&2
            exit 1 ;;
    esac
done

if [ -z "$REPO_SLUG" ]; then
    echo "ERROR: --repo is required." >&2
    show_usage >&2
    exit 1
fi

if [ -z "$ISSUE_TITLE" ]; then
    echo "ERROR: --title is required." >&2
    show_usage >&2
    exit 1
fi

# ---------- Resolve identity ----------

WORKTREE_ID="${WORKTREE_ID:-}"
if [ -z "$WORKTREE_ID" ]; then
    echo "ERROR: WORKTREE_ID not set. This script must run inside a container launched by docker_run_agent.sh." >&2
    exit 1
fi

SOURCE_ISSUE="${WORKTREE_ISSUE:-unknown}"

# ---------- Resolve body ----------

if [ -n "$ISSUE_BODY_FILE" ]; then
    if [ ! -f "$ISSUE_BODY_FILE" ]; then
        echo "ERROR: Body file not found: $ISSUE_BODY_FILE" >&2
        exit 1
    fi
    ISSUE_BODY="$(cat "$ISSUE_BODY_FILE")"
fi

# ---------- Write signal file ----------

ISSUE_DIR="$ROOT_DIR/.agent/scratchpad/issue-requests/$WORKTREE_ID"
mkdir -p "$ISSUE_DIR"

# Sequential numbering: count existing .json files and increment
EXISTING=$(find "$ISSUE_DIR" -maxdepth 1 -name '*.json' 2>/dev/null | wc -l)
SEQ=$(printf '%03d' $((EXISTING + 1)))

# Generate slug from title (lowercase, spaces to hyphens, strip non-alnum)
SLUG=$(printf '%s' "$ISSUE_TITLE" | tr '[:upper:]' '[:lower:]' | tr ' ' '-' | sed 's/[^a-z0-9-]//g' | cut -c1-40)

SIGNAL_FILE="$ISSUE_DIR/${SEQ}-${SLUG}.json"
BODY_FILE_PATH="$ISSUE_DIR/${SEQ}-${SLUG}-body.md"

# Write body to separate file
if [ -n "$ISSUE_BODY" ]; then
    printf '%s\n' "$ISSUE_BODY" > "$BODY_FILE_PATH"
else
    BODY_FILE_PATH=""
fi

# Convert labels to JSON array
if [ -n "$LABELS" ]; then
    LABELS_JSON=$(printf '%s' "$LABELS" | jq -Rs '[split(",") | .[] | ltrimstr(" ") | rtrimstr(" ")]')
else
    LABELS_JSON="[]"
fi

# Write JSON signal
cat > "$SIGNAL_FILE" <<ENDJSON
{
  "worktree_id": "$WORKTREE_ID",
  "source_issue": "$SOURCE_ISSUE",
  "repo_slug": "$REPO_SLUG",
  "title": $(printf '%s' "$ISSUE_TITLE" | jq -Rs .),
  "body_file": "$BODY_FILE_PATH",
  "labels": $LABELS_JSON,
  "requested_at": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
  "status": "pending"
}
ENDJSON

echo ""
echo "========================================="
echo "  Issue Request Queued"
echo "========================================="
echo "  Repo:      $REPO_SLUG"
echo "  Title:     $ISSUE_TITLE"
echo "  Labels:    ${LABELS:-none}"
echo "  Sequence:  $SEQ"
echo "  Signal:    $SIGNAL_FILE"
echo "========================================="
echo ""
echo "Exit the container to process queued issues on the host."
