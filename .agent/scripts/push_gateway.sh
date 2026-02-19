#!/bin/bash
# .agent/scripts/push_gateway.sh
# Host-side interactive script to process push requests from sandboxed containers.
#
# Scans .agent/scratchpad/push-requests/ for pending JSON signal files and
# presents each to the user for confirmation before pushing and creating PRs.
#
# Usage:
#   push_gateway.sh                  # Process all pending requests
#   push_gateway.sh --issue <N>      # Process specific issue only

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"
PUSH_DIR="$ROOT_DIR/.agent/scratchpad/push-requests"

# ---------- Argument parsing ----------

FILTER_ISSUE=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --issue)
            FILTER_ISSUE="$2"; shift 2 ;;
        -h|--help)
            echo "Usage: push_gateway.sh [--issue <N>]"
            echo ""
            echo "Process pending push requests from sandboxed agent containers."
            echo "Scans .agent/scratchpad/push-requests/ for pending signal files."
            echo ""
            echo "Options:"
            echo "  --issue <N>   Process only the request for issue N"
            echo "  -h, --help    Show this help"
            exit 0 ;;
        *)
            echo "Unknown option: $1" >&2
            exit 1 ;;
    esac
done

# ---------- Find pending requests ----------

if [ ! -d "$PUSH_DIR" ]; then
    echo "No push requests directory found."
    exit 0
fi

PENDING_FILES=()
if [ -n "$FILTER_ISSUE" ]; then
    # Process specific issue
    SIGNAL="$PUSH_DIR/$FILTER_ISSUE.json"
    if [ -f "$SIGNAL" ]; then
        STATUS=$(jq -r '.status // "pending"' "$SIGNAL" 2>/dev/null || echo "unknown")
        if [ "$STATUS" = "pending" ]; then
            PENDING_FILES+=("$SIGNAL")
        else
            echo "Push request for issue #$FILTER_ISSUE has status: $STATUS"
            exit 0
        fi
    else
        echo "No push request found for issue #$FILTER_ISSUE."
        exit 0
    fi
else
    # Scan all .json files for pending status
    for signal_file in "$PUSH_DIR"/*.json; do
        [ -f "$signal_file" ] || continue
        STATUS=$(jq -r '.status // "pending"' "$signal_file" 2>/dev/null || echo "unknown")
        if [ "$STATUS" = "pending" ]; then
            PENDING_FILES+=("$signal_file")
        fi
    done
fi

if [ ${#PENDING_FILES[@]} -eq 0 ]; then
    echo "No pending push requests."
    exit 0
fi

echo "========================================="
echo "  Push Gateway"
echo "========================================="
echo "  Pending requests: ${#PENDING_FILES[@]}"
echo ""

# ---------- Process each request ----------

for signal_file in "${PENDING_FILES[@]}"; do
    ISSUE=$(jq -r '.issue' "$signal_file")
    BRANCH=$(jq -r '.branch' "$signal_file")
    REPO_SLUG=$(jq -r '.repo_slug' "$signal_file")
    REPO_PATH=$(jq -r '.repo_path' "$signal_file")
    COMMIT_COUNT=$(jq -r '.commit_count' "$signal_file")
    PR_TITLE=$(jq -r '.pr_title' "$signal_file")
    BODY_FILE=$(jq -r '.pr_body_file // ""' "$signal_file")
    REQUESTED_AT=$(jq -r '.requested_at' "$signal_file")

    echo "-----------------------------------------"
    echo "  Issue:      #$ISSUE"
    echo "  Branch:     $BRANCH"
    echo "  Repo:       $REPO_SLUG"
    echo "  Commits:    $COMMIT_COUNT"
    echo "  PR Title:   $PR_TITLE"
    echo "  Requested:  $REQUESTED_AT"
    echo "-----------------------------------------"
    echo ""

    while true; do
        read -rp "Action: [y]es push / [d]iff / [s]kip / [c]ancel all? " choice
        case "$choice" in
            y|Y|yes)
                echo ""
                echo "Pushing $BRANCH to origin..."

                # Push from the repo that has the commits
                if ! git -C "$REPO_PATH" push -u origin "$BRANCH"; then
                    echo "ERROR: git push failed." >&2
                    echo "You may need to push manually from: $REPO_PATH" >&2
                    break
                fi

                echo "Push successful."
                echo ""

                # Create PR
                echo "Creating pull request..."
                PR_ARGS=(
                    pr create
                    --repo "$REPO_SLUG"
                    --head "$BRANCH"
                    --title "$PR_TITLE"
                )

                if [ -n "$BODY_FILE" ] && [ -f "$BODY_FILE" ]; then
                    PR_ARGS+=(--body-file "$BODY_FILE")
                else
                    PR_ARGS+=(--body "Closes #$ISSUE")
                fi

                if PR_URL=$(gh "${PR_ARGS[@]}" 2>&1); then
                    echo "PR created: $PR_URL"
                else
                    echo "WARNING: PR creation failed: $PR_URL" >&2
                    echo "You can create it manually:" >&2
                    echo "  gh pr create --repo $REPO_SLUG --head $BRANCH --title \"$PR_TITLE\"" >&2
                fi

                # Update signal status
                TMP=$(mktemp)
                jq '.status = "pushed"' "$signal_file" > "$TMP" && mv "$TMP" "$signal_file"
                echo ""
                break
                ;;

            d|D|diff)
                echo ""
                echo "Showing commits on $BRANCH:"
                echo ""
                git -C "$REPO_PATH" log --oneline "origin/$(git -C "$REPO_PATH" symbolic-ref refs/remotes/origin/HEAD 2>/dev/null | sed 's|refs/remotes/origin/||' || echo 'main')..HEAD" 2>/dev/null || \
                    git -C "$REPO_PATH" log --oneline -"$COMMIT_COUNT"
                echo ""
                echo "Diff:"
                git -C "$REPO_PATH" diff "origin/$(git -C "$REPO_PATH" symbolic-ref refs/remotes/origin/HEAD 2>/dev/null | sed 's|refs/remotes/origin/||' || echo 'main')..HEAD" 2>/dev/null || \
                    git -C "$REPO_PATH" diff HEAD~"$COMMIT_COUNT"..HEAD
                echo ""
                # Loop back to prompt
                ;;

            s|S|skip)
                echo "Skipped."
                TMP=$(mktemp)
                jq '.status = "skipped"' "$signal_file" > "$TMP" && mv "$TMP" "$signal_file"
                echo ""
                break
                ;;

            c|C|cancel)
                echo "Cancelled — remaining requests left as pending."
                exit 0
                ;;

            *)
                echo "Invalid choice. Use y/d/s/c."
                ;;
        esac
    done
done

echo "========================================="
echo "  Push gateway complete."
echo "========================================="
