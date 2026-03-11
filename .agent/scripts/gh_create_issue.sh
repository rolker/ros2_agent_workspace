#!/bin/bash
# Helper script for creating GitHub issues with label validation
# Usage: gh_create_issue.sh [gh issue create options]
#
# This script wraps 'gh issue create' and validates labels against .agent/github_metadata.json
# before calling the GitHub CLI. This prevents failures from invalid label names.
#
# Examples:
#   # Create issue with labels
#   .agent/scripts/gh_create_issue.sh --title "Fix bug" --body "Description" --label "bug" --label "enhancement"
#
#   # Create issue with body from file
#   .agent/scripts/gh_create_issue.sh --title "My Issue" --body-file /tmp/body.md --label "documentation"
#
#   # Interactive mode (no validation since labels selected from GitHub)
#   .agent/scripts/gh_create_issue.sh
#
# Note: Labels with spaces should be quoted: --label "good first issue"
#
# Exit codes:
#   0 - Success
#   1 - Invalid label detected
#   2 - GitHub CLI error
#   3 - Missing dependencies

if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
    echo "Error: This script should be executed, not sourced."
    echo "  Run: ${BASH_SOURCE[0]} $*"
    return 1
fi
set -euo pipefail

# Get repository root
REPO_ROOT="$(git rev-parse --show-toplevel 2>/dev/null)" || {
    echo "❌ Error: Not in a git repository"
    exit 3
}

METADATA_FILE="$REPO_ROOT/.agent/github_metadata.json"

# Check for required dependencies
if ! command -v gh &> /dev/null; then
    echo "❌ Error: 'gh' (GitHub CLI) is not installed or not in PATH"
    echo "   Install from: https://cli.github.com/"
    exit 3
fi

if ! command -v jq &> /dev/null; then
    echo "❌ Error: 'jq' is not installed or not in PATH"
    echo "   Install with: sudo apt-get install jq (or brew install jq)"
    exit 3
fi

# Parse command line arguments to extract labels
LABELS=()

# Preserve original arguments for passing to gh
ORIGINAL_ARGS=("$@")

# Single pass to extract labels
while [ $# -gt 0 ]; do
    case "$1" in
        --label|-l)
            if [ -n "${2:-}" ]; then
                LABELS+=("$2")
                shift 2
            else
                shift
            fi
            ;;
        --label=*)
            LABEL_VALUE="${1#*=}"
            LABELS+=("$LABEL_VALUE")
            shift
            ;;
        *)
            shift
            ;;
    esac
done

# Auto-inject parent issue reference when $WORKTREE_ISSUE is set
if [ -n "${WORKTREE_ISSUE:-}" ]; then
    # Determine the correct issue reference format (cross-repo vs same-repo)
    PARENT_REF="#${WORKTREE_ISSUE}"
    CURRENT_REPO_SLUG=$(gh repo view --json nameWithOwner --jq '.nameWithOwner' 2>/dev/null || echo "")
    WORKSPACE_REPO_SLUG=""
    WORKSPACE_ROOT="${ROS2_WORKSPACE_ROOT:-}"
    if [ -n "$WORKSPACE_ROOT" ] && git -C "$WORKSPACE_ROOT" remote get-url origin &>/dev/null; then
        _WS_URL=$(git -C "$WORKSPACE_ROOT" remote get-url origin)
        WORKSPACE_REPO_SLUG=$(echo "$_WS_URL" | sed -E 's#.*github\.com[:/]##' | sed 's/\.git$//')
    fi
    if [ -n "$CURRENT_REPO_SLUG" ] && [ -n "$WORKSPACE_REPO_SLUG" ] && [ "$CURRENT_REPO_SLUG" != "$WORKSPACE_REPO_SLUG" ]; then
        PARENT_REF="${WORKSPACE_REPO_SLUG}#${WORKTREE_ISSUE}"
    fi

    # Check if body already references the parent issue
    BODY_TEXT=""
    BODY_FILE_PATH=""
    BODY_ARG_INDEX=-1
    BODY_FILE_ARG_INDEX=-1
    for (( i=0; i<${#ORIGINAL_ARGS[@]}; i++ )); do
        case "${ORIGINAL_ARGS[$i]}" in
            --body)
                if [ -n "${ORIGINAL_ARGS[$((i+1))]:-}" ]; then
                    BODY_TEXT="${ORIGINAL_ARGS[$((i+1))]}"
                    BODY_ARG_INDEX=$((i+1))
                fi
                ;;
            --body-file)
                if [ -n "${ORIGINAL_ARGS[$((i+1))]:-}" ]; then
                    BODY_FILE_PATH="${ORIGINAL_ARGS[$((i+1))]}"
                    BODY_FILE_ARG_INDEX=$((i+1))
                fi
                ;;
        esac
    done

    NEEDS_INJECTION=true
    # Use word-boundary regex to avoid substring false positives (e.g., #12 matching #123)
    _PARENT_PATTERN="(^|[^0-9])#${WORKTREE_ISSUE}([^0-9]|$)"
    if [ -n "$BODY_TEXT" ]; then
        if printf '%s\n' "$BODY_TEXT" | grep -Eq "$_PARENT_PATTERN"; then
            NEEDS_INJECTION=false
        fi
    elif [ -n "$BODY_FILE_PATH" ] && [ -f "$BODY_FILE_PATH" ]; then
        if grep -Eq "$_PARENT_PATTERN" "$BODY_FILE_PATH"; then
            NEEDS_INJECTION=false
        fi
    fi

    if [ "$NEEDS_INJECTION" = true ]; then
        if [ -n "$BODY_TEXT" ]; then
            # Append to --body value
            ORIGINAL_ARGS[$BODY_ARG_INDEX]="${BODY_TEXT}

Part of ${PARENT_REF}"
            echo "ℹ️  Auto-added parent reference: Part of ${PARENT_REF}"
        elif [ -n "$BODY_FILE_PATH" ] && [ -f "$BODY_FILE_PATH" ]; then
            # Create temp copy with appended reference; trap ensures cleanup on exit
            PARENT_BODY_FILE=$(mktemp /tmp/gh_body_parent.XXXXXX.md)
            trap 'rm -f "$PARENT_BODY_FILE"' EXIT
            cp "$BODY_FILE_PATH" "$PARENT_BODY_FILE"
            printf '\n\nPart of %s\n' "$PARENT_REF" >> "$PARENT_BODY_FILE"
            ORIGINAL_ARGS[$BODY_FILE_ARG_INDEX]="$PARENT_BODY_FILE"
            echo "ℹ️  Auto-added parent reference: Part of ${PARENT_REF}"
        else
            echo "⚠️  No --body or --body-file provided; cannot auto-inject parent reference (Part of ${PARENT_REF})"
        fi
    fi
fi

# If no labels specified or metadata file doesn't exist, just pass through to gh
if [ ${#LABELS[@]} -eq 0 ]; then
    echo "ℹ️  No labels specified, passing through to 'gh issue create'"
    gh issue create "${ORIGINAL_ARGS[@]}"; exit $?
fi

if [ ! -f "$METADATA_FILE" ]; then
    echo "⚠️  Warning: $METADATA_FILE not found"
    echo "   Skipping label validation. Labels will be validated by GitHub."
    echo "   To enable validation, create the metadata file with valid labels."
    gh issue create "${ORIGINAL_ARGS[@]}"; exit $?
fi

# Load valid labels from metadata
VALID_LABELS=$(jq -r '.labels[]' "$METADATA_FILE" 2>/dev/null) || {
    echo "⚠️  Warning: Failed to parse $METADATA_FILE"
    echo "   Skipping label validation."
    gh issue create "${ORIGINAL_ARGS[@]}"; exit $?
}

# Validate each label
INVALID_LABELS=()
for label in "${LABELS[@]}"; do
    if ! echo "$VALID_LABELS" | grep -Fxq "$label"; then
        INVALID_LABELS+=("$label")
    fi
done

# If invalid labels found, report and exit
if [ ${#INVALID_LABELS[@]} -gt 0 ]; then
    echo "❌ Invalid label(s) detected: ${INVALID_LABELS[*]}"
    echo ""
    echo "Valid labels (from $METADATA_FILE):"
    echo "$VALID_LABELS" | sed 's/^/  - /'
    echo ""
    echo "To add a new label to the repository:"
    echo "  1. Create it: gh label create '<name>' --description '<desc>' --color '<hex>'"
    echo "  2. Update $METADATA_FILE"
    echo ""
    echo "Or fetch current labels: gh label list --json name --jq '.[].name' | sort"
    exit 1
fi

# All labels valid, proceed with gh issue create
echo "✅ All labels valid, creating issue..."
exec gh issue create "${ORIGINAL_ARGS[@]}"
