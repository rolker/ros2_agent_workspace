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

# If no labels specified or metadata file doesn't exist, just pass through to gh
if [ ${#LABELS[@]} -eq 0 ]; then
    echo "ℹ️  No labels specified, passing through to 'gh issue create'"
    exec gh issue create "${ORIGINAL_ARGS[@]}"
fi

if [ ! -f "$METADATA_FILE" ]; then
    echo "⚠️  Warning: $METADATA_FILE not found"
    echo "   Skipping label validation. Labels will be validated by GitHub."
    echo "   To enable validation, create the metadata file with valid labels."
    exec gh issue create "${ORIGINAL_ARGS[@]}"
fi

# Load valid labels from metadata
VALID_LABELS=$(jq -r '.labels[]' "$METADATA_FILE" 2>/dev/null) || {
    echo "⚠️  Warning: Failed to parse $METADATA_FILE"
    echo "   Skipping label validation."
    exec gh issue create "${ORIGINAL_ARGS[@]}"
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
