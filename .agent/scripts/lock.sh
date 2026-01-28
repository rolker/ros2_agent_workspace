#!/bin/bash
# scripts/lock.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"
LOCK_FILE="$ROOT_DIR/.agent/scratchpad/workspace.lock"

# Ensure .agent/scratchpad exists
mkdir -p "$ROOT_DIR/.agent/scratchpad"

if [ -f "$LOCK_FILE" ]; then
    echo "❌ Workspace is already LOCKED."
    echo "Lock info:"
    cat "$LOCK_FILE"
    exit 1
fi

REASON=${1:-"Unknown task"}
USER_ID=${USER:-"Agent"}
TIMESTAMP=$(date)

echo "LOCKED_BY=\"$USER_ID\"" > "$LOCK_FILE"
echo "REASON=\"$REASON\"" >> "$LOCK_FILE"
echo "TIMESTAMP=\"$TIMESTAMP\"" >> "$LOCK_FILE"

echo "🔒 Workspace locked by $USER_ID for: $REASON"
