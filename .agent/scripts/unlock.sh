#!/bin/bash
# scripts/unlock.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"
LOCK_FILE="$ROOT_DIR/.agent/scratchpad/workspace.lock"

if [ ! -f "$LOCK_FILE" ]; then
    echo "⚠️ Workspace is NOT locked."
    exit 0
fi

# Ideally we should check if the user unlocking is the one who locked it,
# but for now we assume trust between agents/users.

rm "$LOCK_FILE"
echo "🔓 Workspace unlocked."
