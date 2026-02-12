#!/bin/bash
# scripts/generate_knowledge.sh
#
# Previously this script created symlinks from .agent/knowledge/ to
# project-specific documentation inside cloned layer repos. That pattern
# was abandoned in favor of fully version-controlled knowledge files
# (see commit 89bcd1f).
#
# Project-specific agent context is now accessed via the
# .agent/project_knowledge symlink, which setup.sh creates pointing to
# the manifest repo's agent_context/ directory (if it exists).
#
# This script is retained as a no-op for backward compatibility with
# any automation that calls it.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

echo "Workspace knowledge is version-controlled in .agent/knowledge/."

if [ -L "$ROOT_DIR/.agent/project_knowledge" ]; then
    echo "Project-specific knowledge available at .agent/project_knowledge/"
    echo "  -> $(readlink "$ROOT_DIR/.agent/project_knowledge")"
elif [ -d "$ROOT_DIR/.agent/project_knowledge" ]; then
    echo "Project-specific knowledge available at .agent/project_knowledge/"
else
    echo "No project-specific knowledge found (.agent/project_knowledge does not exist)."
    echo "The manifest repo can provide project knowledge by creating an agent_context/"
    echo "directory inside its config path."
fi

echo "Done."
