#!/bin/bash
# .agent/scripts/set_git_identity_env.sh
# Sets ephemeral git identity using environment variables (session-only, doesn't modify .git/config)
#
# USAGE FOR HOST-BASED AGENTS (Copilot CLI, Gemini CLI):
#   source .agent/scripts/set_git_identity_env.sh "<Agent Name>" "<email>"
#
# Example:
#   source .agent/scripts/set_git_identity_env.sh "Copilot CLI Agent" "roland+copilot-cli@ccom.unh.edu"
#
# This sets identity ONLY for the current shell session without modifying .git/config.
# The user's .git/config remains unchanged, allowing them to commit as themselves afterward.
#
# IMPORTANT: Must be sourced (not executed) to export variables to current shell:
#   source .agent/scripts/set_git_identity_env.sh "..." "..."   ✓ Correct
#   ./agent/scripts/set_git_identity_env.sh "..." "..."         ✗ Wrong (variables won't persist)

# Check if script is being sourced (not executed)
if [ "${BASH_SOURCE[0]}" = "${0}" ]; then
    echo "❌ ERROR: This script must be sourced, not executed directly."
    echo ""
    echo "Correct usage:"
    echo "  source $0 \"<Agent Name>\" \"<email>\""
    echo ""
    echo "Example:"
    echo "  source $0 \"Copilot CLI Agent\" \"roland+copilot-cli@ccom.unh.edu\""
    echo "  source $0 \"Gemini CLI Agent\" \"roland+gemini-cli@ccom.unh.edu\""
    exit 1
fi

if [ $# -ne 2 ]; then
    echo "Usage: source $0 \"<Agent Name>\" \"<email>\""
    echo ""
    echo "Example:"
    echo "  source $0 \"Copilot CLI Agent\" \"roland+copilot-cli@ccom.unh.edu\""
    echo "  source $0 \"Gemini CLI Agent\" \"roland+gemini-cli@ccom.unh.edu\""
    echo ""
    echo "This sets git identity ONLY for the current shell session using environment variables."
    echo "It does NOT modify .git/config, so the user's identity remains unchanged."
    return 1
fi

AGENT_NAME="$1"
AGENT_EMAIL="$2"

echo "Setting ephemeral git identity for: $AGENT_NAME <$AGENT_EMAIL>"
echo ""

# Export Git environment variables for this session
# These take precedence over .git/config and global config
export GIT_AUTHOR_NAME="$AGENT_NAME"
export GIT_AUTHOR_EMAIL="$AGENT_EMAIL"
export GIT_COMMITTER_NAME="$AGENT_NAME"
export GIT_COMMITTER_EMAIL="$AGENT_EMAIL"

echo "✅ Ephemeral git identity configured for this session!"
echo ""
echo "Environment variables set:"
echo "  GIT_AUTHOR_NAME=$GIT_AUTHOR_NAME"
echo "  GIT_AUTHOR_EMAIL=$GIT_AUTHOR_EMAIL"
echo "  GIT_COMMITTER_NAME=$GIT_COMMITTER_NAME"
echo "  GIT_COMMITTER_EMAIL=$GIT_COMMITTER_EMAIL"
echo ""
echo "Verification:"
echo "  Current session identity: $(git var GIT_AUTHOR_IDENT | cut -d'<' -f1 | xargs)<$(git var GIT_AUTHOR_IDENT | grep -oP '<\K[^>]+')"
echo ""
echo "ℹ️  This identity applies ONLY to this shell session."
echo "ℹ️  The .git/config file remains unchanged."
echo "ℹ️  When the session ends, the user's original identity is restored."
