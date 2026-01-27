#!/bin/bash
# .agent/scripts/set_git_identity_env.sh
# Sets ephemeral git identity using environment variables (session-only, doesn't modify .git/config)
#
# USAGE FOR HOST-BASED AGENTS (Copilot CLI, Gemini CLI):
#   source .agent/scripts/set_git_identity_env.sh "<Agent Name>" "<email>"
#   source .agent/scripts/set_git_identity_env.sh --agent <framework>
#   source .agent/scripts/set_git_identity_env.sh --detect
#
# Examples:
#   source .agent/scripts/set_git_identity_env.sh "Copilot CLI Agent" "roland+copilot-cli@ccom.unh.edu"
#   source .agent/scripts/set_git_identity_env.sh --agent copilot
#   source .agent/scripts/set_git_identity_env.sh --detect
#
# This sets identity ONLY for the current shell session without modifying .git/config.
# The user's .git/config remains unchanged, allowing them to commit as themselves afterward.
#
# IMPORTANT: Must be sourced (not executed) to export variables to current shell:
#   source .agent/scripts/set_git_identity_env.sh "..." "..."   ✓ Correct
#   ./.agent/scripts/set_git_identity_env.sh "..." "..."        ✗ Wrong (variables won't persist)

# Load framework identity lookup table from shared configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FRAMEWORK_CONFIG="${SCRIPT_DIR}/framework_config.sh"

if [ -f "${FRAMEWORK_CONFIG}" ]; then
    # shellcheck source=/dev/null
    source "${FRAMEWORK_CONFIG}"
else
    echo "❌ ERROR: Framework configuration not found at ${FRAMEWORK_CONFIG}."
    echo "This script expects FRAMEWORK_NAMES and FRAMEWORK_EMAILS to be defined in a shared config."
    return 1
fi

# Add aliases for backward compatibility
FRAMEWORK_NAMES["copilot-cli"]="${FRAMEWORK_NAMES[copilot]}"
FRAMEWORK_EMAILS["copilot-cli"]="${FRAMEWORK_EMAILS[copilot]}"
FRAMEWORK_NAMES["gemini-cli"]="${FRAMEWORK_NAMES[gemini]}"
FRAMEWORK_EMAILS["gemini-cli"]="${FRAMEWORK_EMAILS[gemini]}"

# Check if script is being sourced (not executed)
if [ "${BASH_SOURCE[0]}" = "${0}" ]; then
    echo "❌ ERROR: This script must be sourced, not executed directly."
    echo ""
    echo "Correct usage:"
    echo "  source $0 \"<Agent Name>\" \"<email>\""
    echo "  source $0 --agent <framework>"
    echo "  source $0 --detect"
    echo ""
    echo "Examples:"
    echo "  source $0 \"Copilot CLI Agent\" \"roland+copilot-cli@ccom.unh.edu\""
    echo "  source $0 --agent copilot"
    echo "  source $0 --detect"
    exit 1
fi

show_usage() {
    echo "Usage: source $0 [OPTIONS] [\"<Agent Name>\" \"<email>\"]"
    echo ""
    echo "Options:"
    echo "  --agent <framework>    Use predefined identity for framework"
    echo "                         Supported: copilot, gemini, antigravity, claude"
    echo "  --detect              Auto-detect framework from environment"
    echo ""
    echo "Examples:"
    echo "  source $0 \"Copilot CLI Agent\" \"roland+copilot-cli@ccom.unh.edu\""
    echo "  source $0 --agent copilot"
    echo "  source $0 --detect"
    echo ""
    echo "This sets git identity ONLY for the current shell session using environment variables."
    echo "It does NOT modify .git/config, so the user's identity remains unchanged."
}

# Auto-detect framework
detect_framework() {
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    if [ -f "$SCRIPT_DIR/detect_cli_env.sh" ]; then
        source "$SCRIPT_DIR/detect_cli_env.sh"
        echo "$AGENT_FRAMEWORK"
    else
        echo "unknown"
    fi
}

# Parse arguments
if [ $# -eq 0 ]; then
    show_usage
    return 1
fi

if [ "$1" == "--help" ] || [ "$1" == "-h" ]; then
    show_usage
    return 0
fi

if [ "$1" == "--agent" ]; then
    if [ $# -ne 2 ]; then
        echo "Error: --agent requires framework name"
        show_usage
        return 1
    fi
    
    FRAMEWORK="${2,,}"  # Convert to lowercase
    AGENT_NAME="${FRAMEWORK_NAMES[$FRAMEWORK]}"
    AGENT_EMAIL="${FRAMEWORK_EMAILS[$FRAMEWORK]}"
    
    if [ -z "$AGENT_NAME" ]; then
        echo "Error: Unknown framework '$2'"
        echo "Supported frameworks: ${!FRAMEWORK_NAMES[@]}"
        return 1
    fi
    
elif [ "$1" == "--detect" ]; then
    DETECTED=$(detect_framework)
    
    if [ "$DETECTED" == "unknown" ]; then
        echo "Error: Could not auto-detect framework"
        echo "Please use --agent <framework> or provide name/email manually"
        return 1
    fi
    
    AGENT_NAME="${FRAMEWORK_NAMES[$DETECTED]}"
    AGENT_EMAIL="${FRAMEWORK_EMAILS[$DETECTED]}"
    echo "Detected framework: $DETECTED"
    
elif [ $# -eq 2 ]; then
    # Manual name and email
    AGENT_NAME="$1"
    AGENT_EMAIL="$2"
else
    echo "Error: Invalid arguments"
    show_usage
    return 1
fi

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
AUTHOR_IDENT=$(git var GIT_AUTHOR_IDENT)
AUTHOR_NAME=$(echo "$AUTHOR_IDENT" | sed 's/ <.*//')
AUTHOR_EMAIL=$(echo "$AUTHOR_IDENT" | sed 's/.*<\(.*\)>.*/\1/')
echo "  Current session identity: $AUTHOR_NAME <$AUTHOR_EMAIL>"
echo ""
echo "ℹ️  This identity applies ONLY to this shell session."
echo "ℹ️  The .git/config file remains unchanged."
echo "ℹ️  When the session ends, the user's original identity is restored."
