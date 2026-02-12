#!/bin/bash
# .agent/scripts/configure_git_identity.sh
# Configures git identity for AI agents across workspace and all cloned repositories
#
# Usage:
#   ./configure_git_identity.sh "<Agent Name>" "<email>"
#   ./configure_git_identity.sh --agent <framework>
#   ./configure_git_identity.sh --detect
#
# Examples:
#   ./configure_git_identity.sh "Copilot CLI Agent" "roland+copilot-cli@ccom.unh.edu"
#   ./configure_git_identity.sh --agent copilot
#   ./configure_git_identity.sh --detect

# Load framework identity lookup table from shared configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FRAMEWORK_CONFIG="${SCRIPT_DIR}/framework_config.sh"

if [ -f "${FRAMEWORK_CONFIG}" ]; then
    # shellcheck source=/dev/null
    source "${FRAMEWORK_CONFIG}"
else
    echo "❌ ERROR: Framework configuration not found at ${FRAMEWORK_CONFIG}."
    echo "This script expects FRAMEWORK_NAMES and FRAMEWORK_EMAILS to be defined in a shared config."
    exit 1
fi

# Add aliases for backward compatibility
FRAMEWORK_NAMES["copilot-cli"]="${FRAMEWORK_NAMES[copilot]}"
FRAMEWORK_EMAILS["copilot-cli"]="${FRAMEWORK_EMAILS[copilot]}"
FRAMEWORK_NAMES["gemini-cli"]="${FRAMEWORK_NAMES[gemini]}"
FRAMEWORK_EMAILS["gemini-cli"]="${FRAMEWORK_EMAILS[gemini]}"

show_usage() {
    echo "Usage: $0 [OPTIONS] [\"<Agent Name>\" \"<email>\"]"
    echo ""
    echo "Options:"
    echo "  --agent <framework>    Use predefined identity for framework"
    echo "                         Supported: copilot, gemini, antigravity, claude"
    echo "  --detect              Auto-detect framework from environment"
    echo ""
    echo "Examples:"
    echo "  $0 \"Copilot CLI Agent\" \"roland+copilot-cli@ccom.unh.edu\""
    echo "  $0 --agent copilot"
    echo "  $0 --agent gemini"
    echo "  $0 --detect"
    echo ""
    echo "This configures git identity in:"
    echo "  - The workspace repository (ros2_agent_workspace)"
    echo "  - All repositories under layers/*/src/*"
}

# Auto-detect framework from environment
detect_framework() {
    # Source the detection script if available
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
    exit 1
fi

if [ "$1" == "--help" ] || [ "$1" == "-h" ]; then
    show_usage
    exit 0
fi

if [ "$1" == "--agent" ]; then
    if [ $# -ne 2 ]; then
        echo "Error: --agent requires framework name"
        show_usage
        exit 1
    fi

    FRAMEWORK="${2,,}"  # Convert to lowercase
    AGENT_NAME="${FRAMEWORK_NAMES[$FRAMEWORK]}"
    AGENT_EMAIL="${FRAMEWORK_EMAILS[$FRAMEWORK]}"

    if [ -z "$AGENT_NAME" ]; then
        echo "Error: Unknown framework '$2'"
        echo "Supported frameworks: ${!FRAMEWORK_NAMES[@]}"
        exit 1
    fi

elif [ "$1" == "--detect" ]; then
    DETECTED=$(detect_framework)

    if [ "$DETECTED" == "unknown" ]; then
        echo "Error: Could not auto-detect framework"
        echo "Please use --agent <framework> or provide name/email manually"
        exit 1
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
    exit 1
fi

echo "Configuring git identity for: $AGENT_NAME <$AGENT_EMAIL>"
echo ""

# Configure workspace repository
echo "Configuring workspace repository..."
git config user.name "$AGENT_NAME"
git config user.email "$AGENT_EMAIL"
echo "  ✓ Workspace repository configured"

# Configure all repositories in layers/

if [ -d "layers" ]; then
    REPO_COUNT=0
    # Use process substitution with find to be robust against weird filenames
    # and avoid subshell exit issues with set -e
    while IFS= read -r git_dir; do
        repo_dir="$(dirname "$git_dir")"
        echo "Configuring $repo_dir..."

        # Capture failure but don't crash whole script immediately if one fails
        if ! git -C "$repo_dir" config user.name "$AGENT_NAME" || \
           ! git -C "$repo_dir" config user.email "$AGENT_EMAIL"; then
            echo "  ✗ Failed to configure $repo_dir" >&2
            # Decide if we want to fail hard or continue.
            # Continuing sees more errors.
            continue
        fi
        ((REPO_COUNT++))
    done < <(find layers -type d -name ".git")

    if [ $REPO_COUNT -gt 0 ]; then
        echo "  ✓ Configured $REPO_COUNT repositories in layers/"
    else
        echo "  ℹ No repositories found in layers/ yet"
    fi
else
    echo "  ℹ No layers/ directory found yet"
fi

echo ""
echo "✅ Git identity configuration complete!"
echo ""
echo "Verification:"
echo "  Workspace: $(git config user.name) <$(git config user.email)>"
