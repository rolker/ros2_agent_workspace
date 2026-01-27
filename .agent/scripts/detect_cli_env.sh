#!/bin/bash
# .agent/scripts/detect_cli_env.sh
# Detects which AI CLI framework is currently running
#
# Exports:
#   AGENT_FRAMEWORK - Framework identifier (copilot-cli, gemini-cli, antigravity, unknown)
#   AGENT_FRAMEWORK_VERSION - Version if detectable
#
# Usage:
#   source .agent/scripts/detect_cli_env.sh
#   echo $AGENT_FRAMEWORK

# Default to unknown
export AGENT_FRAMEWORK="unknown"
export AGENT_FRAMEWORK_VERSION=""

# Detection logic (order matters - more specific checks first)

# GitHub Copilot CLI detection
if command -v gh &> /dev/null; then
    if gh copilot --version &> /dev/null 2>&1; then
        export AGENT_FRAMEWORK="copilot-cli"
        # Try to get version
        VERSION=$(gh copilot --version 2>/dev/null | head -n 1)
        if [ -n "$VERSION" ]; then
            export AGENT_FRAMEWORK_VERSION="$VERSION"
        fi
        return 0
    fi
fi

# Gemini CLI detection
# Check for Gemini-specific environment variables
if [ -n "$GEMINI_API_KEY" ] || [ -n "$GOOGLE_API_KEY" ]; then
    export AGENT_FRAMEWORK="gemini-cli"
    # Try to detect version from common locations
    if command -v gemini &> /dev/null; then
        VERSION=$(gemini --version 2>/dev/null | head -n 1)
        if [ -n "$VERSION" ]; then
            export AGENT_FRAMEWORK_VERSION="$VERSION"
        fi
    fi
    return 0
fi

# Check USER environment variable for Gemini patterns
if echo "$USER" | grep -qi "gemini"; then
    export AGENT_FRAMEWORK="gemini-cli"
    return 0
fi

# Antigravity detection
# Typically runs in containers with specific USER or environment
if [ "$USER" == "antigravity" ] || [ -n "$ANTIGRAVITY_SESSION" ]; then
    export AGENT_FRAMEWORK="antigravity"
    return 0
fi

# Claude CLI detection (if applicable in the future)
if [ -n "$CLAUDE_API_KEY" ] || command -v claude &> /dev/null; then
    export AGENT_FRAMEWORK="claude-cli"
    return 0
fi

# Check for generic AI agent indicators
if [ -n "$AI_AGENT_NAME" ]; then
    export AGENT_FRAMEWORK="${AI_AGENT_NAME,,}"  # Convert to lowercase
    return 0
fi

# If we reach here, framework is unknown
return 1
