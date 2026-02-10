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
# Check for environment variables first (more reliable in CI/container environments)
if [ -n "$COPILOT_API_URL" ] || [ -n "$COPILOT_AGENT_CALLBACK_URL" ]; then
    export AGENT_FRAMEWORK="copilot-cli"
    # Try to get version from runtime env
    if [ -n "$COPILOT_AGENT_RUNTIME_VERSION" ]; then
        export AGENT_FRAMEWORK_VERSION="$COPILOT_AGENT_RUNTIME_VERSION"
    fi
    return 0
fi

# Fallback: Check for gh copilot command
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

# Claude Code detection
# Check for Claude Code specific environment variables or command
if [ -n "$CLAUDE_CODE" ] || [ -n "$ANTHROPIC_API_KEY" ] || [ -n "$CLAUDE_API_KEY" ]; then
    export AGENT_FRAMEWORK="claude-code"
    # Try to detect version from claude command
    if command -v claude &> /dev/null; then
        VERSION=$(claude --version 2>/dev/null | head -n 1)
        if [ -n "$VERSION" ]; then
            export AGENT_FRAMEWORK_VERSION="$VERSION"
        fi
    fi
    return 0
fi

# Fallback: Check for claude command with version validation
if command -v claude &> /dev/null; then
    CLAUDE_VERSION_OUTPUT=$(claude --version 2>/dev/null)
    if [ $? -eq 0 ]; then
        CLAUDE_VERSION_LINE=$(echo "$CLAUDE_VERSION_OUTPUT" | head -n 1)
        if echo "$CLAUDE_VERSION_LINE" | grep -qi "claude"; then
            export AGENT_FRAMEWORK="claude-code"
            if [ -n "$CLAUDE_VERSION_LINE" ]; then
                export AGENT_FRAMEWORK_VERSION="$CLAUDE_VERSION_LINE"
            fi
            return 0
        fi
    fi
fi

# Check for generic AI agent indicators
if [ -n "$AI_AGENT_NAME" ]; then
    export AGENT_FRAMEWORK="${AI_AGENT_NAME,,}"  # Convert to lowercase
    return 0
fi

# If we reach here, framework is unknown
return 1
