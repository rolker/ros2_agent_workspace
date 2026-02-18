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
# Only session-specific env vars are reliable — they are set by the Copilot
# runtime when it spawns shells.  Having `gh copilot` installed is not proof
# we are running inside it.
if [ -n "$COPILOT_API_URL" ] || [ -n "$COPILOT_AGENT_CALLBACK_URL" ]; then
    export AGENT_FRAMEWORK="copilot-cli"
    if [ -n "$COPILOT_AGENT_RUNTIME_VERSION" ]; then
        export AGENT_FRAMEWORK_VERSION="$COPILOT_AGENT_RUNTIME_VERSION"
    fi
    return 0
fi

# Gemini CLI detection
# GEMINI_API_KEY / GOOGLE_API_KEY are API credentials that users commonly set
# in their profile — they do NOT indicate an active Gemini CLI session.
# Rely on session-specific indicators only.
if [ -n "$GEMINI_SESSION" ]; then
    export AGENT_FRAMEWORK="gemini-cli"
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
# Only $CLAUDE_CODE is a reliable signal — it is set by Claude Code when it
# spawns subshells.  $ANTHROPIC_API_KEY / $CLAUDE_API_KEY are API credentials
# that users commonly have in their profile and do NOT indicate an active
# Claude Code session.  Likewise, having the `claude` CLI installed does not
# mean we are running inside it.
if [ -n "$CLAUDECODE" ] || [ -n "$CLAUDE_CODE" ] || [ -n "$CLAUDE_CODE_ENTRYPOINT" ]; then
    export AGENT_FRAMEWORK="claude-code"
    if command -v claude &> /dev/null; then
        VERSION=$(claude --version 2>/dev/null | head -n 1)
        if [ -n "$VERSION" ]; then
            export AGENT_FRAMEWORK_VERSION="$VERSION"
        fi
    fi
    return 0
fi

# Check for generic AI agent indicators
if [ -n "$AI_AGENT_NAME" ]; then
    export AGENT_FRAMEWORK="${AI_AGENT_NAME,,}"  # Convert to lowercase
    return 0
fi

# If we reach here, framework is unknown
return 1
