#!/usr/bin/env bash
# Framework Identity Configuration
# Shared lookup tables for framework names, email addresses, and default models
# Sourced by: set_git_identity_env.sh, configure_git_identity.sh, detect_agent_identity.sh

# shellcheck disable=SC2034

# Framework name lookup table
# Maps framework key to display name
declare -A FRAMEWORK_NAMES=(
    ["copilot"]="Copilot CLI Agent"
    ["gemini"]="Gemini CLI Agent"
    ["antigravity"]="Antigravity Agent"
    ["claude"]="Claude Code Agent"
    ["claude-code"]="Claude Code Agent"
)

# Framework email lookup table
# Maps framework key to email address
declare -A FRAMEWORK_EMAILS=(
    ["copilot"]="roland+copilot-cli@ccom.unh.edu"
    ["gemini"]="roland+gemini-cli@ccom.unh.edu"
    ["antigravity"]="roland+antigravity@ccom.unh.edu"
    ["claude"]="roland+claude-code@ccom.unh.edu"
    ["claude-code"]="roland+claude-code@ccom.unh.edu"
)

# Framework default model lookup table
# Maps framework key to typical/default model name
# NOTE: These are defaults - actual runtime model may differ
# Agents should detect their actual model when possible
declare -A FRAMEWORK_MODELS=(
    ["copilot"]="GPT-4o"
    ["gemini"]="Gemini 2.0 Flash"
    ["antigravity"]="Gemini 2.5 Pro"
    ["claude"]="Claude Opus 4.5"
    ["claude-code"]="Claude Opus 4.5"
)
