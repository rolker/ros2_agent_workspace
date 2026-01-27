#!/usr/bin/env bash
# Framework Identity Configuration
# Shared lookup tables for framework names and email addresses
# Sourced by: set_git_identity_env.sh, configure_git_identity.sh

# Framework name lookup table
# Maps framework key to display name
declare -A FRAMEWORK_NAMES=(
    ["copilot"]="Copilot CLI Agent"
    ["gemini"]="Gemini CLI Agent"
    ["antigravity"]="Antigravity Agent"
    ["claude"]="Claude CLI Agent"
)

# Framework email lookup table
# Maps framework key to email address
declare -A FRAMEWORK_EMAILS=(
    ["copilot"]="roland+copilot-cli@ccom.unh.edu"
    ["gemini"]="roland+gemini-cli@ccom.unh.edu"
    ["antigravity"]="roland+antigravity@ccom.unh.edu"
    ["claude"]="roland+claude-cli@ccom.unh.edu"
)
