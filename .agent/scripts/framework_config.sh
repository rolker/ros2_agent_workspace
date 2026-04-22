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
# Maps framework key to fallback model name.
#
# IMPORTANT: These are FALLBACKS ONLY — used when an agent sources this script
# with only 2 args (name, email) and framework auto-detection can't determine
# a model. Agents MUST pass their actual runtime model as the 3rd argument to
# `set_git_identity_env.sh` (from their system prompt).
#
# DO NOT bump these values for every new model release. Doing so re-creates
# the staleness treadmill that issue #407 eliminated. If an agent intended to
# self-report its runtime model and its AI signature shows one of these values,
# it forgot to pass the 3rd arg — fix the caller, not this file. (Callers using
# --agent or --detect intentionally read from this table by design.)
declare -A FRAMEWORK_MODELS=(
    ["copilot"]="GPT-4o"
    ["gemini"]="Gemini 2.0 Flash"
    ["antigravity"]="Gemini 2.5 Pro"
    ["claude"]="Claude Opus 4.6"
    ["claude-code"]="Claude Opus 4.6"
)
