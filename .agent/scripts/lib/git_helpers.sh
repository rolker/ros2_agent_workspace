#!/bin/bash
# Git helper functions for CLI agents
# Prevents agents from getting stuck in interactive editors

# Safe git rebase - skips interactive editor
safe_git_rebase() {
    GIT_EDITOR=true git rebase "$@"
}

# Safe git commit amend - keeps existing message
safe_git_amend() {
    git commit --amend --no-edit "$@"
}

# Safe git merge - skips editor for merge commit message
safe_git_merge() {
    GIT_EDITOR=true git merge "$@"
}

# Safe git rebase continue - skips editor after conflict resolution
safe_git_rebase_continue() {
    GIT_EDITOR=true git rebase --continue
}

# Safe git cherry-pick - skips editor
safe_git_cherry_pick() {
    GIT_EDITOR=true git cherry-pick "$@"
}

# Export functions so they're available in subshells
export -f safe_git_rebase
export -f safe_git_amend
export -f safe_git_merge
export -f safe_git_rebase_continue
export -f safe_git_cherry_pick
