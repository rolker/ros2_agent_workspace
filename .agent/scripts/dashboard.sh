#!/bin/bash

# ROS2 Agent Workspace Dashboard
# Unified workspace status: health checks, repository sync, git status,
# worktree status, GitHub PRs/issues, and test results.
#
# Replaces status_report.sh and health_check.sh with a single view.
#
# Usage:
#   dashboard.sh [OPTIONS]
#
# OPTIONS:
#   --quick          Quick local-only mode (skip sync and GitHub API calls)
#   --skip-sync      Skip repository fetch step (faster, may show stale data)
#   --skip-github    Skip GitHub PR/issue queries (offline mode)
#   --help           Show this help message
#
# EXAMPLES:
#   dashboard.sh                    # Full dashboard with sync and GitHub
#   dashboard.sh --quick            # Fast local-only check
#   dashboard.sh --skip-sync        # Skip fetch, keep GitHub queries
#
# DEPENDENCIES:
#   Required: git, python3
#   Optional: vcs (for layer sync), gh + jq (for GitHub data)

# Note: do not use 'set -e' here; many checks are best-effort and may fail.

# Suppress Python deprecation warnings from vcstool
export PYTHONWARNINGS="ignore::DeprecationWarning"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

# --- Parse arguments ---
SKIP_SYNC=false
SKIP_GITHUB=false
while [[ $# -gt 0 ]]; do
    case $1 in
        --quick)
            SKIP_SYNC=true
            SKIP_GITHUB=true
            shift
            ;;
        --skip-sync)
            SKIP_SYNC=true
            shift
            ;;
        --skip-github)
            SKIP_GITHUB=true
            shift
            ;;
        --help)
            awk 'NR>1 && /^# Note: do not use/ {exit} NR>1 {sub(/^# ?/, ""); print}' "$0"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# --- Worktree detection ---
WORKTREE_INFO=""
if [[ "$ROOT_DIR" == *"/layers/worktrees/"* ]]; then
    WORKTREE_INFO="layer worktree"
    MAIN_ROOT="$(dirname "$(dirname "$(dirname "$ROOT_DIR")")")"
    LAYERS_DIR="$MAIN_ROOT/layers/main"
elif [[ "$ROOT_DIR" == *"/.workspace-worktrees/"* ]]; then
    WORKTREE_INFO="workspace worktree"
    MAIN_ROOT="$(dirname "$(dirname "$ROOT_DIR")")"
    LAYERS_DIR="$MAIN_ROOT/layers/main"
else
    MAIN_ROOT="$ROOT_DIR"
    LAYERS_DIR="$ROOT_DIR/layers/main"
fi

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

check_pass() { echo -e "  ${GREEN}✅ $1${NC}"; }
check_fail() { echo -e "  ${RED}❌ $1${NC}"; }
check_warn() { echo -e "  ${YELLOW}⚠️  $1${NC}"; }

# --- Header ---
echo ""
echo "# Workspace Dashboard"
echo "**Date**: $(date)"
if [ -n "$WORKTREE_INFO" ]; then
    echo "**Context**: Running in $WORKTREE_INFO"
fi
echo ""

SECTION=0

#######################################
# SECTION: HEALTH CHECKS
#######################################

SECTION=$((SECTION + 1))
echo "## $SECTION. Health Checks"
echo ""

FAILED_CHECKS=0

# ROS 2
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    check_pass "ROS 2 Jazzy installed"
else
    check_fail "ROS 2 Jazzy not found. Run: make build (auto-bootstraps)"
    ((FAILED_CHECKS++))
fi

# Required tools
for tool in vcs colcon rosdep python3; do
    if command -v "$tool" &> /dev/null; then
        check_pass "$tool found"
    else
        check_fail "$tool not found"
        ((FAILED_CHECKS++))
    fi
done

# Dev tools
VENV_ROOT="${MAIN_ROOT:-$ROOT_DIR}"
if [ -x "$VENV_ROOT/.venv/bin/pre-commit" ]; then
    check_pass "pre-commit installed in .venv"

    HOOK_FILE=$(git -C "$ROOT_DIR" rev-parse --path-format=absolute --git-path hooks/pre-commit 2>/dev/null || true)
    if [ -n "$HOOK_FILE" ] && [ -f "$HOOK_FILE" ]; then
        INSTALL_PYTHON=$(sed -n 's/^INSTALL_PYTHON=//p' "$HOOK_FILE" | tr -d "'" | tr -d '"')
        if [ -n "$INSTALL_PYTHON" ] && [ -x "$INSTALL_PYTHON" ]; then
            check_pass "pre-commit hook installed"
        else
            check_warn "pre-commit hook has invalid INSTALL_PYTHON. Run: make lint"
        fi
    elif [ -n "$HOOK_FILE" ]; then
        check_warn "pre-commit hook not installed. Run: make lint (auto-installs)"
    fi
else
    check_warn "pre-commit not found. Run: make lint (auto-installs)"
fi

# git-bug (optional)
if command -v git-bug &>/dev/null; then
    check_pass "git-bug found ($(git-bug version 2>/dev/null | head -1))"
else
    check_warn "git-bug not installed (optional: offline issue tracking)"
fi

# Workspace structure
if [ -d "$ROOT_DIR/configs" ]; then
    check_pass "configs/ directory exists"
else
    check_fail "configs/ directory not found"
    ((FAILED_CHECKS++))
fi

# Configuration validation
if [ -f "$SCRIPT_DIR/validate_workspace.py" ]; then
    if python3 "$SCRIPT_DIR/validate_workspace.py" &>/dev/null; then
        check_pass "Workspace matches .repos configuration"
    else
        check_warn "Workspace drift detected. Run: make validate"
    fi
fi

# Lock status
LOCK_FILE="$ROOT_DIR/.agent/scratchpad/workspace.lock"
if [ -f "$LOCK_FILE" ]; then
    check_warn "Workspace is LOCKED (run: make unlock)"
else
    check_pass "Workspace is unlocked"
fi

# Git status
cd "$ROOT_DIR" || exit
if git rev-parse --git-dir > /dev/null 2>&1; then
    BRANCH=$(git branch --show-current)
    if [ -n "$(git status --porcelain)" ]; then
        check_warn "Root repo has uncommitted changes (branch: $BRANCH)"
    else
        check_pass "Root repo is clean (branch: $BRANCH)"
    fi
fi

# Layers
if [ -d "$LAYERS_DIR" ]; then
    LAYER_COUNT=$(ls -d "$LAYERS_DIR"/*_ws 2>/dev/null | wc -l)
    BUILT_COUNT=0
    for layer_dir in "$LAYERS_DIR"/*_ws; do
        [ -f "$layer_dir/install/setup.bash" ] && ((BUILT_COUNT++))
    done
    check_pass "$LAYER_COUNT layer(s), $BUILT_COUNT built"
else
    check_warn "layers/main/ not found (run: make build)"
fi

echo ""
if [ $FAILED_CHECKS -eq 0 ]; then
    echo -e "  ${GREEN}All critical checks passed.${NC}"
else
    echo -e "  ${RED}$FAILED_CHECKS critical check(s) failed.${NC}"
fi
echo ""

#######################################
# SECTION: SYNC REPOSITORIES
#######################################

if [ "$SKIP_SYNC" = false ]; then
    SECTION=$((SECTION + 1))
    echo "## $SECTION. Syncing Repositories"
    echo ""

    (
        cd "$ROOT_DIR" || exit 0
        echo -n "Syncing root repository... "
        if git fetch --quiet 2>/dev/null; then
            echo "✅"
        else
            echo "⚠️ (fetch failed)"
        fi
    )

    if command -v vcs &> /dev/null; then
        for ws_dir in "$LAYERS_DIR"/*; do
            if [ -d "$ws_dir/src" ]; then
                ws_name=$(basename "$ws_dir" | sed 's/_ws//')
                echo -n "Syncing $ws_name workspace... "
                (
                    cd "$ws_dir/src" || exit 0
                    if vcs custom --git --args fetch --quiet >/dev/null 2>&1; then
                        echo "✅"
                    else
                        echo "⚠️ (some repos may have failed)"
                    fi
                )
            fi
        done
    else
        echo "⚠️ vcs command not found - skipping layer sync"
    fi

    echo ""
fi

#######################################
# SECTION: REPOSITORY STATUS
#######################################

SECTION=$((SECTION + 1))
echo "## $SECTION. Repository Status"
echo ""

# Root repository
echo "### Root Repository"
cd "$ROOT_DIR" || exit
if command -v git &> /dev/null; then
    if [ -n "$(git status --porcelain)" ]; then
        echo "- **Status**: ⚠️ Modified"
        echo "- **Branch**: $(git branch --show-current)"
        echo ""
        echo "**Modified Files:**"
        echo '```'
        git status --short
        echo '```'
    else
        echo "- **Status**: ✅ Clean"
        echo "- **Branch**: $(git branch --show-current)"
    fi
fi
echo ""

# Layer repositories
if command -v vcs &> /dev/null; then
    EXPECTED_REPOS=$(python3 "$SCRIPT_DIR/list_overlay_repos.py" --include-underlay --format names 2>/dev/null)
    if [ $? -ne 0 ] || [ -z "$EXPECTED_REPOS" ]; then
        EXPECTED_REPOS=""
    fi

    for ws_dir in "$LAYERS_DIR"/*; do
        if [ -d "$ws_dir/src" ]; then
            ws_name=$(basename "$ws_dir" | sed 's/_ws//')

            if ! cd "$ws_dir/src"; then
                echo "Warning: could not enter '$ws_dir/src'; skipping."
                continue
            fi

            raw_output=$(vcs custom --git --args status --porcelain -b 2>/dev/null || echo "")

            clean_count=0
            modified_count=0
            modified_repos=()
            current_repo=""
            is_dirty=false
            sync_status=""
            branch=""

            process_repo() {
                if [ "$current_repo" != "" ]; then
                    local status_str=""
                    if [ "$is_dirty" = true ]; then
                        status_str="⚠️ Modified"
                    fi

                    if [ "$sync_status" != "" ]; then
                        status_str="${status_str:+$status_str, }$sync_status"
                    fi

                    if [ -f "$SCRIPT_DIR/get_repo_info.py" ]; then
                        expected_branch=$(python3 "$SCRIPT_DIR/get_repo_info.py" "$current_repo" 2>/dev/null)
                    else
                        expected_branch="unknown"
                    fi

                    if [ "$expected_branch" != "unknown" ] && [ -n "$expected_branch" ]; then
                        if [ "$branch" != "$expected_branch" ]; then
                            warning="$branch (Want: $expected_branch)"
                            status_str="${status_str:+$status_str, }$warning"
                        fi
                    else
                        status_str="${status_str:+$status_str, }Expected branch unknown"
                    fi

                    if [ -n "$EXPECTED_REPOS" ]; then
                        if ! echo "$EXPECTED_REPOS" | grep -qx "$current_repo"; then
                            status_str="${status_str:+$status_str, }Untracked"
                        fi
                    fi

                    if [ "$status_str" != "" ]; then
                        modified_repos+=("$current_repo|$status_str|$branch")
                        modified_count=$((modified_count + 1))
                    else
                        clean_count=$((clean_count + 1))
                    fi
                fi
            }

            while IFS= read -r line; do
                if [[ "$line" == "="* ]]; then
                    process_repo
                    current_repo=$(echo "$line" | sed 's/^=== \(.*\) (git) ===$/\1/')
                    current_repo=$(basename "$current_repo")
                    is_dirty=false
                    sync_status=""
                    branch=""
                elif [[ "$line" =~ ^##[[:space:]](.*)$ ]]; then
                    branch_line="${BASH_REMATCH[1]}"
                    if [[ "$branch_line" =~ \.\.\..*\[ahead[[:space:]]([0-9]+)\] ]]; then
                        sync_status="Ahead ${BASH_REMATCH[1]}"
                    elif [[ "$branch_line" =~ \.\.\..*\[behind[[:space:]]([0-9]+)\] ]]; then
                        sync_status="Behind ${BASH_REMATCH[1]}"
                    elif [[ "$branch_line" =~ \.\.\..*\[ahead[[:space:]]([0-9]+),[[:space:]]behind[[:space:]]([0-9]+)\] ]]; then
                        sync_status="Ahead ${BASH_REMATCH[1]}, Behind ${BASH_REMATCH[2]}"
                    fi
                    branch=$(echo "$branch_line" | sed 's/\.\.\..*//')
                elif [[ "$line" =~ ^[[:space:]]?[MADRCU?!] ]]; then
                    is_dirty=true
                fi
            done <<< "$raw_output"

            process_repo

            total=$((clean_count + modified_count))
            echo "### Workspace: $ws_name (Total: $total, Clean: $clean_count, Attention: $modified_count)"

            if [ $modified_count -gt 0 ]; then
                echo ""
                echo "| Repository | Status | Branch |"
                echo "|---|---|---|"
                for repo_entry in "${modified_repos[@]}"; do
                    IFS='|' read -r repo_name status_info branch_info <<< "$repo_entry"
                    printf "| %-30s | %-40s | %-20s |\n" "$repo_name" "$status_info" "$branch_info"
                done
            fi
            echo ""
        fi
    done
else
    echo "⚠️ vcs command not found — cannot show layer status"
    echo ""
fi

echo "---"
echo ""

#######################################
# SECTION: ACTIVE WORKTREES
#######################################

SECTION=$((SECTION + 1))
echo "## $SECTION. Active Worktrees"
echo ""

WORKTREE_SCRIPT="$SCRIPT_DIR/worktree_list.sh"
if [ -x "$WORKTREE_SCRIPT" ]; then
    # Count worktrees (exclude main)
    WT_COUNT=$(git -C "$ROOT_DIR" worktree list 2>/dev/null | grep -v "(bare)" | grep -vF "$ROOT_DIR " | wc -l)
    if [ "$WT_COUNT" -gt 0 ]; then
        # Show summary from worktree_list.sh output
        "$WORKTREE_SCRIPT" 2>/dev/null | grep -E "^(\[main\]|\[workspace\]|\[layer\]|  )" || true
        echo ""
    else
        echo "No active worktrees."
        echo ""
    fi
else
    echo "⚠️ worktree_list.sh not found"
    echo ""
fi

echo "---"
echo ""

#######################################
# SECTION: GITHUB PULL REQUESTS & ISSUES
#######################################

if [ "$SKIP_GITHUB" = false ]; then
    # Ensure required tools
    if ! command -v gh &> /dev/null; then
        SECTION=$((SECTION + 1))
        echo "## $SECTION. GitHub Status"
        echo ""
        echo "⚠️ **GitHub CLI (\`gh\`) not found**"
        echo "Install: \`sudo apt install gh\`, then: \`gh auth login\`"
        echo ""
    elif ! gh auth status &> /dev/null; then
        SECTION=$((SECTION + 1))
        echo "## $SECTION. GitHub Status"
        echo ""
        echo "⚠️ **GitHub CLI not authenticated**"
        echo "Run: \`gh auth login\`"
        echo ""
    elif ! command -v jq &> /dev/null; then
        SECTION=$((SECTION + 1))
        echo "## $SECTION. GitHub Status"
        echo ""
        echo "⚠️ **jq not found** (required for GitHub queries)"
        echo "Install: \`sudo apt install jq\`"
        echo ""
    else
        # Build repo list
        REPOS=$(python3 "$SCRIPT_DIR/list_overlay_repos.py" --include-underlay 2>/dev/null | jq -r '.[].url' 2>/dev/null | sed 's|https://github.com/||' | sed 's|.git$||' || true)
        ROOT_REPO=$(cd "$ROOT_DIR" && git remote get-url origin 2>/dev/null | sed 's|git@github.com:||' | sed 's|https://github.com/||' | sed 's|.git$||' || true)
        if [ -n "$ROOT_REPO" ]; then
            REPOS=$(echo -e "$ROOT_REPO\n$REPOS")
        fi
        REPOS=$(echo "$REPOS" | sort -u | grep -v '^$')

        # Pull Requests
        SECTION=$((SECTION + 1))
        echo "## $SECTION. GitHub Pull Requests"
        echo ""

        PR_COUNT=0
        PR_OUTPUT=""
        for repo in $REPOS; do
            [ -z "$repo" ] && continue
            prs=$(gh pr list --repo "$repo" --json number,title,url --limit 100 2>/dev/null || echo "[]")
            if [ -n "$prs" ] && [ "$prs" != "[]" ]; then
                while read -r pr_line; do
                    [ -z "$pr_line" ] && continue
                    number=$(echo "$pr_line" | jq -r '.number')
                    title=$(echo "$pr_line" | jq -r '.title')
                    url=$(echo "$pr_line" | jq -r '.url')
                    [ ${#title} -gt 60 ] && title="${title:0:57}..."
                    repo_name=$(basename "$repo")
                    PR_OUTPUT+="| $repo_name | [#$number]($url) | $title |"$'\n'
                    PR_COUNT=$((PR_COUNT + 1))
                done < <(echo "$prs" | jq -c '.[]' 2>/dev/null)
            fi
        done

        if [ $PR_COUNT -gt 0 ]; then
            echo "| Repository | PR | Title |"
            echo "|------------|-----|-------|"
            printf '%s\n' "$PR_OUTPUT"
            echo "**Total Open PRs**: $PR_COUNT"
        else
            echo "✅ No open pull requests"
        fi
        echo ""

        # Issues
        SECTION=$((SECTION + 1))
        echo "## $SECTION. GitHub Issues"
        echo ""

        ISSUE_COUNT=0
        ISSUE_OUTPUT=""
        for repo in $REPOS; do
            [ -z "$repo" ] && continue
            # Try git-bug for the workspace repo (where a bridge is configured)
            count=""
            if [ "$repo" = "$ROOT_REPO" ] && command -v git-bug &>/dev/null; then
                _GITBUG_HELPERS="$(dirname "${BASH_SOURCE[0]}")/gitbug_helpers.sh"
                if [ -f "$_GITBUG_HELPERS" ]; then
                    # shellcheck source=gitbug_helpers.sh
                    source "$_GITBUG_HELPERS"
                fi
                if declare -F gitbug_has_bridge &>/dev/null && gitbug_has_bridge "$ROOT_DIR"; then
                    count=$(gitbug_count_open "$ROOT_DIR" 2>/dev/null || echo "")
                fi
                if [ -z "$count" ] || ! [[ "$count" =~ ^[0-9]+$ ]]; then
                    echo "  ⚠️  git-bug count failed for $repo, falling back to gh API" >&2
                fi
            fi
            # Fall back to gh API
            if [ -z "$count" ] || ! [[ "$count" =~ ^[0-9]+$ ]]; then
                count=$(gh api -X GET search/issues -f q="repo:$repo is:issue is:open" --jq '.total_count' 2>/dev/null || echo "0")
            fi
            [[ "$count" =~ ^[0-9]+$ ]] || count=0
            if [ "$count" -gt 0 ]; then
                repo_name=$(basename "$repo")
                issue_url="https://github.com/$repo/issues"
                ISSUE_OUTPUT+="| [$repo_name]($issue_url) | $count |"$'\n'
                ISSUE_COUNT=$((ISSUE_COUNT + count))
            fi
        done

        if [ ${#ISSUE_OUTPUT} -gt 0 ]; then
            echo "| Repository | Open Issues |"
            echo "|------------|-------------|"
            printf '%s\n' "$ISSUE_OUTPUT"
            echo "**Total Open Issues**: $ISSUE_COUNT"
        else
            echo "✅ No open issues"
        fi
        echo ""
    fi

    echo "---"
    echo ""
fi

#######################################
# SECTION: LATEST TEST STATUS
#######################################

SUMMARY_JSON=""
for _candidate in \
    "${MAIN_ROOT:-$ROOT_DIR}/.agent/scratchpad/test_summary.json" \
    "${MAIN_ROOT:-$ROOT_DIR}/.scratchpad/test_summary.json"; do
    if [ -f "$_candidate" ]; then
        SUMMARY_JSON="$_candidate"
        break
    fi
done
if [ -n "$SUMMARY_JSON" ]; then
    SECTION=$((SECTION + 1))
    echo "## $SECTION. Latest Test Results"
    python3 -c "
import json, sys
try:
    with open('$SUMMARY_JSON') as f:
        data = json.load(f)
    ts = data.get('timestamp', 'Unknown')
    overall = '✅ Passed' if data.get('overall_success') else '❌ Failed'
    print(f'**Date**: {ts}')
    print(f'**Overall**: {overall}')
    print('')
    print('| Layer | Result | Summary |')
    print('|---|---|---|')
    for layer in data.get('layers', []):
        r = layer.get('result')
        res = '✅ Passed' if r == 'passed' else ('⏭️ Skipped' if r == 'skipped' else '❌ Failed')
        summ = layer.get('summary', '')
        name = layer.get('name')
        print(f'| {name} | {res} | {summ} |')
except Exception as e:
    print(f'Error parsing test summary: {e}')
"
    echo ""
    echo "---"
    echo ""
fi

#######################################
# DONE
#######################################

echo "✅ **Dashboard complete**"
