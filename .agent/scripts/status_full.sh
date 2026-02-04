#!/bin/bash

# Enhanced ROS2 Agent Workspace Status Report Script
# Provides comprehensive status including repository sync, GitHub PRs, and open issues.
#
# Usage:
#   status_full.sh [OPTIONS]
#
# OPTIONS:
#   --skip-sync      Skip repository fetch step (faster, may show stale data)
#   --skip-github    Skip GitHub PR/issue queries (offline mode)
#   --help           Show this help message
#
# EXAMPLES:
#   status_full.sh                    # Full status with sync and GitHub
#   status_full.sh --skip-sync        # Quick check without network fetch
#   status_full.sh --skip-github      # Local status only (offline)
#
# DEPENDENCIES:
#   Required: vcs, git, python3, jq
#   Optional: gh (GitHub CLI) - for PR/issue tracking

# Note: do not use 'set -e' here; many status checks are best-effort and may fail.

# Suppress Python deprecation warnings from vcstool
export PYTHONWARNINGS="ignore::DeprecationWarning"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

# Parse command-line arguments
SKIP_SYNC=false
SKIP_GITHUB=false

while [[ $# -gt 0 ]]; do
    case $1 in
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

# Worktree detection - adjust paths based on context
WORKTREE_INFO=""
if [[ "$ROOT_DIR" == *"/layers/worktrees/"* ]]; then
    WORKTREE_INFO="layer worktree"
    LAYERS_DIR="$ROOT_DIR"  # In layer worktree, ROOT_DIR contains *_ws dirs
elif [[ "$ROOT_DIR" == *"/.workspace-worktrees/"* ]]; then
    WORKTREE_INFO="workspace worktree"
    LAYERS_DIR="$ROOT_DIR/layers/main"
else
    LAYERS_DIR="$ROOT_DIR/layers/main"
fi

echo "# Enhanced Workspace Status Report"
echo "**Date**: $(date)"
if [ -n "$WORKTREE_INFO" ]; then
    echo "**Context**: Running in $WORKTREE_INFO"
fi
echo ""

#######################################
# STEP 1: SYNC REPOSITORIES
#######################################

if [ "$SKIP_SYNC" = false ]; then
    echo "## Step 1: Syncing Repositories"
    echo ""
    
    # Sync root repository
    cd "$ROOT_DIR"
    echo -n "Syncing root repository... "
    if git fetch --quiet 2>/dev/null; then
        echo "✅"
    else
        echo "⚠️ (fetch failed or skipped)"
    fi
    
    # Sync layer repositories
    if command -v vcs &> /dev/null; then
        for ws_dir in "$LAYERS_DIR"/*; do
            if [ -d "$ws_dir/src" ]; then
                ws_name=$(basename "$ws_dir" | sed 's/_ws//')
                echo -n "Syncing $ws_name workspace... "
                
                cd "$ws_dir/src"
                if vcs custom --git --args fetch --quiet >/dev/null 2>&1; then
                    echo "✅"
                else
                    echo "⚠️ (some repos may have failed)"
                fi
            fi
        done
    else
        echo "⚠️ vcs command not found - skipping layer sync"
    fi
    
    echo ""
    echo "---"
    echo ""
fi

#######################################
# STEP 2: REPOSITORY STATUS
#######################################

STEP_NUM=1
if [ "$SKIP_SYNC" = false ]; then
    STEP_NUM=2
fi

echo "## Step $STEP_NUM: Repository Status"
echo ""

# Root Repository Status
echo "### Root Repository"
cd "$ROOT_DIR"
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
else
    echo "- **Status**: ❓ Git not found"
fi
echo ""

# Layer Repositories (VCS)
if ! command -v vcs &> /dev/null; then
    echo "### Layers"
    echo "**Error**: \`vcs\` command not found. Please install \`python3-vcstool\`."
else
    # Fetch expected repositories (including underlay) for tracking checks
    EXPECTED_REPOS=$(python3 "$SCRIPT_DIR/list_overlay_repos.py" --include-underlay --format names 2>/dev/null)
    if [ $? -ne 0 ] || [ -z "$EXPECTED_REPOS" ]; then
        EXPECTED_REPOS=""
    fi

    for ws_dir in "$LAYERS_DIR"/*; do
        if [ -d "$ws_dir/src" ]; then
            ws_name=$(basename "$ws_dir" | sed 's/_ws//')
            
            cd "$ws_dir/src"
            
            # Get status with branch info
            raw_output=$(vcs custom --git --args status --porcelain -b 2>/dev/null || echo "")
            
            clean_count=0
            modified_count=0
            modified_repos=()
            
            # Process the output
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
                    
                    # Fetch expected branch from config
                    if [ -f "$SCRIPT_DIR/get_repo_info.py" ]; then
                        expected_branch=$(python3 "$SCRIPT_DIR/get_repo_info.py" "$current_repo" 2>/dev/null)
                    else
                        expected_branch="unknown"
                    fi
                    
                    if [ "$expected_branch" != "unknown" ] && [ -n "$expected_branch" ]; then
                         if [ "$branch" != "$expected_branch" ]; then
                            warning="🔀 $branch (Want: $expected_branch)"
                            status_str="${status_str:+$status_str, }$warning"
                         fi
                    elif [ "$branch" != "jazzy" ] && [ "$current_repo" != "ros2_agent_workspace" ]; then
                         status_str="${status_str:+$status_str, }🔀 Non-Jazzy?"
                    fi

                    if [ -n "$EXPECTED_REPOS" ]; then
                        if ! echo "$EXPECTED_REPOS" | grep -qx "$current_repo"; then
                             status_str="${status_str:+$status_str, }❓ Untracked"
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
                    # Process previous repo
                    process_repo
                    
                    # Start new repo
                    current_repo=$(echo "$line" | sed 's/^=== \(.*\) (git) ===$/\1/')
                    is_dirty=false
                    sync_status=""
                    branch=""
                elif [[ "$line" =~ ^##[[:space:]](.*)$ ]]; then
                    # Extract branch and sync status
                    branch_line="${BASH_REMATCH[1]}"
                    if [[ "$branch_line" =~ \.\.\..*\[ahead[[:space:]]([0-9]+)\] ]]; then
                        sync_status="⬆️ Ahead ${BASH_REMATCH[1]}"
                    elif [[ "$branch_line" =~ \.\.\..*\[behind[[:space:]]([0-9]+)\] ]]; then
                        sync_status="⬇️ Behind ${BASH_REMATCH[1]}"
                    elif [[ "$branch_line" =~ \.\.\..*\[ahead[[:space:]]([0-9]+),[[:space:]]behind[[:space:]]([0-9]+)\] ]]; then
                        sync_status="↕️ Ahead ${BASH_REMATCH[1]}, Behind ${BASH_REMATCH[2]}"
                    fi
                    # Extract branch name (first word)
                    branch=$(echo "$branch_line" | awk '{print $1}')
                elif [[ "$line" =~ ^[[:space:]]?[MADRCU?!] ]]; then
                    is_dirty=true
                fi
            done <<< "$raw_output"
            
            # Handle last repo
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
fi

echo "---"
echo ""

#######################################
# STEP 3: GITHUB PULL REQUESTS
#######################################

if [ "$SKIP_GITHUB" = false ] && command -v gh &> /dev/null; then
    # Ensure jq is available for GitHub-related JSON parsing
    if ! command -v jq &> /dev/null; then
        echo "Error: 'jq' is required for GitHub status checks but is not installed." >&2
        echo "Install 'jq' (https://stedolan.github.io/jq/) or re-run this script with --skip-github to skip GitHub checks." >&2
        SKIP_GITHUB=true
    fi
fi

if [ "$SKIP_GITHUB" = false ] && command -v gh &> /dev/null; then
    # Generate repository list once for both PR and Issues sections
    REPOS=$(python3 "$SCRIPT_DIR/list_overlay_repos.py" --include-underlay 2>/dev/null | jq -r '.[].url' 2>/dev/null | sed 's|https://github.com/||' | sed 's|.git$||' || true)
    
    # Add root repository
    ROOT_REPO=$(cd "$ROOT_DIR" && git remote get-url origin 2>/dev/null | sed 's|git@github.com:||' | sed 's|https://github.com/||' | sed 's|.git$||' || true)
    
    if [ -n "$ROOT_REPO" ]; then
        REPOS=$(echo -e "$ROOT_REPO\n$REPOS")
    fi
    
    # Remove duplicates and sort
    REPOS=$(echo "$REPOS" | sort -u | grep -v '^$')
fi

if [ "$SKIP_GITHUB" = false ]; then
    STEP_NUM=$((STEP_NUM + 1))
    echo "## Step $STEP_NUM: GitHub Pull Requests"
    echo ""
    
    if ! command -v gh &> /dev/null; then
        echo "⚠️ **GitHub CLI (\`gh\`) not found**"
        echo ""
        echo "Install with: \`sudo apt install gh\` or \`brew install gh\`"
        echo "Then authenticate: \`gh auth login\`"
        echo ""
    elif ! gh auth status &> /dev/null; then
        echo "⚠️ **GitHub CLI not authenticated**"
        echo ""
        echo "Please authenticate with: \`gh auth login\`"
        echo "This is required to fetch PR and issue information."
        echo ""
    else
        # Repository list already generated above
        
        # Query PRs for each repository
        PR_COUNT=0
        PR_OUTPUT=""
        
        for repo in $REPOS; do
            if [ -z "$repo" ]; then
                continue
            fi
            
            # Query GitHub API for open PRs
            prs=$(gh pr list --repo "$repo" --json number,title,url --limit 100 2>/dev/null || echo "[]")
            
            if [ -n "$prs" ] && [ "$prs" != "[]" ]; then
                # Process each PR
                while read -r pr_line; do
                    if [ -z "$pr_line" ]; then
                        continue
                    fi
                    
                    number=$(echo "$pr_line" | jq -r '.number')
                    title=$(echo "$pr_line" | jq -r '.title')
                    url=$(echo "$pr_line" | jq -r '.url')
                    
                    # Truncate title if too long
                    if [ ${#title} -gt 60 ]; then
                        title="${title:0:57}..."
                    fi
                    
                    # Format as table row
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
    fi
    
    echo "---"
    echo ""
fi

#######################################
# STEP 4: GITHUB ISSUES
#######################################

if [ "$SKIP_GITHUB" = false ]; then
    STEP_NUM=$((STEP_NUM + 1))
    echo "## Step $STEP_NUM: GitHub Issues"
    echo ""
    
    if ! command -v gh &> /dev/null; then
        echo "⚠️ **GitHub CLI (\`gh\`) not found** (same as above)"
        echo ""
    else
        # Check if REPOS variable is set (should be from PR section above)
        if [ -z "${REPOS:-}" ]; then
            echo "ℹ️  No repositories available for GitHub issue query."
            echo ""
        else
            # Query issues for each repository
            ISSUE_COUNT=0
            ISSUE_OUTPUT=""
            
            for repo in $REPOS; do
                if [ -z "$repo" ]; then
                    continue
                fi
                
                # Query GitHub API for open issue count using search API (accurate total_count)
                count=$(gh api -X GET search/issues -f q="repo:$repo is:issue is:open" --jq '.total_count' 2>/dev/null || echo "0")
                
                # Ensure count is a valid integer
                if ! [[ "$count" =~ ^[0-9]+$ ]]; then
                    count=0
                fi
                
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
        fi
        echo ""
    fi
    
    echo "---"
    echo ""
fi

#######################################
# STEP 5: LATEST TEST STATUS
#######################################

SUMMARY_JSON="$ROOT_DIR/.agent/scratchpad/test_summary.json"
if [ -f "$SUMMARY_JSON" ]; then
    STEP_NUM=$((STEP_NUM + 1))
    echo "## Step $STEP_NUM: Latest Test Status"
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
# COMPLETION
#######################################

echo "✅ **Status report complete**"
