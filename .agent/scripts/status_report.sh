#!/bin/bash

# ROS2 Agent Workspace Status Report Script
# Generates a concise Markdown report of the root git repo and all sub-repos.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

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

# Helper function to print a table row
print_row() {
    printf "| %-30s | %-20s | %-30s |\n" "$1" "$2" "$3"
}

# Helper function to print table header
print_header() {
    print_row "Repository" "Status" "Branch"
    print_row "---" "---" "---"
}

echo "# Workspace Status Report"
echo "**Date**: $(date)"
if [ -n "$WORKTREE_INFO" ]; then
    echo "**Context**: Running in $WORKTREE_INFO"
fi
echo ""

# 1. Root Repository Status
echo "## Root Repository"
cd "$ROOT_DIR"
if command -v git &> /dev/null; then
    # Check for modifications
    if ! git fetch -q 2>/dev/null; then
        echo "- **Note**: \`git fetch\` failed/skipped; status may not reflect latest remote state."
    fi
    if [ -n "$(git status --porcelain)" ]; then
        echo "- **Status**: ⚠️ Modified"
        echo "- **Branch**: $(git branch --show-current)"
        echo ""
        echo "### Modified Files"
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

# 2. Layer Repositories (VCS)
if ! command -v vcs &> /dev/null; then
    echo "## Layers"
    echo "**Error**: \`vcs\` command not found. Please install \`python3-vcstool\`."
    # We continue to show test results even if VCS fails
else
    # Fetch expected repositories (including underlay) for tracking checks
    EXPECTED_REPOS=$(python3 "$SCRIPT_DIR/list_overlay_repos.py" --include-underlay --format names 2>/dev/null)
    if [ $? -ne 0 ] || [ -z "$EXPECTED_REPOS" ]; then
        # echo "**Error**: Failed to list expected repositories. Tracking check disabled."
        EXPECTED_REPOS=""
    fi

    for ws_dir in "$LAYERS_DIR"/*; do
        if [ -d "$ws_dir/src" ]; then
            ws_name=$(basename "$ws_dir" | sed 's/_ws//')
            
            cd "$ws_dir/src"
            
            # Fetch updates quietly
            vcs custom --git --args fetch -q >/dev/null 2>&1
            
            raw_output=$(vcs custom --git --args status --porcelain -b)
            
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
                        if [ "$status_str" != "" ]; then
                            status_str="$status_str, $sync_status"
                        else
                            status_str="$sync_status"
                        fi
                    fi
                    
                    # Fetch expected branch from config
                    if [ -f "$SCRIPT_DIR/get_repo_info.py" ]; then
                        expected_branch=$(python3 "$SCRIPT_DIR/get_repo_info.py" "$current_repo")
                    else
                        expected_branch="unknown"
                    fi
                    
                    if [ "$expected_branch" != "unknown" ]; then
                         if [ "$branch" != "$expected_branch" ]; then
                            warning="🔀 $branch (Want: $expected_branch)"
                            status_str="${status_str:+$status_str, }$warning"
                         fi
                    elif [ "$branch" != "jazzy" ] && [ "$current_repo" != "ros2_agent_workspace" ]; then
                         status_str="${status_str:+$status_str, }🔀 Non-Jazzy?"
                    fi

                    if ! echo "$EXPECTED_REPOS" | grep -qx "$current_repo"; then
                         status_str="${status_str:+$status_str, }❓ Untracked"
                    fi
                    
                    if [ "$status_str" != "" ]; then
                        modified_repos+=("$current_repo|$status_str|$branch")
                        ((modified_count++))
                    else
                        ((clean_count++))
                    fi
                fi
            }

            while IFS= read -r line; do
                if [[ "$line" =~ ^===\ \./(.*)\ \(git\)\ ===$ ]]; then
                    process_repo
                    current_repo="${BASH_REMATCH[1]}"
                    is_dirty=false
                    sync_status=""
                    branch="unknown"
                elif [[ "$line" =~ ^##\ ([^.]*)(\.\.\.([^\ ]*))?(\ \[(.*)\])? ]]; then
                     branch="${BASH_REMATCH[1]}"
                     if [ -n "${BASH_REMATCH[5]}" ]; then
                         sync_status="${BASH_REMATCH[5]}"
                         sync_status="${sync_status//ahead/🚀 Ahead}"
                         sync_status="${sync_status//behind/⬇️ Behind}"
                     fi
                elif [[ -n "$line" && "$line" != "" ]]; then
                    is_dirty=true
                fi
            done <<< "$raw_output"
            
            process_repo
            
            total_count=$((clean_count + modified_count))
            echo "## Workspace: **$ws_name** (Total: $total_count, Clean: $clean_count, Attention: $modified_count)"
            
            if [ $modified_count -gt 0 ]; then
                 echo ""
                 echo "### ⚠️ Attention Needed"
                 print_header
                 for repo_info in "${modified_repos[@]}"; do
                     repo_name="${repo_info%%|*}"
                     rest="${repo_info#*|}"
                     repo_status="${rest%%|*}"
                     repo_branch="${rest#*|}"
                     print_row "$repo_name" "$repo_status" "$repo_branch"
                 done
            fi
            echo ""
        fi
    done
fi

# 3. Test Status (New Section)
SUMMARY_JSON="$ROOT_DIR/.agent/scratchpad/test_summary.json"
if [ -f "$SUMMARY_JSON" ]; then
    echo "## Latest Test Status"
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
fi
