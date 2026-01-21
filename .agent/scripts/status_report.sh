#!/bin/bash

# ROS2 Agent Workspace Status Report Script
# Generates a concise Markdown report of the root git repo and all sub-repos.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"
WORKSPACES_DIR="$ROOT_DIR/workspaces"

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
echo ""

# 1. Root Repository Status
echo "## Root Repository"
cd "$ROOT_DIR"
if command -v git &> /dev/null; then
    # Check for modifications
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

# 2. Workspace Repositories (VCS)
if ! command -v vcs &> /dev/null; then
    echo "## Workspaces"
    echo "**Error**: \`vcs\` command not found. Please install \`python3-vcstool\`."
    exit 1
fi

for ws_dir in "$WORKSPACES_DIR"/*; do
    if [ -d "$ws_dir/src" ]; then
        ws_name=$(basename "$ws_dir" | sed 's/_ws//')
        # Header printed later with stats
        
        cd "$ws_dir/src"
        
        # Get list of all repos (directories with .git)
        # We use vcs custom to efficiently check status
        # Output format for status --porcelain -b:
        # ## branch...origin/branch [ahead 1]
        #  M file
        # ?? file
        
        raw_output=$(vcs custom --git --args status --porcelain -b)
        
        clean_count=0
        modified_count=0
        modified_repos=()
        
        # Process the output
        current_repo=""
        is_dirty=false
        sync_status=""
        branch=""
        
        # Function to finalize the current repo processing
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
                expected_branch=$(python3 "$SCRIPT_DIR/get_repo_info.py" "$current_repo")
                
                # Check for branch mismatch
                if [ "$expected_branch" != "unknown" ]; then
                     if [ "$branch" != "$expected_branch" ]; then
                        warning="🔀 $branch (Want: $expected_branch)"
                        if [ "$status_str" != "" ]; then
                            status_str="$status_str, $warning"
                        else
                            status_str="$warning"
                        fi
                     fi
                elif [ "$branch" != "jazzy" ] && [ "$current_repo" != "ros2_agent_workspace" ]; then
                     # Fallback for repos not in config (assume Jazzy default)
                     if [ "$status_str" != "" ]; then
                        status_str="$status_str, 🔀 Non-Jazzy?"
                     else
                        status_str="🔀 Non-Jazzy?"
                     fi
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
                # New repo block started, process previous one
                process_repo
                
                current_repo="${BASH_REMATCH[1]}"
                is_dirty=false
                sync_status=""
                branch="unknown"
            elif [[ "$line" =~ ^##\ ([^.]*)(\.\.\.([^\ ]*))?(\ \[(.*)\])? ]]; then
                 # Parse branch line: ## main...origin/main [ahead 1]
                 branch="${BASH_REMATCH[1]}"
                 # ${BASH_REMATCH[5]} captures "ahead 1" or "ahead 1, behind 2"
                 if [ -n "${BASH_REMATCH[5]}" ]; then
                     sync_status="${BASH_REMATCH[5]}"
                     # Add emojis for better visibility
                     sync_status="${sync_status//ahead/🚀 Ahead}"
                     sync_status="${sync_status//behind/⬇️ Behind}"
                 fi
            elif [[ -n "$line" && "$line" != "" ]]; then
                # Any other non-empty line means local changes (A, M, D, ??, etc.)
                is_dirty=true
            fi
        done <<< "$raw_output"
        
        # Handle the last repo
        process_repo
        
        
        total_count=$((clean_count + modified_count))
        
        # Summary Header
        echo "## Workspace: **$ws_name** (Total: $total_count, Clean: $clean_count, Attention: $modified_count)"
        
        if [ $modified_count -gt 0 ]; then
             echo ""
             echo "### ⚠️ Attention Needed"
             print_header
             for repo_info in "${modified_repos[@]}"; do
                 # Split string "repo|status|branch"
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
