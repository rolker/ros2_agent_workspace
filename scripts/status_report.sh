#!/bin/bash

# ROS2 Agent Workspace Status Report Script
# This script aggregates the status of the root git repo and all sub-repos.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
WORKSPACES_DIR="$ROOT_DIR/workspaces"

echo "=========================================="
echo "      WORKSPACE STATUS REPORT             "
echo "=========================================="
date
echo ""

# 1. Root Repository Status
echo "------------------------------------------"
echo "ROOT REPOSITORY (ros2_agent_workspace)"
echo "------------------------------------------"
cd "$ROOT_DIR"
if command -v git &> /dev/null; then
    git_status=$(git status --porcelain)
    if [ -z "$git_status" ]; then
        echo "  [OK] Clean"
    else
        echo "  [WARN] Modified:"
        echo "$git_status" | sed 's/^/    /'
    fi
else
    echo "  [ERROR] git command not found."
fi
echo ""

# 2. Workspace Repositories (VCS)
echo "------------------------------------------"
echo "ROS2 WORKSPACES (vcs status)"
echo "------------------------------------------"

if ! command -v vcs &> /dev/null; then
    echo "  [ERROR] vcs command not found. Cannot check workspace repos."
    echo "  Please install python3-vcstool."
    exit 1
fi

# Find all src directories inside workspaces
found_workspaces=false
for ws_dir in "$WORKSPACES_DIR"/*; do
    if [ -d "$ws_dir/src" ]; then
        found_workspaces=true
        ws_name=$(basename "$ws_dir" | sed 's/_ws//')
        echo "Checking workspace: $ws_name ..."
        
        # Capture vcs status output
        # vcs status returns non-empty output if there are local modifications or interesting states
        # The -q flag usually suppresses uninteresting files, but we want to see what's changed.
        cd "$ws_dir/src"
        vcs_output=$(vcs status)
        
        if [ -z "$vcs_output" ]; then
             echo "  [OK] All repositories clean."
        else
             # Indent output for readability
             echo "$vcs_output" | sed 's/^/    /'
        fi
        echo ""
    fi
done

if [ "$found_workspaces" = false ]; then
    echo "  [INFO] No populated workspaces found (no 'src' directories in workspaces/)."
fi

echo "=========================================="
echo "End of Report"
echo "=========================================="
