#!/bin/bash
# .agent/scripts/env.sh
# Source ROS 2 environment for all workspace layers in correct order.
#
# This script is worktree-aware: when run from within a worktree,
# it sources from that worktree's layers instead of the main workspace.

# 1. Base ROS 2 Environment
# Ensure we start from a clean Jazzy base.
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    # Check if a workspace is already sourced that IS NOT this one or the base
    if [ -n "${COLCON_PREFIX_PATH:-}" ] && [[ ! "${COLCON_PREFIX_PATH:-}" == *"/opt/ros/jazzy"* ]]; then
        echo "  ! Warning: Another ROS 2 workspace might be active. Sourcing Jazzy base now."
    fi
    source /opt/ros/jazzy/setup.bash
else
    echo "  ! Error: /opt/ros/jazzy/setup.bash not found. Please install ROS 2 Jazzy."
    return 1 2>/dev/null || exit 1
fi

# Determine workspace root directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

# Detect if we're in a worktree and adjust paths accordingly
LAYERS_BASE="$ROOT_DIR/layers"
WORKTREE_CONTEXT=""

# Check if we're in a layer worktree (layers/worktrees/issue-N/)
if [[ "$ROOT_DIR" == *"/layers/worktrees/"* ]]; then
    # We're in a layer worktree - use this worktree's layers
    WORKTREE_CONTEXT="layer"
    # The worktree root contains the layers directly
    # But we need to check if layers exist here or fall back to main
    if [ -d "$ROOT_DIR/layers" ]; then
        LAYERS_BASE="$ROOT_DIR/layers"
    fi
    echo "  ℹ Worktree detected: layer worktree"
elif [[ "$ROOT_DIR" == *"/.workspace-worktrees/"* ]]; then
    # We're in a workspace worktree
    WORKTREE_CONTEXT="workspace"
    echo "  ℹ Worktree detected: workspace worktree"
fi

# Export worktree info for other scripts
export ROS2_WORKSPACE_ROOT="$ROOT_DIR"
export ROS2_LAYERS_BASE="$LAYERS_BASE"
[ -n "$WORKTREE_CONTEXT" ] && export ROS2_WORKTREE_TYPE="$WORKTREE_CONTEXT"

# 2. Workspace Layers
# Define the order of layers to source. Order determines overlay priority (last one is top).
LAYERS_CONFIG="$LAYERS_BASE/core_ws/src/unh_marine_autonomy/config/layers.txt"

if [ -f "$LAYERS_CONFIG" ]; then
    # Read non-empty lines into array
    mapfile -t LAYERS < <(grep -v '^[[:space:]]*$' "$LAYERS_CONFIG" | grep -v '^#')
else
    # Fallback/Bootstrap layers
    echo "  ! Warning: Layer config not found at $LAYERS_CONFIG. Using defaults."
    LAYERS=("underlay" "core" "platforms" "sensors" "simulation" "ui")
fi

echo "Sourcing ROS2 Agent Workspace layers..."

for layer in "${LAYERS[@]}"; do
    SETUP_FILE="$LAYERS_BASE/${layer}_ws/install/setup.bash"
    if [ -f "$SETUP_FILE" ]; then
        echo "  - Sourcing $layer..."
        source "$SETUP_FILE"
    else
        if [ -d "$LAYERS_BASE/${layer}_ws/src" ]; then
             echo "  ! Warning: $layer exists but is not built (setup.bash not found)."
        fi
    fi
done

echo "Environment ready."
