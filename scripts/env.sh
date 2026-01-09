#!/bin/bash

# Define the order of workspaces to source
# You can customize this list. Order determines overlay priority (last one is top).
LAYERS=("underlay" "core" "ui")

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"

echo "Sourcing ROS2 Agent Workspace layers..."

for layer in "${LAYERS[@]}"; do
    SETUP_FILE="$ROOT_DIR/workspaces/${layer}_ws/install/setup.bash"
    if [ -f "$SETUP_FILE" ]; then
        echo "  - Sourcing $layer..."
        source "$SETUP_FILE"
    else
        # Fallback to local_setup if install/setup.bash isn't fully ready or for specific overlay behavior
        # But generally source install/setup.bash to get the chain.
        # If the layer hasn't been built yet, warn.
        if [ -d "$ROOT_DIR/workspaces/${layer}_ws/src" ]; then
             echo "  ! Warning: $layer exists but is not built (setup.bash not found)."
        fi
    fi
done

echo "Environment ready."
