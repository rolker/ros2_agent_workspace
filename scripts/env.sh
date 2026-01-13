# 1. Base ROS 2 Environment
# Ensure we start from a clean Jazzy base.
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    # Check if a workspace is already sourced that IS NOT this one or the base
    if [ ! -z "${COLCON_PREFIX_PATH:-}" ] && [[ ! "${COLCON_PREFIX_PATH:-}" == *"/opt/ros/jazzy"* ]]; then
        echo "  ! Warning: Another ROS 2 workspace might be active. Sourcing Jazzy base now."
    fi
    source /opt/ros/jazzy/setup.bash
else
    echo "  ! Error: /opt/ros/jazzy/setup.bash not found. Please install ROS 2 Jazzy."
    return 1 2>/dev/null || exit 1
fi

# 2. Workspace Layers
# Define the order of workspaces to source. Order determines overlay priority (last one is top).
LAYERS=("underlay" "core" "platforms" "sensors" "simulation" "ui")

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"

echo "Sourcing ROS2 Agent Workspace layers..."

for layer in "${LAYERS[@]}"; do
    SETUP_FILE="$ROOT_DIR/workspaces/${layer}_ws/install/setup.bash"
    if [ -f "$SETUP_FILE" ]; then
        echo "  - Sourcing $layer..."
        source "$SETUP_FILE"
    else
        if [ -d "$ROOT_DIR/workspaces/${layer}_ws/src" ]; then
             echo "  ! Warning: $layer exists but is not built (setup.bash not found)."
        fi
    fi
done

echo "Environment ready."
