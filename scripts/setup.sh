#!/bin/bash
# scripts/setup.sh
# Sets up a ROS2 workspace layer by importing repositories from a .repos file
#
# Usage: ./scripts/setup.sh <workspace_name>
# Example: ./scripts/setup.sh core
#
# This script will:
# 1. Create workspaces/<workspace_name>_ws/src directory
# 2. Import repositories from configs/<workspace_name>.repos using vcs tool
#
# Available workspace layers:
#   - underlay: Additional dependencies
#   - core: Main autonomy framework (Project11, AIS, Navigation)
#   - platforms: Platform-specific packages
#   - sensors: Sensor drivers and perception
#   - simulation: Simulation tools
#   - ui: Visualization and user interfaces

set -e

WORKSPACE_NAME=$1
WORKSPACE_DIR="workspaces/${WORKSPACE_NAME}_ws"
CONFIG_FILE="configs/${WORKSPACE_NAME}.repos"

if [ -z "$WORKSPACE_NAME" ]; then
    echo "Usage: $0 <workspace_name>"
    echo "Available configs:"
    ls configs/*.repos | xargs -n 1 basename | sed 's/.repos//'
    exit 1
fi

if [ ! -f "$CONFIG_FILE" ]; then
    echo "Error: Configuration file $CONFIG_FILE not found."
    exit 1
fi

echo "Setting up workspace: $WORKSPACE_NAME"
mkdir -p "$WORKSPACE_DIR/src"

if command -v vcs &> /dev/null; then
    echo "Importing repositories into $WORKSPACE_DIR/src..."
    vcs import "$WORKSPACE_DIR/src" < "$CONFIG_FILE"
else
    echo "Warning: 'vcs' command not found. Skipping repository import."
    echo "To install all necessary dependencies, run:"
    echo "  ./scripts/bootstrap.sh"
fi

echo "Setup complete for $WORKSPACE_NAME."
echo "To build manually:"
echo "  cd $WORKSPACE_DIR"
echo "  colcon build --symlink-install"
echo ""
echo "Or use the unified build script:"
echo "  ./scripts/build.sh"

