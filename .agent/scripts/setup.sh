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

# Bootstrapping Configuration
BOOTSTRAP_REPO_DIR="workspaces/core_ws/src/unh_marine_autonomy"
BOOTSTRAP_CONFIG="configs/bootstrap.repos"

if [ -z "$WORKSPACE_NAME" ]; then
    echo "Usage: $0 <workspace_name>"
    echo "Available configs:"
    # Check both old and new locations for listing
    ls configs/*.repos "$BOOTSTRAP_REPO_DIR/config/repos/"*.repos 2>/dev/null | xargs -n 1 basename | sed 's/.repos//' | sort | uniq
    exit 1
fi

# 1. Bootstrap if necessary (ensure unh_marine_autonomy exists)
# We need this repo to access the other .repos files
if [ ! -d "$BOOTSTRAP_REPO_DIR" ]; then
    if [ -f "$BOOTSTRAP_CONFIG" ]; then
        echo "Bootstrapping: Cloning unh_marine_autonomy..."
        mkdir -p workspaces/core_ws/src
        if command -v vcs &> /dev/null; then
            vcs import workspaces/core_ws/src < "$BOOTSTRAP_CONFIG"
        else
            echo "Error: 'vcs' command not found. Please run .agent/scripts/bootstrap.sh"
            exit 1
        fi
    else
        echo "Warning: Bootstrap config $BOOTSTRAP_CONFIG not found."
    fi
fi

# 2. Locate the configuration file
if [ -f "configs/${WORKSPACE_NAME}.repos" ]; then
    CONFIG_FILE="configs/${WORKSPACE_NAME}.repos"
elif [ -f "$BOOTSTRAP_REPO_DIR/config/repos/${WORKSPACE_NAME}.repos" ]; then
    CONFIG_FILE="$BOOTSTRAP_REPO_DIR/config/repos/${WORKSPACE_NAME}.repos"
else
    echo "Error: Configuration file for '$WORKSPACE_NAME' not found."
    echo "Checked: configs/${WORKSPACE_NAME}.repos"
    echo "Checked: $BOOTSTRAP_REPO_DIR/config/repos/${WORKSPACE_NAME}.repos"
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
    echo "  ./.agent/scripts/bootstrap.sh"
fi

echo "Setup complete for $WORKSPACE_NAME."
echo "To build manually:"
echo "  cd $WORKSPACE_DIR"
echo "  colcon build --symlink-install"
echo ""
echo "Or use the unified build script:"
echo "  ./.agent/scripts/build.sh"

