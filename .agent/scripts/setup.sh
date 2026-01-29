#!/bin/bash
# scripts/setup.sh
# Sets up a ROS2 layer by importing repositories from a .repos file
#
# Usage: ./scripts/setup.sh <layer_name>
# Example: ./scripts/setup.sh core
#
# This script will:
# 1. Create layers/<layer_name>_ws/src directory
# 2. Import repositories from configs/<layer_name>.repos using vcs tool
#
# Available layers:
#   - underlay: Additional dependencies
#   - core: Main autonomy framework (Project11, AIS, Navigation)
#   - platforms: Platform-specific packages
#   - sensors: Sensor drivers and perception
#   - simulation: Simulation tools
#   - ui: Visualization and user interfaces

set -e

LAYER_NAME=$1
LAYER_DIR="layers/${LAYER_NAME}_ws"

# Bootstrapping Configuration
# Bootstrapping Configuration
BOOTSTRAP_URL_FILE="configs/project_bootstrap.url"
# Default to assuming core_ws/src/unh_marine_autonomy if we can't determine otherwise yet,
# but really we should read this from the fetched config.
# For now, we'll assume the Key Repo goes into core_ws/src/unh_marine_autonomy until we parse the config.
KEY_REPO_TARGET_DIR="layers/core_ws/src/unh_marine_autonomy"

if [ -z "$LAYER_NAME" ]; then
    echo "Usage: $0 <layer_name>"
    echo "Available configs:"
    # Check both old and new locations for listing
    ls configs/*.repos "$KEY_REPO_TARGET_DIR/config/repos/"*.repos 2>/dev/null | xargs -n 1 basename | sed 's/.repos//' | sort | uniq
    exit 1
fi

# 1. Bootstrap if necessary
# We check if the Key Repo exists. If not, we try to clone it using info from project_bootstrap.url
if [ ! -d "$KEY_REPO_TARGET_DIR" ]; then
    if [ -f "$BOOTSTRAP_URL_FILE" ]; then
        echo "Bootstrapping: Reading configuration from $BOOTSTRAP_URL_FILE..."
        REMOTE_CONFIG_URL=$(cat "$BOOTSTRAP_URL_FILE" | tr -d '[:space:]')
        
        echo "Fetching bootstrap config from $REMOTE_CONFIG_URL..."
        # Create a temp file for the config
        TEMP_CONFIG=$(mktemp)
        if curl -sSLf "$REMOTE_CONFIG_URL" -o "$TEMP_CONFIG"; then
            # Parse YAML (robust grep/awk approach)
            # 1. grep for key
            # 2. cut at comment (#)
            # 3. awk to print value (handles leading/trailing whitespace)
            KEY_REPO_URL=$(grep "^git_url:" "$TEMP_CONFIG" | cut -d '#' -f 1 | awk '{print $2}')
            KEY_REPO_BRANCH=$(grep "^branch:" "$TEMP_CONFIG" | cut -d '#' -f 1 | awk '{print $2}')
            
            rm "$TEMP_CONFIG"

            if [ -n "$KEY_REPO_URL" ] && [ -n "$KEY_REPO_BRANCH" ]; then
                echo "Cloning Key Repo from $KEY_REPO_URL (branch: $KEY_REPO_BRANCH)..."
                mkdir -p layers/core_ws/src
                git clone -b "$KEY_REPO_BRANCH" "$KEY_REPO_URL" "$KEY_REPO_TARGET_DIR"
            else
                echo "Error: Failed to parse 'git_url' or 'branch' from bootstrap config."
                exit 1
            fi
        else
            echo "Error: Failed to download bootstrap config from $REMOTE_CONFIG_URL"
            echo "Please ensure the remote file exists and is accessible."
            exit 1
        fi
    elif [ -f "configs/bootstrap.repos" ]; then
         # Fallback to legacy behavior if file exists
         echo "Warning: Using legacy configs/bootstrap.repos"
         mkdir -p layers/core_ws/src
         vcs import layers/core_ws/src < configs/bootstrap.repos
    else
        echo "Error: No bootstrap configuration found (checked $BOOTSTRAP_URL_FILE)."
        exit 1
    fi
fi

# 2. Locate the configuration file
if [ -f "configs/${LAYER_NAME}.repos" ]; then
    CONFIG_FILE="configs/${LAYER_NAME}.repos"
elif [ -f "$KEY_REPO_TARGET_DIR/config/repos/${LAYER_NAME}.repos" ]; then
    CONFIG_FILE="$KEY_REPO_TARGET_DIR/config/repos/${LAYER_NAME}.repos"
else
    echo "Error: Configuration file for '$LAYER_NAME' not found."
    echo "Checked: configs/${LAYER_NAME}.repos"
    echo "Checked: $KEY_REPO_TARGET_DIR/config/repos/${LAYER_NAME}.repos"
    exit 1
fi

echo "Setting up layer: $LAYER_NAME"
mkdir -p "$LAYER_DIR/src"

if command -v vcs &> /dev/null; then
    echo "Importing repositories into $LAYER_DIR/src..."
    vcs import "$LAYER_DIR/src" < "$CONFIG_FILE"
else
    echo "Warning: 'vcs' command not found. Skipping repository import."
    echo "To install all necessary dependencies, run:"
    echo "  ./.agent/scripts/bootstrap.sh"
fi

echo "Setup complete for $LAYER_NAME."
echo "To build manually:"
echo "  cd $LAYER_DIR"
echo "  colcon build --symlink-install"
echo ""
echo "Or use the unified build script:"
echo "  ./.agent/scripts/build.sh"

