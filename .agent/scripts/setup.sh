#!/bin/bash
# scripts/setup.sh
# Sets up a ROS2 layer by importing repositories from a .repos file
#
# Usage: 
#   ./scripts/setup.sh              # Auto-setup all layers from project's layers.txt
#   ./scripts/setup.sh <layer_name> # Setup a specific layer
#
# Example: ./scripts/setup.sh core
#
# This script will:
# 1. Bootstrap the project's key repository (if needed)
# 2. Create layers/main/<layer_name>_ws/src directory
# 3. Import repositories from the project's config/repos/<layer_name>.repos file
#
# Available layers are defined in the project repository (unh_marine_autonomy):
#   - underlay: Additional dependencies (geodesy, audio_common, etc.)
#   - core: Main autonomy framework (Project11, AIS, Navigation)
#   - platforms: Platform-specific packages
#   - sensors: Sensor drivers and perception
#   - simulation: Simulation tools
#   - ui: Visualization and user interfaces

set -e

LAYER_NAME=$1

# Bootstrapping Configuration
BOOTSTRAP_URL_FILE="configs/project_bootstrap.url"
KEY_REPO_TARGET_DIR="layers/main/core_ws/src/unh_marine_autonomy"

# Function to bootstrap the key repository if needed
bootstrap_key_repo() {
    if [ -d "$KEY_REPO_TARGET_DIR" ]; then
        return 0  # Already bootstrapped
    fi
    
    if [ ! -f "$BOOTSTRAP_URL_FILE" ]; then
        echo "Error: Bootstrap configuration not found: $BOOTSTRAP_URL_FILE"
        echo "This file should contain a URL to the project's bootstrap.yaml"
        exit 1
    fi
    
    echo "Bootstrapping: Reading configuration from $BOOTSTRAP_URL_FILE..."
    REMOTE_CONFIG_URL=$(cat "$BOOTSTRAP_URL_FILE" | tr -d '[:space:]')
    
    # Note: The bootstrap config URL is controlled by this project's own
    # configs/project_bootstrap.url file. The downloaded YAML provides
    # git_url and branch values used in git clone. This is trusted because
    # the URL file is committed to this repository and points to a known
    # project-controlled location. If stronger integrity guarantees are
    # needed in the future, consider pinning to a specific commit hash.
    echo "Fetching bootstrap config from $REMOTE_CONFIG_URL..."
    TEMP_CONFIG=$(mktemp)
    trap 'rm -f "$TEMP_CONFIG"' EXIT

    if ! curl -sSLf "$REMOTE_CONFIG_URL" -o "$TEMP_CONFIG"; then
        echo "Error: Failed to download bootstrap config from $REMOTE_CONFIG_URL"
        echo "Please ensure the remote file exists and is accessible."
        exit 1
    fi

    KEY_REPO_URL=$(grep "^git_url:" "$TEMP_CONFIG" | cut -d '#' -f 1 | awk '{print $2}')
    KEY_REPO_BRANCH=$(grep "^branch:" "$TEMP_CONFIG" | cut -d '#' -f 1 | awk '{print $2}')

    if [ -z "$KEY_REPO_URL" ] || [ -z "$KEY_REPO_BRANCH" ]; then
        echo "Error: Failed to parse 'git_url' or 'branch' from bootstrap config."
        exit 1
    fi
    
    echo "Cloning Key Repo from $KEY_REPO_URL (branch: $KEY_REPO_BRANCH)..."
    mkdir -p layers/main/core_ws/src
    git clone -b "$KEY_REPO_BRANCH" "$KEY_REPO_URL" "$KEY_REPO_TARGET_DIR"
}

# If no layer specified, auto-setup all layers from project config
if [ -z "$LAYER_NAME" ]; then
    # Ensure the key repo is bootstrapped
    bootstrap_key_repo
    
    # Check for layers.txt
    LAYERS_FILE="$KEY_REPO_TARGET_DIR/config/layers.txt"
    if [ -f "$LAYERS_FILE" ]; then
        echo "Auto-setup: Reading layers from $LAYERS_FILE..."
        while IFS= read -r layer || [ -n "$layer" ]; do
            # Skip empty lines and comments
            [[ -z "$layer" || "$layer" =~ ^[[:space:]]*# ]] && continue
            layer=$(echo "$layer" | xargs)  # Trim whitespace
            echo ""
            echo "========================================="
            echo "Setting up layer: $layer"
            echo "========================================="
            if ! "$0" "$layer"; then
                echo ""
                echo "Error: Failed to set up layer: $layer"
                echo "Aborting auto-setup."
                exit 1
            fi
        done < "$LAYERS_FILE"
        echo ""
        echo "========================================="
        echo "✅ All layers set up successfully!"
        echo "========================================="
        echo ""
        echo "Next steps:"
        echo "  1. Source environment: source .agent/scripts/env.sh"
        echo "  2. Build all layers:   .agent/scripts/build.sh"
        exit 0
    else
        echo "Warning: $LAYERS_FILE not found in project repository."
        echo "Falling back to default layer: core"
        echo ""
        echo "Available layers:"
        ls "$KEY_REPO_TARGET_DIR/config/repos/"*.repos 2>/dev/null | xargs -n 1 basename | sed 's/.repos//' | sort || echo "  (none found)"
        echo ""
        echo "========================================="
        echo "Setting up default layer: core"
        echo "========================================="
        "$0" core
        exit 0
    fi
fi

# Validate layer name: must be alphanumeric with optional hyphens/underscores
if ! [[ "$LAYER_NAME" =~ ^[a-zA-Z0-9_-]+$ ]]; then
    echo "Error: Invalid layer name '$LAYER_NAME'."
    echo "Layer names must contain only letters, numbers, hyphens, and underscores."
    exit 1
fi

LAYER_DIR="layers/main/${LAYER_NAME}_ws"

# Bootstrap if necessary (for individual layer setup)
bootstrap_key_repo

# Locate the configuration file in the project repository
CONFIG_FILE="$KEY_REPO_TARGET_DIR/config/repos/${LAYER_NAME}.repos"

if [ ! -f "$CONFIG_FILE" ]; then
    echo "Error: Configuration file not found: $CONFIG_FILE"
    echo ""
    echo "Available layers:"
    ls "$KEY_REPO_TARGET_DIR/config/repos/"*.repos 2>/dev/null | xargs -n 1 basename | sed 's/.repos//' | sort || echo "  (none found)"
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

