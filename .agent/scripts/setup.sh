#!/bin/bash
# scripts/setup.sh
# Sets up a ROS2 layer by importing repositories from a .repos file
#
# Usage:
#   ./scripts/setup.sh              # Auto-setup all layers from manifest repo
#   ./scripts/setup.sh <layer_name> # Setup a specific layer
#
# Example: ./scripts/setup.sh core
#
# This script will:
# 1. Bootstrap the manifest repository (if needed)
# 2. Create configs/manifest symlink to the manifest repo's config directory
# 3. Create layers/main/<layer_name>_ws/src directory
# 4. Import repositories from the manifest's repos/<layer_name>.repos file
#
# The manifest repo is the repository that provides layer definitions and .repos
# files. It supports two patterns:
#
#   Pattern A (standalone manifest): layer field absent in bootstrap.yaml
#     -> cloned to configs/manifest_repo/<repo_name>/
#
#   Pattern B (manifest with ROS packages): layer field present in bootstrap.yaml
#     -> cloned into layers/main/<layer>_ws/src/<repo_name>/ so colcon finds packages

set -e

LAYER_NAME=$1

# Bootstrapping Configuration
BOOTSTRAP_URL_FILE="configs/project_bootstrap.url"
MANIFEST_SYMLINK="configs/manifest"

# Function to bootstrap the manifest repository if needed
bootstrap_manifest_repo() {
    # If the symlink already exists and resolves, we're bootstrapped
    if [ -d "$MANIFEST_SYMLINK" ]; then
        return 0
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
    # git_url, branch, layer, and config_path values used to clone and
    # configure the manifest repo. This is trusted because the URL file is
    # committed to this repository and points to a known project-controlled
    # location. If stronger integrity guarantees are needed in the future,
    # consider pinning to a specific commit hash.
    echo "Fetching bootstrap config from $REMOTE_CONFIG_URL..."
    TEMP_CONFIG=$(mktemp)
    trap 'rm -f "$TEMP_CONFIG"' EXIT

    if ! curl -sSLf "$REMOTE_CONFIG_URL" -o "$TEMP_CONFIG"; then
        echo "Error: Failed to download bootstrap config from $REMOTE_CONFIG_URL"
        echo "Please ensure the remote file exists and is accessible."
        exit 1
    fi

    MANIFEST_REPO_URL=$(grep "^git_url:" "$TEMP_CONFIG" | cut -d '#' -f 1 | awk '{print $2}')
    MANIFEST_REPO_BRANCH=$(grep "^branch:" "$TEMP_CONFIG" | cut -d '#' -f 1 | awk '{print $2}')
    MANIFEST_REPO_LAYER=$(grep "^layer:" "$TEMP_CONFIG" | cut -d '#' -f 1 | awk '{print $2}')
    MANIFEST_REPO_CONFIG_PATH=$(grep "^config_path:" "$TEMP_CONFIG" | cut -d '#' -f 1 | awk '{print $2}')

    if [ -z "$MANIFEST_REPO_URL" ] || [ -z "$MANIFEST_REPO_BRANCH" ]; then
        echo "Error: Failed to parse 'git_url' or 'branch' from bootstrap config."
        exit 1
    fi

    # Default config_path to "config" if not specified
    MANIFEST_REPO_CONFIG_PATH="${MANIFEST_REPO_CONFIG_PATH:-config}"

    # Derive repo name from URL
    MANIFEST_REPO_NAME=$(basename "$MANIFEST_REPO_URL" .git)

    # Determine clone target based on whether layer is specified
    if [ -n "$MANIFEST_REPO_LAYER" ]; then
        # Pattern B: manifest repo contains ROS packages, clone into a layer's src/
        MANIFEST_CLONE_DIR="layers/main/${MANIFEST_REPO_LAYER}_ws/src/${MANIFEST_REPO_NAME}"
        echo "Cloning manifest repo (Pattern B: layer=$MANIFEST_REPO_LAYER)..."
        mkdir -p "layers/main/${MANIFEST_REPO_LAYER}_ws/src"
    else
        # Pattern A: standalone manifest repo (config only)
        MANIFEST_CLONE_DIR="configs/manifest_repo/${MANIFEST_REPO_NAME}"
        echo "Cloning manifest repo (Pattern A: standalone config)..."
        mkdir -p "configs/manifest_repo"
    fi

    echo "  From: $MANIFEST_REPO_URL (branch: $MANIFEST_REPO_BRANCH)"
    echo "  To:   $MANIFEST_CLONE_DIR"
    git clone -b "$MANIFEST_REPO_BRANCH" "$MANIFEST_REPO_URL" "$MANIFEST_CLONE_DIR"

    # Create configs/manifest symlink pointing to the config directory
    mkdir -p configs
    MANIFEST_CONFIG_DIR="${MANIFEST_CLONE_DIR}/${MANIFEST_REPO_CONFIG_PATH}"
    if [ ! -d "$MANIFEST_CONFIG_DIR" ]; then
        echo "Error: Config directory not found at $MANIFEST_CONFIG_DIR"
        echo "Check the 'config_path' value in bootstrap.yaml (current: $MANIFEST_REPO_CONFIG_PATH)"
        exit 1
    fi

    # Use relative symlink for relocatability
    SYMLINK_TARGET=$(realpath --relative-to="configs" "$MANIFEST_CONFIG_DIR")
    ln -sfn "$SYMLINK_TARGET" "$MANIFEST_SYMLINK"
    echo "Created symlink: $MANIFEST_SYMLINK -> $SYMLINK_TARGET"

    # Create .agent/project_knowledge symlink if agent_context/ exists in config
    if [ -d "${MANIFEST_CONFIG_DIR}/agent_context" ]; then
        KNOWLEDGE_TARGET=$(realpath --relative-to=".agent" "${MANIFEST_CONFIG_DIR}/agent_context")
        ln -sfn "$KNOWLEDGE_TARGET" ".agent/project_knowledge"
        echo "Created symlink: .agent/project_knowledge -> $KNOWLEDGE_TARGET"
    fi
}

# If no layer specified, auto-setup all layers from project config
if [ -z "$LAYER_NAME" ]; then
    # Ensure the manifest repo is bootstrapped
    bootstrap_manifest_repo

    # Check for layers.txt
    LAYERS_FILE="$MANIFEST_SYMLINK/layers.txt"
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
        echo "All layers set up successfully!"
        echo "========================================="
        echo ""
        echo "Next steps:"
        echo "  1. Source environment: source .agent/scripts/env.sh"
        echo "  2. Build all layers:   .agent/scripts/build.sh"
        exit 0
    else
        echo "Warning: $LAYERS_FILE not found in manifest repository."
        echo "Falling back to default layer: core"
        echo ""
        echo "Available layers:"
        # shellcheck disable=SC2011
        ls "$MANIFEST_SYMLINK/repos/"*.repos 2>/dev/null | xargs -n 1 basename | sed 's/.repos//' | sort || echo "  (none found)"
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
bootstrap_manifest_repo

# Locate the configuration file via the manifest symlink
CONFIG_FILE="$MANIFEST_SYMLINK/repos/${LAYER_NAME}.repos"

if [ ! -f "$CONFIG_FILE" ]; then
    echo "Error: Configuration file not found: $CONFIG_FILE"
    echo ""
    echo "Available layers:"
    # shellcheck disable=SC2011
    ls "$MANIFEST_SYMLINK/repos/"*.repos 2>/dev/null | xargs -n 1 basename | sed 's/.repos//' | sort || echo "  (none found)"
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
