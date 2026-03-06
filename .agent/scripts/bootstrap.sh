#!/bin/bash
if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
    echo "Error: This script should be executed, not sourced."
    echo "  Run: ${BASH_SOURCE[0]} $*"
    return 1
fi
set -e

# [TASK-003] Bootstrap Automation
# This script aligns with the official ROS 2 Jazzy installation guide:
# https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

# --- Helpers ---

is_installed() { dpkg -s "$1" 2>/dev/null | grep -q 'Status: install ok installed'; }

DRY_RUN=false
if [ "${1:-}" = "--dry-run" ]; then
    DRY_RUN=true
fi

PENDING_COMMANDS=()
SOMETHING_INSTALLED=false

run_or_collect() {
    if [ "$DRY_RUN" = true ]; then
        PENDING_COMMANDS+=("$*")
    else
        SOMETHING_INSTALLED=true
        "$@"
    fi
}

APT_UPDATED=false
ensure_apt_updated() {
    if [ "$APT_UPDATED" = false ]; then
        run_or_collect sudo apt update
        APT_UPDATED=true
    fi
}

# --- Main ---

if [ "$DRY_RUN" = false ]; then
    echo "Bootstrapping ROS 2 Agent Workspace..."
fi

# 1. Locale Check
if ! locale | grep -q 'LC_ALL=.*UTF-8'; then
    if [ "$DRY_RUN" = false ]; then
        echo "Locale does not appear to support UTF-8. Attempting to set it..."
    fi
    ensure_apt_updated
    run_or_collect sudo apt install -y locales
    run_or_collect sudo locale-gen en_US en_US.UTF-8
    run_or_collect sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    if [ "$DRY_RUN" = false ]; then
        export LANG=en_US.UTF-8
        echo "Locale set to en_US.UTF-8"
    fi
else
    if [ "$DRY_RUN" = false ]; then
        echo "Locale looks good."
    fi
fi

# 2. Setup Sources

# Enable Universe
if ! is_installed software-properties-common; then
    ensure_apt_updated
    run_or_collect sudo apt install -y software-properties-common
else
    [ "$DRY_RUN" = false ] && echo "software-properties-common already installed."
fi

if ! grep -Rqs '^[^#].*universe' /etc/apt/sources.list /etc/apt/sources.list.d/ 2>/dev/null; then
    run_or_collect sudo add-apt-repository -y universe
else
    [ "$DRY_RUN" = false ] && echo "Universe repository already enabled."
fi

# Install curl
if ! is_installed curl; then
    ensure_apt_updated
    run_or_collect sudo apt install -y curl
else
    [ "$DRY_RUN" = false ] && echo "curl already installed."
fi

# Install ros2-apt-source (Handles GPG/Repo automatically)
if ! compgen -G "/etc/apt/sources.list.d/ros2*.list" >/dev/null 2>&1 \
   && ! compgen -G "/etc/apt/sources.list.d/ros2*.sources" >/dev/null 2>&1; then
    if [ "$DRY_RUN" = false ]; then
        echo "Installing ros2-apt-source..."
        ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
        export ROS_APT_SOURCE_VERSION
        if [ -z "${ROS_APT_SOURCE_VERSION}" ]; then
            echo "Error: Failed to determine ROS apt source version from GitHub API." >&2
            exit 1
        fi
        curl -fL -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")_all.deb"

        if [ ! -s /tmp/ros2-apt-source.deb ]; then
            echo "Error: Failed to download ros2-apt-source .deb or file is empty." >&2
            exit 1
        fi
        SOMETHING_INSTALLED=true
        sudo dpkg -i /tmp/ros2-apt-source.deb
        rm /tmp/ros2-apt-source.deb
        APT_UPDATED=false  # New source added, need fresh apt update
    else
        PENDING_COMMANDS+=("sudo dpkg -i /tmp/ros2-apt-source.deb (version auto-detected from GitHub)")
        APT_UPDATED=false
    fi
else
    [ "$DRY_RUN" = false ] && echo "ROS 2 apt source already configured."
fi

# 3. Install Dependencies

if ! is_installed ros-dev-tools; then
    ensure_apt_updated
    run_or_collect sudo apt install -y ros-dev-tools
else
    [ "$DRY_RUN" = false ] && echo "ros-dev-tools already installed."
fi

# Check for ROS 2 Base
if ! is_installed ros-jazzy-ros-base; then
    ensure_apt_updated
    run_or_collect sudo apt install -y ros-jazzy-ros-base
else
    [ "$DRY_RUN" = false ] && echo "ros-jazzy-ros-base already installed."
fi

# 4. Initialize rosdep
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    run_or_collect sudo rosdep init
else
    [ "$DRY_RUN" = false ] && echo "rosdep already initialized."
fi

if [ "$DRY_RUN" = false ]; then
    rosdep update
fi

# 5. Codespace Configuration
if [ "${CODESPACES}" = "true" ] && [ "$DRY_RUN" = false ]; then
    echo "Detected GitHub Codespaces environment."
    echo "Converting SSH URLs to HTTPS in .repos files..."
    while IFS= read -r -d '' repos_file; do
        # Create a backup before in-place modification to avoid inconsistent state on failure
        if ! sed -i.bak 's|git@github.com:|https://github.com/|g' "${repos_file}"; then
            echo "Error: Failed to convert SSH URLs to HTTPS in '${repos_file}'." >&2
            exit 1
        fi
        # Remove backup file after successful conversion
        rm -f "${repos_file}.bak"
    done < <(find -L configs -name "*.repos" -print0)
    echo "URL conversion complete."
fi

# --- Output ---

if [ "$DRY_RUN" = true ]; then
    if [ ${#PENDING_COMMANDS[@]} -eq 0 ]; then
        echo "All prerequisites already installed — nothing to do."
    else
        echo "The following commands require sudo:"
        echo ""
        for cmd in "${PENDING_COMMANDS[@]}"; do
            echo "  $cmd"
        done
    fi
    exit 0
fi

if [ "$SOMETHING_INSTALLED" = false ]; then
    echo ""
    echo "All prerequisites already installed — nothing to do."
else
    echo ""
    echo "Bootstrap complete! Next steps:"
    echo "  1. ./.agent/scripts/setup_layers.sh [<layer_name>]     # Set up one layer or all (default)"
    echo "  2. make lint                                     # Install pre-commit hooks and run"
fi
