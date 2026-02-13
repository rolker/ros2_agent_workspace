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
# Main builds live in layers/main/, worktree builds in layers/worktrees/issue-N/
LAYERS_BASE="$ROOT_DIR/layers/main"
WORKTREE_CONTEXT=""

# Check if we're in a layer worktree (layers/worktrees/issue-N/)
if [[ "$ROOT_DIR" == *"/layers/worktrees/"* ]]; then
    # We're in a layer worktree - this directory IS the layers base
    # It contains the working layer + symlinks to main for others
    WORKTREE_CONTEXT="layer"
    LAYERS_BASE="$ROOT_DIR"
    echo "  ℹ Worktree detected: layer worktree"
elif [[ "$ROOT_DIR" == *"/.workspace-worktrees/"* ]]; then
    # We're in a workspace worktree - uses symlinked layers/main
    WORKTREE_CONTEXT="workspace"
    echo "  ℹ Worktree detected: workspace worktree"
fi

# Export worktree info for other scripts
export ROS2_WORKSPACE_ROOT="$ROOT_DIR"
export ROS2_LAYERS_BASE="$LAYERS_BASE"
[ -n "$WORKTREE_CONTEXT" ] && export ROS2_WORKTREE_TYPE="$WORKTREE_CONTEXT"

# 2. Workspace Layers
# Define the order of layers to source. Order determines overlay priority (last one is top).
# Primary: use the configs/manifest symlink (created by setup.sh)
LAYERS_CONFIG="$ROOT_DIR/configs/manifest/layers.txt"

# For worktrees, the configs/manifest symlink may not exist (gitignored).
# Derive the main workspace root and check there.
if [ ! -f "$LAYERS_CONFIG" ] && [ -n "$WORKTREE_CONTEXT" ]; then
    if [ "$WORKTREE_CONTEXT" = "workspace" ]; then
        # Workspace worktrees: .workspace-worktrees/issue-N -> main root is two levels up
        MAIN_ROOT="$(dirname "$(dirname "$ROOT_DIR")")"
    else
        # Layer worktrees: layers/worktrees/issue-N -> main root is three levels up
        MAIN_ROOT="$(dirname "$(dirname "$(dirname "$ROOT_DIR")")")"
    fi
    LAYERS_CONFIG="$MAIN_ROOT/configs/manifest/layers.txt"
fi

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
        # shellcheck disable=SC1090
        source "$SETUP_FILE"
    else
        if [ -d "$LAYERS_BASE/${layer}_ws/src" ]; then
             echo "  ! Warning: $layer exists but is not built (setup.bash not found)."
        fi
    fi
done

# 3. Guard against branch switching with git checkout.
# Agents must use worktrees instead. This wrapper blocks `git checkout <branch>`
# while still allowing file-restore forms like `git checkout -- <file>`.
git() {
    if [[ "${1:-}" == "checkout" ]]; then
        # Allow restore forms: git checkout -- <file>, git checkout -p, git checkout .
        if [[ "${2:-}" == "--" || "${2:-}" == "-p" ]]; then
            command git "$@"
            return $?
        fi
        echo "❌ 'git checkout' for branch switching is disabled in this workspace."
        echo "   Use worktrees instead:"
        echo "     .agent/scripts/worktree_create.sh --issue <N> --type workspace"
        echo "     source .agent/scripts/worktree_enter.sh --issue <N>"
        echo ""
        echo "   To restore files, use: git checkout -- <file>"
        return 1
    fi
    command git "$@"
}

# 4. Expose dev tools from .venv without altering the Python environment.
# A shell function avoids prepending .venv/bin to PATH, which would shadow
# the system python3 that ROS 2 depends on.
# .venv lives in the main workspace root (gitignored), so resolve it for worktrees.
_VENV_ROOT="$ROOT_DIR"
if [ ! -d "$_VENV_ROOT/.venv" ] && [ -n "$WORKTREE_CONTEXT" ]; then
    if [ "$WORKTREE_CONTEXT" = "workspace" ]; then
        _VENV_ROOT="$(dirname "$(dirname "$ROOT_DIR")")"
    else
        _VENV_ROOT="$(dirname "$(dirname "$(dirname "$ROOT_DIR")")")"
    fi
fi
if [ -x "$_VENV_ROOT/.venv/bin/pre-commit" ]; then
    # Capture the resolved path so the function works regardless of cwd
    _PRECOMMIT_BIN="$_VENV_ROOT/.venv/bin/pre-commit"
    pre-commit() { "$_PRECOMMIT_BIN" "$@"; }
fi
unset _VENV_ROOT

# 5. Prevent interactive editor hangs (safe for both agents and humans)
export GIT_EDITOR=true

# 6. Auto-configure git identity for agents
if [ -z "${GIT_AUTHOR_NAME:-}" ]; then
    # No identity set yet — try to detect and configure
    if source "$SCRIPT_DIR/detect_cli_env.sh" 2>/dev/null && [ "$AGENT_FRAMEWORK" != "unknown" ]; then
        # Framework detected — configure identity via --detect
        source "$SCRIPT_DIR/set_git_identity_env.sh" --detect
    else
        echo "  ! Warning: Could not auto-detect agent framework. Git identity not configured."
        echo "    Run: source .agent/scripts/set_git_identity_env.sh --agent <framework>"
    fi
else
    echo "  Git identity: $GIT_AUTHOR_NAME <${GIT_AUTHOR_EMAIL:-not set}>"
fi

echo "Environment ready."
