#!/bin/bash
# .devcontainer/agent/agent-entrypoint.sh
# Container entrypoint for the sandboxed agent DevContainer.
#
# Responsibilities:
#   1. Fix ownership of anonymous volumes (build/install/log dirs)
#   2. Configure persistent git identity
#   3. Source ROS 2 environment
#   4. Install rosdep dependencies (best-effort)
#   5. Hand off to CMD (claude --dangerously-skip-permissions)

set -euo pipefail

WORKSPACE_ROOT="${ROS2_AGENT_WORKSPACE_ROOT:-}"
if [ -z "$WORKSPACE_ROOT" ]; then
    echo "ERROR: ROS2_AGENT_WORKSPACE_ROOT not set. Use docker_run_agent.sh to launch." >&2
    exit 1
fi

# ---------- 1. Fix anonymous volume ownership ----------
# Docker creates anonymous volumes as root. Fix ownership so the ros user
# can write to build/install/log directories.
echo "Fixing volume ownership..."
for ws_dir in "$WORKSPACE_ROOT"/layers/main/*_ws; do
    [ -d "$ws_dir" ] || continue
    for subdir in build install log; do
        target="$ws_dir/$subdir"
        if [ -d "$target" ]; then
            sudo chown -R "$(id -u):$(id -g)" "$target" 2>/dev/null || true
        else
            mkdir -p "$target"
        fi
    done
done

# ---------- 2. Configure persistent git identity ----------
# In a container, we use persistent config (writes to .git/config) rather than
# ephemeral env vars, since the container is an isolated environment.
if [ -x "$WORKSPACE_ROOT/.agent/scripts/configure_git_identity.sh" ]; then
    echo "Configuring git identity..."
    cd "$WORKSPACE_ROOT"
    "$WORKSPACE_ROOT/.agent/scripts/configure_git_identity.sh" --detect
fi

# ---------- 3. Source ROS 2 environment ----------
# Temporarily relax strict mode — ROS 2 setup scripts reference unbound variables
set +u
echo "Sourcing ROS 2 environment..."
cd "$WORKSPACE_ROOT"
# shellcheck disable=SC1091
source "$WORKSPACE_ROOT/.agent/scripts/env.sh"
set -u

# ---------- 4. Install rosdep dependencies (best-effort) ----------
echo "Installing rosdep dependencies..."
for ws_dir in "$WORKSPACE_ROOT"/layers/main/*_ws; do
    [ -d "$ws_dir/src" ] || continue
    ws_name="$(basename "$ws_dir")"
    echo "  rosdep install for $ws_name..."
    rosdep install --from-paths "$ws_dir/src" --ignore-src -y --rosdistro jazzy 2>/dev/null || {
        echo "  (some dependencies unavailable for $ws_name — continuing)"
    }
done

# ---------- 5. Initialize Claude Code config ----------
# Copy host config files (mounted at staging paths) into user-owned locations
# so Claude Code skips onboarding and can create its own subdirectories.
#   ~/.claude.json              — onboarding state (hasCompletedOnboarding flag)
#   ~/.claude/settings.json     — user preferences
#   ~/.claude/.credentials.json — Anthropic subscription auth
mkdir -p "$HOME/.claude"
if [ -f /tmp/claude-state.json ]; then
    cp /tmp/claude-state.json "$HOME/.claude.json"
fi
if [ -f /tmp/claude-settings.json ]; then
    cp /tmp/claude-settings.json "$HOME/.claude/settings.json"
fi
if [ -f /tmp/claude-credentials.json ]; then
    cp /tmp/claude-credentials.json "$HOME/.claude/.credentials.json"
    chmod 600 "$HOME/.claude/.credentials.json"
fi

# ---------- 6. Hand off to CMD ----------
# Restore working directory to the worktree (entrypoint cd'd to WORKSPACE_ROOT above)
if [ -n "${WORKTREE_ROOT:-}" ] && [ -d "$WORKTREE_ROOT" ]; then
    cd "$WORKTREE_ROOT"
fi

echo ""
echo "========================================="
echo "  Sandboxed Agent Container Ready"
echo "========================================="
echo "  Workspace: $WORKSPACE_ROOT"
echo "  Identity:  $(git config user.name 2>/dev/null || echo 'not set') <$(git config user.email 2>/dev/null || echo 'not set')>"
echo "  ROS 2:     ${ROS_DISTRO:-not sourced}"
echo "========================================="
echo ""

exec "$@"
