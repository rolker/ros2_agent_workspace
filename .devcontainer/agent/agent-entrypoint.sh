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

# Resolve the target non-root user (matches Dockerfile ARG USERNAME)
TARGET_USER="ros"
TARGET_UID="$(id -u "$TARGET_USER")"
TARGET_GID="$(id -g "$TARGET_USER")"

# ---------- 1. Fix anonymous volume ownership ----------
# Docker creates anonymous volumes as root. Entrypoint runs as root so we
# can fix ownership directly — no sudo needed.
echo "Fixing volume ownership..."
for ws_dir in "$WORKSPACE_ROOT"/layers/main/*_ws; do
    [ -d "$ws_dir" ] || continue
    for subdir in build install log; do
        target="$ws_dir/$subdir"
        if [ -d "$target" ]; then
            chown -R "$TARGET_UID:$TARGET_GID" "$target" 2>/dev/null || true
        else
            mkdir -p "$target"
            chown "$TARGET_UID:$TARGET_GID" "$target"
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
TARGET_HOME="$(eval echo "~$TARGET_USER")"
mkdir -p "$TARGET_HOME/.claude"
if [ -f /tmp/claude-state.json ]; then
    cp /tmp/claude-state.json "$TARGET_HOME/.claude.json"
fi
if [ -f /tmp/claude-settings.json ]; then
    cp /tmp/claude-settings.json "$TARGET_HOME/.claude/settings.json"
fi
if [ -f /tmp/claude-credentials.json ]; then
    cp /tmp/claude-credentials.json "$TARGET_HOME/.claude/.credentials.json"
    chmod 600 "$TARGET_HOME/.claude/.credentials.json"
fi
chown -R "$TARGET_UID:$TARGET_GID" "$TARGET_HOME/.claude" "$TARGET_HOME/.claude.json" 2>/dev/null || true

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

# Drop privileges: run CMD as the non-root target user.
# setpriv uses setuid(2) directly (no setuid bits) so it works with
# --security-opt no-new-privileges:true.
exec setpriv --reuid="$TARGET_UID" --regid="$TARGET_GID" --init-groups -- "$@"
