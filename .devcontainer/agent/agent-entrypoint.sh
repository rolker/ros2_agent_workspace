#!/bin/bash
# .devcontainer/agent/agent-entrypoint.sh
# Container entrypoint for the sandboxed agent DevContainer.
#
# Responsibilities:
#   1. Fix ownership of anonymous volumes (build/install/log dirs)
#   2. (git identity is intentionally NOT configured here — see step 2)
#   3. Source ROS 2 environment
#   4. Check/install rosdep dependencies (skip when already satisfied)
#   5. Initialize Claude Code config; chown the pre-commit cache volume
#   6. Hand off to CMD (claude --dangerously-skip-permissions ...)

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

# ---------- 2. Git identity: intentionally NOT configured here ----------
# Dispatched agents commit with per-invocation identity literals
# (`git -c user.name=… -c user.email=… commit`, per AGENTS.md § Agent Commit
# Identity), which the dispatch handoff prompt embeds. The agent runs as the
# repo-owning `ros` user (UID matches the host owner), so there is no
# dubious-ownership issue and no persistent `.git/config` identity is needed.
#
# Deliberately NOT setting a default identity: a `git commit` that omits the
# `-c` literals then fails loudly (and leaves the work staged in the
# bind-mounted worktree for the host to recover) rather than committing under
# a silent/generic identity. That loud failure is the guard against
# misattribution, not a bug. (This replaces an earlier root-run
# `configure_git_identity.sh --detect` that tripped git's dubious-ownership
# protection — running as root over host-uid-owned mounts.)

# ---------- 3. Source ROS 2 environment ----------
# Temporarily relax strict mode — ROS 2 setup scripts reference unbound variables
set +u
echo "Sourcing ROS 2 environment..."
cd "$WORKSPACE_ROOT"
# shellcheck disable=SC1091
source "$WORKSPACE_ROOT/.agent/scripts/setup.bash"
set -u

# ---------- 4. Install rosdep dependencies (best-effort, skip when satisfied) ----------
# `rosdep check` is fast and offline. Only pay the apt-get update +
# rosdep install cost for layers that actually report missing deps —
# on a workspace whose deps are already in the base image this skips
# the 30–60s+ refresh entirely. Set FORCE_DEPS_REFRESH=1 to always
# refresh (e.g. after editing a package.xml that adds a dependency).
echo "Checking rosdep dependencies..."
APT_REFRESHED=0
for ws_dir in "$WORKSPACE_ROOT"/layers/main/*_ws; do
    [ -d "$ws_dir/src" ] || continue
    ws_name="$(basename "$ws_dir")"
    if [ "${FORCE_DEPS_REFRESH:-0}" != "1" ] && \
       rosdep check --from-paths "$ws_dir/src" --ignore-src --rosdistro jazzy >/dev/null 2>&1; then
        echo "  $ws_name: dependencies satisfied — skipping install"
        continue
    fi
    # Missing deps (or forced refresh, or rosdep db not yet usable):
    # refresh the apt index once, then install for this layer.
    if [ "$APT_REFRESHED" = "0" ]; then
        echo "  refreshing apt index (deps missing or refresh forced)..."
        apt-get update -qq || echo "  (apt-get update failed — continuing)"
        APT_REFRESHED=1
    fi
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

# Pre-commit cache named volume (docker_run_agent.sh mount #7): docker
# creates the mountpoint root-owned, so chown it to the target user — the
# pre-commit hooks run as that user and must read/write hook environments.
if [ -d "$TARGET_HOME/.cache" ]; then
    chown -R "$TARGET_UID:$TARGET_GID" "$TARGET_HOME/.cache" 2>/dev/null || true
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
echo "  Identity:  per-commit via 'git -c' literals (no persistent config — by design)"
echo "  ROS 2:     ${ROS_DISTRO:-not sourced}"
echo "========================================="
echo ""

# Drop privileges: run CMD as the non-root target user.
# setpriv uses setuid(2) directly (no setuid bits) so it works with
# --security-opt no-new-privileges:true.
#
# setpriv inherits the entrypoint's environment, where HOME is still /root
# (the entrypoint runs as root). Without resetting it, the dropped `ros`
# process would resolve HOME=/root and fail with EACCES trying to write
# under /root/.claude (e.g. Claude Code's per-session env dir). Set HOME
# (and USER/LOGNAME) to the target user so $HOME-relative paths land in
# the user-owned /home/ros.
exec setpriv --reuid="$TARGET_UID" --regid="$TARGET_GID" --init-groups -- \
    env HOME="$TARGET_HOME" USER="$TARGET_USER" LOGNAME="$TARGET_USER" "$@"
