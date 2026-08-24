#!/bin/bash
# .devcontainer/agent/fix-volume-ownership.sh
# Chown the container's anonymous-volume mountpoints to the dropped-privilege
# target user. Called by agent-entrypoint.sh (as root) before it hands off.
#
# Usage: fix-volume-ownership.sh <target_uid> <target_gid> <workspace_root> [worktree_root]
#
# WHY THIS EXISTS AT ALL — the mechanic that #602 got wrong (#604):
# docker_run_agent.sh adds `-v <path>` anonymous volumes over every layer
# workspace's build/install/log so container builds don't share artifacts with
# the host. Docker initializes such a volume from the IMAGE's content at that
# path — not from the bind-mounted host directory underneath it. The image has
# no workspace in it, so the volume comes up EMPTY and root:root, and the
# `ros` user the entrypoint drops to cannot write into it. The host-side
# `mkdir -p` in docker_run_agent.sh sections 4/4b guards a different hazard
# (#566: docker creating a missing *host* mountpoint root-owned) and does
# nothing for the volume's ownership. THIS chown is the load-bearing half.
#
# Consequence: every anonymous volume the launcher adds needs a matching loop
# here. #602 added the section-4b worktree mounts without one, and dispatched
# agents could no longer build in their own worktree (#604).
# tests/test_entrypoint_chown_coverage.sh enforces the pairing.
#
# It lives in its own script rather than inline in the entrypoint so a
# container-side regression test can exercise it directly, without paying the
# entrypoint's ROS sourcing and rosdep pass.

set -euo pipefail

if [ "$#" -lt 3 ]; then
    echo "Usage: $(basename "$0") <target_uid> <target_gid> <workspace_root> [worktree_root]" >&2
    exit 2
fi

TARGET_UID="$1"
TARGET_GID="$2"
WORKSPACE_ROOT="$3"
WORKTREE_ROOT="${4:-}"

# Chown one build/install/log mountpoint, creating it if docker did not.
fix_ws_dir() {
    local ws_dir="$1" subdir target
    for subdir in build install log; do
        target="$ws_dir/$subdir"
        if [ -d "$target" ]; then
            chown -R "$TARGET_UID:$TARGET_GID" "$target" 2>/dev/null || true
        else
            mkdir -p "$target"
            chown "$TARGET_UID:$TARGET_GID" "$target"
        fi
    done
}

# 1. layers/main/*_ws — matches docker_run_agent.sh section 4.
#    No [ ! -L ] guard here, deliberately: these are the symlink TARGETS, and
#    the launcher shields them unconditionally. The asymmetry with loop 2 is
#    intentional — do not "harmonize" the guards.
for ws_dir in "$WORKSPACE_ROOT"/layers/main/*_ws; do
    [ -d "$ws_dir" ] || continue
    fix_ws_dir "$ws_dir"
done

# 2. The dispatched worktree's own *_ws — matches docker_run_agent.sh section
#    4b, and mirrors its [ -d ] && [ ! -L ] guard. In a layer worktree only the
#    --layer target is a real directory; the sibling *_ws entries are symlinks
#    back into layers/main, already handled by loop 1. [ -d ] follows symlinks
#    and so does NOT filter them — the explicit [ ! -L ] does, keeping this
#    loop from reaching through a symlink into the already-handled tree. In a
#    workspace worktree there are no real *_ws dirs, so this is a clean no-op.
if [ -n "$WORKTREE_ROOT" ] && [ -d "$WORKTREE_ROOT" ]; then
    for ws_dir in "$WORKTREE_ROOT"/*_ws; do
        [ -d "$ws_dir" ] || continue
        [ ! -L "$ws_dir" ] || continue
        fix_ws_dir "$ws_dir"
    done
fi
