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

# Validate both roots. Neither loop below can tell "nothing to chown here" from
# "I was pointed at the wrong path" — a stale or typo'd root just makes its loop
# iterate over nothing, which is #604's failure mode exactly: volumes left
# root-owned, no diagnostic, and the agent discovering it as an EACCES from
# colcon much later. Fail here instead.
if [ ! -d "$WORKSPACE_ROOT" ]; then
    echo "ERROR: $(basename "$0"): workspace root is not a directory: $WORKSPACE_ROOT" >&2
    exit 1
fi
# An EMPTY worktree root is the legitimate "not a dispatched worktree" case and
# skips loop 2. A non-empty one that isn't a directory is a misconfiguration.
if [ -n "$WORKTREE_ROOT" ] && [ ! -d "$WORKTREE_ROOT" ]; then
    echo "ERROR: $(basename "$0"): worktree root is not a directory: $WORKTREE_ROOT" >&2
    echo "       Its anonymous volumes would be left root-owned (#604)." >&2
    exit 1
fi

# Chown one build/install/log mountpoint, creating it if docker did not.
#
# The recursive chown is non-fatal by design — a populated artifact tree can
# hold entries this pass cannot touch, and aborting there would strand the
# remaining workspaces. But it is NOT silent: a failure here leaves exactly
# #604's signature (root-owned volume, EACCES from colcon much later, no
# diagnostic), so say which path failed and name the usual cause. Under a
# rootless or userns-remapped daemon the container's root is not the volume's
# owner and every chown fails — that is the case this warning exists for.
fix_ws_dir() {
    local ws_dir="$1" subdir target
    for subdir in build install log; do
        target="$ws_dir/$subdir"
        if [ -d "$target" ]; then
            if ! chown -R "$TARGET_UID:$TARGET_GID" "$target" 2>/dev/null; then
                echo "WARNING: $(basename "$0"): chown failed for $target" >&2
                echo "         The agent may hit 'Permission denied' building there (#604)." >&2
                echo "         Usual cause: a rootless / userns-remapped Docker daemon." >&2
            fi
        else
            # Same contract as the recursive branch above: warn, do not abort.
            # This script runs unguarded from the entrypoint under
            # `set -euo pipefail`, so an unguarded failure here is a container
            # that refuses to start with a bare error and no context, and in
            # the `docker exec -u 0` recovery path it strands every workspace
            # after the failing one.
            if ! mkdir -p "$target" 2>/dev/null; then
                echo "WARNING: $(basename "$0"): mkdir failed for $target" >&2
                echo "         The agent may hit 'Permission denied' building there (#604)." >&2
                echo "         Usual cause: a read-only or otherwise unwritable mount." >&2
            elif ! chown "$TARGET_UID:$TARGET_GID" "$target" 2>/dev/null; then
                echo "WARNING: $(basename "$0"): chown failed for $target" >&2
                echo "         The agent may hit 'Permission denied' building there (#604)." >&2
                echo "         Usual cause: a rootless / userns-remapped Docker daemon." >&2
            fi
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
if [ -n "$WORKTREE_ROOT" ]; then   # existence already validated above
    for ws_dir in "$WORKTREE_ROOT"/*_ws; do
        [ -d "$ws_dir" ] || continue
        [ ! -L "$ws_dir" ] || continue
        fix_ws_dir "$ws_dir"
    done
fi
