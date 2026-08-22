#!/bin/bash
# .agent/scripts/tests/test_docker_run_mount_args.sh
# Regression test for #602: docker_run_agent.sh must punch anonymous-volume
# "holes" for build/install/log in the DISPATCHED WORKTREE's *_ws layer
# workspaces (section 4b), not just layers/main/*_ws (section 4). Without the
# shield, container and host share worktree build artifacts at the same
# absolute path and mix objects compiled against different ROS package sets.
#
# The test drives the Docker-free `--print-mounts` dry run against a fabricated
# temp tree injected via the ROOT_DIR env hook, and asserts:
#   - a REAL *_ws dir in the worktree gets -v mounts for build/install/log,
#   - a SYMLINKED *_ws dir in the worktree (a sibling pointing into
#     layers/main) is NOT re-shielded by section 4b — [ ! -L ] filters it,
#   - the symlink's real target under layers/main is still shielded exactly
#     once by section 4 (no duplication),
#   - a workspace worktree (no real *_ws dirs) is a clean no-op.
#
# No Docker daemon and no credentials required (--print-mounts short-circuits
# before both). Run: bash .agent/scripts/tests/test_docker_run_mount_args.sh

set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DRA="$SCRIPT_DIR/../docker_run_agent.sh"

TEST_PASS=0
TEST_FAIL=0
pass() { echo "PASS: $1"; TEST_PASS=$((TEST_PASS + 1)); }
fail() { echo "FAIL: $1"; TEST_FAIL=$((TEST_FAIL + 1)); }

# ---------- Fixtures ----------
# A fabricated workspace root. ROOT_DIR is injected via the env hook so the
# script's worktree resolution + shield loops operate on this tree, not the
# real repo.
ROOT="$(mktemp -d /tmp/dra_mounts.XXXXXX)"
cleanup() { rm -rf "$ROOT"; }
trap cleanup EXIT

NL=999602          # layer-worktree issue number
NW=999603          # workspace-worktree issue number
SLUG="mounttest"

# layers/main real workspace (the symlink target; shielded by section 4).
mkdir -p "$ROOT/layers/main/nav_ws"

# Layer worktree: one REAL *_ws (ui_ws) + one SYMLINKED *_ws (nav_ws ->
# layers/main/nav_ws), mirroring how worktree_create.sh lays out a layer
# worktree (only the --layer target is real; siblings are symlinks).
LWT="$ROOT/layers/worktrees/issue-$SLUG-$NL"
mkdir -p "$LWT/ui_ws"
ln -s "$ROOT/layers/main/nav_ws" "$LWT/nav_ws"

# Workspace worktree: no real *_ws dirs -> section 4b must be a no-op.
WWT="$ROOT/.workspace-worktrees/issue-workspace-$NW"
mkdir -p "$WWT/.agent"

# ---------- Run 1: layer worktree ----------
lout=$(ROOT_DIR="$ROOT" "$DRA" --issue "$NL" --repo-slug "$SLUG" --print-mounts 2>&1); lrc=$?

if [ "$lrc" -ne 0 ]; then
    fail "layer-worktree --print-mounts exits 0 (rc=$lrc; out=$lout)"
else
    pass "layer-worktree --print-mounts exits 0"
fi

# Real ui_ws gets all three anonymous-volume mounts.
missing=""
for sub in build install log; do
    printf '%s\n' "$lout" | grep -qxF -- "$LWT/ui_ws/$sub" || missing="$missing $sub"
done
if [ -z "$missing" ]; then
    pass "real worktree ui_ws is shielded (build/install/log mounted)"
else
    fail "real worktree ui_ws missing mounts:$missing (out=$lout)"
fi

# Symlinked nav_ws in the worktree must NOT be re-shielded by section 4b.
if printf '%s\n' "$lout" | grep -qF -- "$LWT/nav_ws/"; then
    fail "symlinked worktree nav_ws was shielded (should be filtered by [ ! -L ])"
else
    pass "symlinked worktree nav_ws is skipped ([ ! -L ] guard works)"
fi

# The symlink's real target under layers/main is shielded exactly once
# (section 4), not duplicated by section 4b reaching through the symlink.
count=$(printf '%s\n' "$lout" | grep -cxF -- "$ROOT/layers/main/nav_ws/build")
if [ "$count" -eq 1 ]; then
    pass "layers/main nav_ws/build shielded exactly once (no symlink duplication)"
else
    fail "layers/main nav_ws/build appears $count times (want 1; out=$lout)"
fi

# ---------- Run 2: workspace worktree (no real *_ws -> no-op) ----------
wout=$(ROOT_DIR="$ROOT" "$DRA" --issue "$NW" --print-mounts 2>&1); wrc=$?

if [ "$wrc" -ne 0 ]; then
    fail "workspace-worktree --print-mounts exits 0 (rc=$wrc; out=$wout)"
else
    pass "workspace-worktree --print-mounts exits 0"
fi

# No mount should reference a path *under* the workspace worktree: its .agent
# subdir is not a *_ws, and there are no real *_ws dirs to shield. (The
# "Using worktree:" status line names $WWT followed by a space, not a slash,
# so grepping for "$WWT/" won't false-positive on it.)
if printf '%s\n' "$wout" | grep -qF -- "$WWT/"; then
    fail "workspace worktree unexpectedly produced shield mounts (out=$wout)"
else
    pass "workspace worktree is a clean no-op (no *_ws shields)"
fi

echo
echo "docker_run mount-arg tests: $TEST_PASS passed, $TEST_FAIL failed"
[ "$TEST_FAIL" -eq 0 ]
