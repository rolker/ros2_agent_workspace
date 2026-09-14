#!/bin/bash
# .agent/scripts/tests/test_workspace_root.sh
# Tests for .agent/scripts/workspace_root.sh
#
# Hermetic (temp sandboxes, no network, no ROS). Each case builds a throwaway
# workspace root: a temp dir holding `.agent/scripts` as a symlink to the real
# one (bash's `cd` is logical, so the script's own root walk lands on the fake
# root) plus a `configs/` directory — the two markers that make a directory a
# workspace root.
#
# Every failure case also asserts that stdout is EMPTY: a caller must read a
# root or nothing, never an empty string at exit 0 (#609).

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REAL_SCRIPTS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
TEST_PASS=0
TEST_FAIL=0

TMPDIR_ROOT=$(mktemp -d /tmp/test_workspace_root.XXXXXX)
cleanup() { rm -rf "$TMPDIR_ROOT"; }
trap cleanup EXIT

GIT_ID=(-c user.name=Test -c user.email=test@example.invalid)

pass() { echo "✅ PASS: $1"; TEST_PASS=$((TEST_PASS + 1)); }
fail() { echo "❌ FAIL: $1"; TEST_FAIL=$((TEST_FAIL + 1)); }

make_root() {
    local root="$TMPDIR_ROOT/$1"
    mkdir -p "$root/configs" "$root/.agent"
    ln -s "$REAL_SCRIPTS_DIR" "$root/.agent/scripts"
    echo "$root"
}

# The walk-up the skills use verbatim: find any copy of the script above you,
# then let it have the last word.
walk_up_from() {
    (
        cd "$1" || exit 1
        d=$(pwd)
        while [ "$d" != "/" ]; do
            if [ -x "$d/.agent/scripts/workspace_root.sh" ]; then
                "$d/.agent/scripts/workspace_root.sh"
                exit $?
            fi
            d=$(dirname "$d")
        done
        exit 1
    )
}

# --- 1. a plain workspace root resolves to itself ----------------------------
root=$(make_root plain)
out=$("$root/.agent/scripts/workspace_root.sh" 2>"$TMPDIR_ROOT/stderr"); rc=$?
if [ "$rc" -eq 0 ] && [ "$out" = "$root" ]; then
    pass "a plain workspace root resolves to itself"
else
    fail "plain root: rc=$rc out='$out' (expected 0 / '$root'), stderr='$(cat "$TMPDIR_ROOT/stderr")'"
fi

# --- 2. from a WORKSPACE worktree, the MAIN checkout wins --------------------
# `layers/` and the `configs/manifest` symlink exist only in the main checkout,
# and every caller's contract anchors there rather than at the worktree.
root=$(make_root worktree_hop)
touch "$root/configs/test.repos"
git -C "$root" init -q -b main
git -C "$root" add -A >/dev/null 2>&1
git -C "$root" "${GIT_ID[@]}" commit -qm "workspace" >/dev/null 2>&1
wt="$TMPDIR_ROOT/worktree_hop_wt"
if git -C "$root" "${GIT_ID[@]}" worktree add -q -b wt "$wt" >/dev/null 2>&1; then
    out=$("$wt/.agent/scripts/workspace_root.sh" 2>"$TMPDIR_ROOT/stderr"); rc=$?
    if [ "$rc" -eq 0 ] && [ "$out" = "$root" ]; then
        pass "run from a workspace worktree, resolves to the MAIN checkout"
    else
        fail "worktree hop: rc=$rc out='$out' (expected 0 / '$root'), stderr='$(cat "$TMPDIR_ROOT/stderr")'"
    fi

    # --- 3. the LAYER-worktree case: cwd is inside the PROJECT repo ----------
    # This is the case `git --git-common-dir` got wrong: standing in the
    # project repo, it answered with the project repo's own root, where
    # .agent/scripts/ and configs/ do not exist.
    mkdir -p "$wt/demo_ws/src/demo_repo"
    git -C "$wt/demo_ws/src/demo_repo" init -q -b main
    touch "$wt/demo_ws/src/demo_repo/package.xml"
    git -C "$wt/demo_ws/src/demo_repo" add -A >/dev/null 2>&1
    git -C "$wt/demo_ws/src/demo_repo" "${GIT_ID[@]}" commit -qm "project repo" >/dev/null 2>&1
    naive=$( cd "$wt/demo_ws/src/demo_repo" \
             && d=$(git rev-parse --path-format=absolute --git-common-dir 2>/dev/null) \
             && dirname "$d" )
    out=$(walk_up_from "$wt/demo_ws/src/demo_repo"); rc=$?
    if [ "$rc" -eq 0 ] && [ "$out" = "$root" ] && [ "$naive" != "$root" ]; then
        pass "from inside a project repo in a layer worktree, resolves to the workspace (where --git-common-dir did not)"
    else
        fail "layer worktree: rc=$rc out='$out' naive='$naive' (expected 0 / '$root', and naive != it)"
    fi
else
    fail "worktree: could not create the test worktree"
fi

# --- 4. $WORKSPACE_ROOT overrides, but is validated, never taken on faith ----
root=$(make_root override)
other=$(make_root override_target)
out=$(WORKSPACE_ROOT="$other" "$root/.agent/scripts/workspace_root.sh" 2>"$TMPDIR_ROOT/stderr"); rc=$?
if [ "$rc" -eq 0 ] && [ "$out" = "$other" ]; then
    pass "WORKSPACE_ROOT overrides the script's own location"
else
    fail "override: rc=$rc out='$out' (expected 0 / '$other'), stderr='$(cat "$TMPDIR_ROOT/stderr")'"
fi

mkdir -p "$TMPDIR_ROOT/not_a_workspace"
out=$(WORKSPACE_ROOT="$TMPDIR_ROOT/not_a_workspace" \
      "$root/.agent/scripts/workspace_root.sh" 2>"$TMPDIR_ROOT/stderr"); rc=$?
if [ "$rc" -eq 1 ] && [ -z "$out" ] && grep -q "not a workspace root" "$TMPDIR_ROOT/stderr"; then
    pass "a WORKSPACE_ROOT that is not a workspace root → exit 1, empty stdout, with the reason"
else
    fail "bad override: rc=$rc out='$out' (expected 1 / empty), stderr='$(cat "$TMPDIR_ROOT/stderr")'"
fi

# --- 5. a copy of the script outside any workspace fails loudly -------------
# Never "the root is /" or an empty string at exit 0.
mkdir -p "$TMPDIR_ROOT/loose/.agent/scripts"
cp "$REAL_SCRIPTS_DIR/workspace_root.sh" "$TMPDIR_ROOT/loose/.agent/scripts/"
out=$("$TMPDIR_ROOT/loose/.agent/scripts/workspace_root.sh" 2>"$TMPDIR_ROOT/stderr"); rc=$?
if [ "$rc" -eq 1 ] && [ -z "$out" ] && grep -q "not in a workspace" "$TMPDIR_ROOT/stderr"; then
    pass "a copy of the script outside a workspace → exit 1, empty stdout, with the reason"
else
    fail "loose copy: rc=$rc out='$out' (expected 1 / empty), stderr='$(cat "$TMPDIR_ROOT/stderr")'"
fi

# --- 6. sourced rather than executed -----------------------------------------
out=$(bash -c "source '$REAL_SCRIPTS_DIR/workspace_root.sh'" 2>"$TMPDIR_ROOT/stderr"); rc=$?
if [ "$rc" -eq 2 ] && [ -z "$out" ] && grep -q "must be executed" "$TMPDIR_ROOT/stderr"; then
    pass "workspace_root.sh sourced → exit 2 with a reason, never a silent 0"
else
    fail "source guard: rc=$rc out='$out' (expected 2 / empty), stderr='$(cat "$TMPDIR_ROOT/stderr")'"
fi

echo ""
echo "Passed: $TEST_PASS  Failed: $TEST_FAIL"
[ "$TEST_FAIL" -eq 0 ]
