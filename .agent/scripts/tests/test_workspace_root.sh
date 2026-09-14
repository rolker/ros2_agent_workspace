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
#
# Hermetic includes the AMBIENT ENVIRONMENT, not just the filesystem: an
# inherited $WORKSPACE_ROOT is read by the script under test and by the walk-up
# snippet, and every case that does not set it deliberately would otherwise
# resolve to the operator's own workspace and fail — in exactly the environment
# the three skills now tell operators to create. It is unset here once; the
# cases that exercise it set it per invocation.

set -uo pipefail

unset WORKSPACE_ROOT

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

# The walk-up the skills use verbatim: start at $WORKSPACE_ROOT when it is set
# (which is what makes "or set WORKSPACE_ROOT" a remedy from a directory with
# no workspace above it), find the first copy of the script at or above there,
# and let it have the last word. `[ -f ]` + `bash`, not `[ -x ]`: a lost exec
# bit is not a reason to walk past the workspace. $WORKSPACE_ROOT is normalised
# to an ABSOLUTE path first: `dirname .` is `.`, so a relative value would make
# this loop spin forever (case 11).
walk_up_from() {
    (
        cd "$1" || exit 1
        d=$(cd "${WORKSPACE_ROOT:-$(pwd)}" 2>/dev/null && pwd) || d=$(pwd)
        while [ "$d" != "/" ]; do
            if [ -f "$d/.agent/scripts/workspace_root.sh" ]; then
                bash "$d/.agent/scripts/workspace_root.sh"
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

# --- 7. the "or set WORKSPACE_ROOT" remedy works where it is printed --------
# The walk-up is what locates the script, and $WORKSPACE_ROOT is only read
# INSIDE it — so from a directory with no workspace above it at all, a walk
# that started at $(pwd) could never reach a copy of the script to read the
# variable, and the remedy the three skills print there was inert. The walk
# starts at $WORKSPACE_ROOT when set; these two cases are the remedy and its
# absence, from the same directory.
root=$(make_root remedy)
mkdir -p "$TMPDIR_ROOT/nowhere/deep"
out=$(WORKSPACE_ROOT="$root" walk_up_from "$TMPDIR_ROOT/nowhere/deep" 2>"$TMPDIR_ROOT/stderr"); rc=$?
if [ "$rc" -eq 0 ] && [ "$out" = "$root" ]; then
    pass "with no workspace above cwd, setting WORKSPACE_ROOT resolves — the printed remedy works"
else
    fail "remedy: rc=$rc out='$out' (expected 0 / '$root'), stderr='$(cat "$TMPDIR_ROOT/stderr")'"
fi

out=$(walk_up_from "$TMPDIR_ROOT/nowhere/deep" 2>"$TMPDIR_ROOT/stderr"); rc=$?
if [ "$rc" -ne 0 ] && [ -z "$out" ]; then
    pass "with no workspace above cwd and WORKSPACE_ROOT unset → non-zero, empty stdout"
else
    fail "no root: rc=$rc out='$out' (expected non-zero / empty)"
fi

# A WORKSPACE_ROOT that is not a workspace is not taken on faith by the walk
# either: the script it reaches (or does not) must refuse, never answer with
# whatever it walked up to.
out=$(WORKSPACE_ROOT="$TMPDIR_ROOT/not_a_workspace" \
      walk_up_from "$TMPDIR_ROOT/nowhere/deep" 2>"$TMPDIR_ROOT/stderr"); rc=$?
if [ "$rc" -ne 0 ] && [ -z "$out" ]; then
    pass "a WORKSPACE_ROOT that is not a workspace fails the walk too, with empty stdout"
else
    fail "bad remedy: rc=$rc out='$out' (expected non-zero / empty)"
fi

# --- 11. a RELATIVE $WORKSPACE_ROOT terminates instead of hanging -----------
# `dirname .` is `.`, so a walk that started at a relative $WORKSPACE_ROOT
# would spin forever — a silent hang on exactly the remedy the three skills
# print. The walk normalises to an absolute path first. Both halves run under
# `timeout`, so a regression FAILS (rc 124) rather than wedging the suite.
root=$(make_root relative)
out=$(WORKSPACE_ROOT="relative" timeout 20 bash -c \
      "$(declare -f walk_up_from); walk_up_from \"$TMPDIR_ROOT\"" \
      2>"$TMPDIR_ROOT/stderr"); rc=$?
if [ "$rc" -eq 0 ] && [ "$out" = "$root" ]; then
    pass "a relative WORKSPACE_ROOT is normalised and resolves (no hang)"
else
    fail "relative root: rc=$rc out='$out' (expected 0 / '$root'; rc 124 = the walk hung), stderr='$(cat "$TMPDIR_ROOT/stderr")'"
fi

# The same, where the relative value names no workspace at all: it must reach
# the "no root" answer, not loop.
mkdir -p "$TMPDIR_ROOT/nowhere/deep"
out=$(WORKSPACE_ROOT="." timeout 20 bash -c \
      "$(declare -f walk_up_from); walk_up_from \"$TMPDIR_ROOT/nowhere/deep\"" \
      2>"$TMPDIR_ROOT/stderr"); rc=$?
if [ "$rc" -ne 0 ] && [ "$rc" -ne 124 ] && [ -z "$out" ]; then
    pass "a relative WORKSPACE_ROOT naming no workspace terminates non-zero, empty stdout"
else
    fail "relative no-root: rc=$rc out='$out' (expected non-zero and not 124 / empty)"
fi

echo ""
echo "Passed: $TEST_PASS  Failed: $TEST_FAIL"
[ "$TEST_FAIL" -eq 0 ]
