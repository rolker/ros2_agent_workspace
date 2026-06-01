#!/bin/bash
# .agent/scripts/tests/test_worktree_remove.sh
# Tests for worktree_remove.sh — ROOT resolution (regression for #507)
#
# The bug: worktree_remove.sh derived ROOT from its own on-disk location
# (`dirname dirname SCRIPT_DIR`). When invoked through a worktree's *own* copy of
# .agent/scripts — e.g. merge_pr.sh calling "$SCRIPT_DIR/worktree_remove.sh" from
# inside a worktree — ROOT pointed at the worktree instead of the main tree, so the
# worktree dirs it manages (.workspace-worktrees/, layers/worktrees/) were looked
# for under the worktree and removal failed with "No worktree found".
#
# Note: Not using set -e because we want to continue after test failures.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CREATE_SCRIPT="$SCRIPT_DIR/../worktree_create.sh"
REMOVE_SCRIPT="$SCRIPT_DIR/../worktree_remove.sh"
HELPERS="$SCRIPT_DIR/../_worktree_helpers.sh"
TEST_PASS=0
TEST_FAIL=0

# Build a mock workspace (bare origin + clone) with the create/remove scripts and
# helpers copied into .agent/scripts, then create one workspace worktree for issue
# $1. Leaves cwd at WORKSPACE_DIR. Sets WT_DIR to the created worktree path.
setup_with_worktree() {
    local issue="$1"
    TEST_DIR=$(mktemp -d)
    ORIGIN_DIR="$TEST_DIR/origin.git"
    WORKSPACE_DIR="$TEST_DIR/workspace"

    git init -q --bare "$ORIGIN_DIR"
    git clone -q "$ORIGIN_DIR" "$WORKSPACE_DIR"
    cd "$WORKSPACE_DIR" || return 1
    git config user.email "test@example.com"
    git config user.name "Test User"
    git config commit.gpgsign false
    echo "init" > README.md
    git add README.md
    git commit -q -m "Initial commit"
    git push -q origin HEAD 2>/dev/null

    mkdir -p .workspace-worktrees .agent/scripts configs/manifest
    echo "core" > configs/manifest/layers.txt
    cp "$CREATE_SCRIPT" .agent/scripts/worktree_create.sh
    cp "$REMOVE_SCRIPT" .agent/scripts/worktree_remove.sh
    cp "$HELPERS"       .agent/scripts/_worktree_helpers.sh
    chmod +x .agent/scripts/worktree_create.sh .agent/scripts/worktree_remove.sh

    # Slug auto-detects to "origin" (basename of the bare origin path).
    .agent/scripts/worktree_create.sh --issue "$issue" --type workspace >/dev/null 2>&1
    WT_DIR="$WORKSPACE_DIR/.workspace-worktrees/issue-origin-$issue"
    [ -d "$WT_DIR" ]
}

cleanup() {
    if [ -n "$TEST_DIR" ] && [ -d "$TEST_DIR" ]; then
        cd "$WORKSPACE_DIR" 2>/dev/null && git worktree prune 2>/dev/null
        cd /
        rm -rf "$TEST_DIR"
    fi
}

run_test() {
    local test_name="$1" test_func="$2"
    echo "Test: $test_name"
    if $test_func; then
        echo "  PASS: $test_name"; ((TEST_PASS++))
    else
        echo "  FAIL: $test_name"; ((TEST_FAIL++))
    fi
    echo ""
}

echo "=== Testing worktree_remove.sh ROOT resolution (#507) ==="
echo ""

# Regression: invoke the WORKTREE's own copy of worktree_remove.sh (SCRIPT_DIR is
# inside the worktree), from the main tree. ROOT must resolve to the main tree so
# the worktree is found and removed — not to the worktree itself.
test_remove_via_worktree_copy_resolves_main_root() {
    setup_with_worktree 12321 || { echo "    setup failed"; cleanup; return 1; }

    # Place a copy of the scripts inside the worktree (a real workspace worktree
    # has .agent/scripts checked out; the mock's are untracked, so copy them in).
    mkdir -p "$WT_DIR/.agent/scripts"
    cp "$REMOVE_SCRIPT" "$WT_DIR/.agent/scripts/worktree_remove.sh"
    cp "$HELPERS"       "$WT_DIR/.agent/scripts/_worktree_helpers.sh"
    chmod +x "$WT_DIR/.agent/scripts/worktree_remove.sh"

    # Invoke from the main tree (CALLER_PWD must not be inside the target worktree).
    # --force: the mock worktree has an untracked `layers/` symlink (no .gitignore),
    # which is irrelevant to ROOT resolution. The old bug still prints "No worktree
    # found" even with --force (ROOT mis-resolves, so the dir is never located).
    local output rc
    output=$(cd "$WORKSPACE_DIR" && \
        "$WT_DIR/.agent/scripts/worktree_remove.sh" --issue 12321 --repo-slug origin --force 2>&1)
    rc=$?

    if [[ "$output" == *"No worktree found"* ]]; then
        echo "    ROOT mis-resolved to the worktree (the #507 bug): $output"
        cleanup; return 1
    fi
    if [ $rc -ne 0 ]; then
        echo "    removal exited non-zero (rc=$rc): $output"
        cleanup; return 1
    fi
    if [ -d "$WT_DIR" ]; then
        echo "    worktree dir still present after removal: $WT_DIR"
        cleanup; return 1
    fi
    cleanup; return 0
}
run_test "remove via worktree's own script copy resolves ROOT to main tree (#507)" \
    test_remove_via_worktree_copy_resolves_main_root

# Sanity: the normal main-tree invocation still removes correctly (the new
# git-based resolution must not regress the common path).
test_remove_via_main_tree_copy_still_works() {
    setup_with_worktree 12322 || { echo "    setup failed"; cleanup; return 1; }

    local output rc
    output=$(cd "$WORKSPACE_DIR" && \
        .agent/scripts/worktree_remove.sh --issue 12322 --repo-slug origin --force 2>&1)
    rc=$?

    if [ $rc -ne 0 ] || [ -d "$WT_DIR" ]; then
        echo "    main-tree removal failed (rc=$rc): $output"
        cleanup; return 1
    fi
    cleanup; return 0
}
run_test "remove via main-tree script copy still works" \
    test_remove_via_main_tree_copy_still_works

echo "========================================"
echo "TEST RESULTS"
echo "========================================"
echo "Passed: $TEST_PASS"
echo "Failed: $TEST_FAIL"
echo "========================================"

if [ $TEST_FAIL -eq 0 ]; then
    echo "All tests passed!"
    exit 0
else
    echo "Some tests failed"
    exit 1
fi
