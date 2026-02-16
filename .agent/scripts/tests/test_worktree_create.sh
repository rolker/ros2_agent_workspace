#!/bin/bash
# .agent/scripts/tests/test_worktree_create.sh
# Tests for worktree_create.sh — remote branch tracking logic
#
# Note: Not using set -e because we want to continue after test failures

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CREATE_SCRIPT="$SCRIPT_DIR/../worktree_create.sh"
TEST_PASS=0
TEST_FAIL=0

# Setup a mock workspace with bare origin for workspace-type worktree tests
setup_mock_workspace() {
    TEST_DIR=$(mktemp -d)
    ORIGIN_DIR="$TEST_DIR/origin.git"
    WORKSPACE_DIR="$TEST_DIR/workspace"

    # Create bare origin
    git init -q --bare "$ORIGIN_DIR"

    # Clone into workspace
    git clone -q "$ORIGIN_DIR" "$WORKSPACE_DIR"
    cd "$WORKSPACE_DIR" || return 1
    git config user.email "test@example.com"
    git config user.name "Test User"
    git config commit.gpgsign false

    # Initial commit (bare repos need at least one)
    echo "init" > README.md
    git add README.md
    git commit -q -m "Initial commit"
    git push -q origin HEAD 2>/dev/null

    # Create required directory structure
    mkdir -p .workspace-worktrees
    mkdir -p .agent/scripts
    # Copy the script under test into the mock workspace
    cp "$CREATE_SCRIPT" .agent/scripts/worktree_create.sh
    chmod +x .agent/scripts/worktree_create.sh
}

# Cleanup test repository
cleanup_mock_workspace() {
    if [ -n "$TEST_DIR" ] && [ -d "$TEST_DIR" ]; then
        # Clean up any git worktrees first
        cd "$WORKSPACE_DIR" 2>/dev/null && git worktree prune 2>/dev/null
        cd /
        rm -rf "$TEST_DIR"
    fi
}

# Test helper
run_test() {
    local test_name="$1"
    local test_func="$2"

    echo "Test: $test_name"

    if $test_func; then
        echo "  PASS: $test_name"
        ((TEST_PASS++))
    else
        echo "  FAIL: $test_name"
        ((TEST_FAIL++))
    fi
    echo ""
}

echo "=== Testing worktree_create.sh ==="
echo ""

# Test 1: Remote branch exists, no local → tracks remote
test_workspace_tracks_remote_branch() {
    setup_mock_workspace
    cd "$WORKSPACE_DIR" || return 1

    # Create a branch, push it, delete locally
    git checkout -q -b feature/issue-999
    echo "remote work" > remote.txt
    git add remote.txt
    git commit -q -m "Remote work"
    git push -q origin feature/issue-999
    git checkout -q main 2>/dev/null || git checkout -q master
    git branch -D feature/issue-999 2>/dev/null

    local output
    output=$(.agent/scripts/worktree_create.sh --issue 999 --type workspace 2>&1) || true

    cleanup_mock_workspace

    [[ "$output" == *"Tracking remote branch"* ]]
}
run_test "Workspace tracks remote branch when no local exists" test_workspace_tracks_remote_branch

# Test 2: Local branch exists → uses it
test_workspace_uses_local_branch() {
    setup_mock_workspace
    cd "$WORKSPACE_DIR" || return 1

    # Create a local branch (don't push)
    git checkout -q -b feature/issue-888
    echo "local work" > local.txt
    git add local.txt
    git commit -q -m "Local work"
    git checkout -q main 2>/dev/null || git checkout -q master

    local output
    output=$(.agent/scripts/worktree_create.sh --issue 888 --type workspace 2>&1) || true

    cleanup_mock_workspace

    [[ "$output" == *"Using existing local branch"* ]]
}
run_test "Workspace uses existing local branch" test_workspace_uses_local_branch

# Test 3: No branch anywhere → creates new
test_workspace_creates_new_branch() {
    setup_mock_workspace
    cd "$WORKSPACE_DIR" || return 1

    local output
    output=$(.agent/scripts/worktree_create.sh --issue 777 --type workspace 2>&1) || true

    cleanup_mock_workspace

    [[ "$output" == *"Creating new branch"* ]]
}
run_test "Workspace creates new branch when none exists" test_workspace_creates_new_branch

# ===== Layer worktree tests =====

# Setup a mock workspace with a package repo for layer-type worktree tests
setup_mock_layer_workspace() {
    TEST_DIR=$(mktemp -d)
    PKG_ORIGIN_DIR="$TEST_DIR/pkg_origin.git"
    WORKSPACE_DIR="$TEST_DIR/workspace"

    # Create bare origin for the package repo
    git init -q --bare "$PKG_ORIGIN_DIR"

    # Clone into a temp location, make initial commit, push
    local tmp_clone="$TEST_DIR/tmp_clone"
    git clone -q "$PKG_ORIGIN_DIR" "$tmp_clone"
    cd "$tmp_clone" || return 1
    git config user.email "test@example.com"
    git config user.name "Test User"
    git config commit.gpgsign false
    echo "init" > README.md
    git add README.md
    git commit -q -m "Initial commit"
    git push -q origin HEAD 2>/dev/null
    cd /

    # Set up workspace structure
    mkdir -p "$WORKSPACE_DIR"
    cd "$WORKSPACE_DIR" || return 1
    git init -q .
    git config user.email "test@example.com"
    git config user.name "Test User"
    git config commit.gpgsign false
    echo "workspace" > README.md
    git add README.md
    git commit -q -m "Workspace init"

    # Create layer structure with the package as a clone of pkg_origin
    mkdir -p "$WORKSPACE_DIR/layers/main/core_ws/src"
    git clone -q "$PKG_ORIGIN_DIR" "$WORKSPACE_DIR/layers/main/core_ws/src/test_pkg"
    cd "$WORKSPACE_DIR/layers/main/core_ws/src/test_pkg" || return 1
    git config user.email "test@example.com"
    git config user.name "Test User"
    git config commit.gpgsign false
    cd "$WORKSPACE_DIR" || return 1

    # Create required directories
    mkdir -p layers/worktrees
    mkdir -p .agent/scripts
    # Copy the script under test into the mock workspace
    cp "$CREATE_SCRIPT" .agent/scripts/worktree_create.sh
    chmod +x .agent/scripts/worktree_create.sh

    rm -rf "$tmp_clone"
}

# Cleanup layer test repository
cleanup_mock_layer_workspace() {
    if [ -n "$TEST_DIR" ] && [ -d "$TEST_DIR" ]; then
        # Prune worktrees in both workspace and package repos
        cd "$WORKSPACE_DIR" 2>/dev/null && git worktree prune 2>/dev/null
        local pkg_dir="$WORKSPACE_DIR/layers/main/core_ws/src/test_pkg"
        cd "$pkg_dir" 2>/dev/null && git worktree prune 2>/dev/null
        cd /
        rm -rf "$TEST_DIR"
    fi
}

# Test 4: Layer — remote branch exists, no local → tracks remote
test_layer_tracks_remote_branch() {
    setup_mock_layer_workspace
    local pkg_dir="$WORKSPACE_DIR/layers/main/core_ws/src/test_pkg"
    cd "$pkg_dir" || return 1

    # Create branch, push, then delete both local and remote-tracking ref
    # so git can't auto-resolve — only the bare origin has the branch
    git checkout -q -b feature/issue-999
    echo "remote work" > remote.txt
    git add remote.txt
    git commit -q -m "Remote work"
    git push -q origin feature/issue-999
    git checkout -q main 2>/dev/null || git checkout -q master
    git branch -D feature/issue-999 2>/dev/null
    git branch -rd origin/feature/issue-999 2>/dev/null

    cd "$WORKSPACE_DIR" || return 1
    local output
    output=$(.agent/scripts/worktree_create.sh --issue 999 --type layer --layer core --packages test_pkg 2>&1) || true

    cleanup_mock_layer_workspace

    [[ "$output" == *"tracking remote branch"* ]]
}
run_test "Layer tracks remote branch when no local exists" test_layer_tracks_remote_branch

# Test 5: Layer — local branch exists → uses it
test_layer_uses_local_branch() {
    setup_mock_layer_workspace
    local pkg_dir="$WORKSPACE_DIR/layers/main/core_ws/src/test_pkg"
    cd "$pkg_dir" || return 1

    # Create local branch (don't push)
    git checkout -q -b feature/issue-888
    echo "local work" > local.txt
    git add local.txt
    git commit -q -m "Local work"
    git checkout -q main 2>/dev/null || git checkout -q master

    cd "$WORKSPACE_DIR" || return 1
    local output
    output=$(.agent/scripts/worktree_create.sh --issue 888 --type layer --layer core --packages test_pkg 2>&1) || true

    cleanup_mock_layer_workspace

    [[ "$output" == *"existing local branch"* ]]
}
run_test "Layer uses existing local branch" test_layer_uses_local_branch

# Test 6: Layer — no branch anywhere → creates new
test_layer_creates_new_branch() {
    setup_mock_layer_workspace
    cd "$WORKSPACE_DIR" || return 1

    local output
    output=$(.agent/scripts/worktree_create.sh --issue 777 --type layer --layer core --packages test_pkg 2>&1) || true

    cleanup_mock_layer_workspace

    [[ "$output" == *"new branch"* ]]
}
run_test "Layer creates new branch when none exists" test_layer_creates_new_branch

# ===== --draft-pr tests =====

# Test 7: --draft-pr creates work plan for workspace (gh unavailable is non-fatal)
test_draft_pr_workspace_creates_plan() {
    setup_mock_workspace
    cd "$WORKSPACE_DIR" || return 1

    # Copy template into mock workspace and commit it so it appears in worktrees
    mkdir -p .agent/templates
    cat > .agent/templates/ISSUE_PLAN.md << 'TPLEOF'
# Work Plan: Issue #{ISSUE_NUMBER} - {ISSUE_TITLE}
**Assignee**: {AGENT_NAME}
**Started**: {START_DATE}
TPLEOF
    git add .agent/templates/ISSUE_PLAN.md
    git commit -q -m "Add plan template"

    # Hide gh so the script can't create a real PR (non-fatal path)
    # Auto-detected slug will be "origin" (basename of bare repo path)
    local output
    output=$(PATH=$(echo "$PATH" | tr ':' '\n' | grep -v gh | tr '\n' ':') \
        .agent/scripts/worktree_create.sh --issue 555 --type workspace --draft-pr 2>&1) || true

    # Worktree should still be created successfully
    if [[ "$output" != *"Worktree Created Successfully"* ]]; then
        echo "    Expected worktree creation success message"
        echo "    Output: $output"
        cleanup_mock_workspace
        return 1
    fi

    # Work plan file should exist in the worktree
    # Slug is "origin" (basename of the bare repo path without .git)
    local worktree_plan="$WORKSPACE_DIR/.workspace-worktrees/issue-origin-555/.agent/work-plans/PLAN_ISSUE-555.md"
    if [ ! -f "$worktree_plan" ]; then
        echo "    Expected work plan at: $worktree_plan"
        cleanup_mock_workspace
        return 1
    fi

    # Verify template substitution worked (issue number should appear)
    if ! grep -q "Issue #555" "$worktree_plan"; then
        echo "    Expected issue number substitution in work plan"
        cleanup_mock_workspace
        return 1
    fi

    cleanup_mock_workspace
    return 0
}
run_test "--draft-pr creates work plan for workspace type" test_draft_pr_workspace_creates_plan

# Test 8: layer worktree packages have .git file detected by -e (not just -d)
test_layer_worktree_has_git_file() {
    setup_mock_layer_workspace
    cd "$WORKSPACE_DIR" || return 1

    # Create a layer worktree (slug auto-detected as "pkg_origin")
    local output
    output=$(.agent/scripts/worktree_create.sh --issue 666 --type layer --layer core --packages test_pkg 2>&1) || true

    if [[ "$output" != *"Worktree Created Successfully"* ]]; then
        echo "    Failed to create layer worktree"
        cleanup_mock_layer_workspace
        return 1
    fi

    # The package in the layer worktree should have a .git entry
    # (git worktree creates a .git *file*, not a directory)
    local wt_pkg_dir="$WORKSPACE_DIR/layers/worktrees/issue-pkg_origin-666/core_ws/src/test_pkg"
    if [ ! -e "$wt_pkg_dir/.git" ]; then
        echo "    No .git entry in worktree package dir: $wt_pkg_dir"
        cleanup_mock_layer_workspace
        return 1
    fi

    # Confirm it's a file (worktree marker) not a directory
    if [ -d "$wt_pkg_dir/.git" ]; then
        echo "    .git is a directory — expected a file for git worktrees"
        cleanup_mock_layer_workspace
        return 1
    fi

    cleanup_mock_layer_workspace
    return 0
}
run_test "Layer worktree packages have .git file (not directory)" test_layer_worktree_has_git_file

# Summary
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
