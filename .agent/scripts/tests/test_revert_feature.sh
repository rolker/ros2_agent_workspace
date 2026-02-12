#!/bin/bash
# .agent/scripts/tests/test_revert_feature.sh
# Tests for revert_feature.sh script

# Note: Not using set -e because we want to continue after test failures

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REVERT_SCRIPT="$SCRIPT_DIR/../revert_feature.sh"
TEST_PASS=0
TEST_FAIL=0
TEST_SKIP=0

# Setup test repository in temp directory
setup_test_repo() {
    TEST_REPO=$(mktemp -d)
    cd "$TEST_REPO"
    git init -q
    git config user.email "test@example.com"
    git config user.name "Test User"

    # Create initial commit
    echo "initial" > file.txt
    git add file.txt
    git commit -q -m "Initial commit"
}

# Cleanup test repository
cleanup_test_repo() {
    if [ -n "$TEST_REPO" ] && [ -d "$TEST_REPO" ]; then
        cd /
        rm -rf "$TEST_REPO"
    fi
}

# Test helper
run_test() {
    local test_name="$1"
    local test_func="$2"

    echo "Test: $test_name"

    if $test_func; then
        echo "✅ PASS: $test_name"
        ((TEST_PASS++))
    else
        echo "❌ FAIL: $test_name"
        ((TEST_FAIL++))
    fi
    echo ""
}

echo "=== Testing revert_feature.sh ==="
echo ""

# Test 1: Help flag works
test_help_flag() {
    local output
    output=$("$REVERT_SCRIPT" --help 2>&1 || true)
    [[ "$output" =~ "Usage:" ]] && [[ "$output" =~ "--issue" ]]
}
run_test "Help flag displays usage" test_help_flag

# Test 2: Missing issue argument shows error
test_missing_issue() {
    local output
    output=$("$REVERT_SCRIPT" 2>&1 || true)
    [[ "$output" =~ "Error: --issue" ]]
}
run_test "Missing --issue argument shows error" test_missing_issue

# Test 3: Non-numeric issue number rejected
test_invalid_issue_number() {
    (
        setup_test_repo
        local output result
        output=$("$REVERT_SCRIPT" --issue "abc" 2>&1 || true)
        result=$?
        cleanup_test_repo
        [[ $result -ne 0 ]] && [[ "$output" =~ "must be numeric" ]]
    )
}
run_test "Non-numeric issue number rejected" test_invalid_issue_number

# Test 4: No commits found for issue
test_no_commits_found() {
    (
        setup_test_repo
        local output result
        output=$("$REVERT_SCRIPT" --issue 999 2>&1 || true)
        result=$?
        cleanup_test_repo
        [[ $result -ne 0 ]] && [[ "$output" =~ "No commits found" ]]
    )
}
run_test "Non-existent issue exits with error" test_no_commits_found

# Test 5: Dry-run mode doesn't make changes
test_dry_run_mode() {
    setup_test_repo

    # Create a commit referencing issue #123
    echo "change1" > file.txt
    git add file.txt
    git commit -q -m "Fix something

Fixes #123"

    local before_commit=$(git rev-parse HEAD)

    # Run dry-run
    local output
    output=$("$REVERT_SCRIPT" --issue 123 --dry-run 2>&1)

    local after_commit=$(git rev-parse HEAD)

    cleanup_test_repo

    # Verify no changes and correct output
    [[ "$before_commit" == "$after_commit" ]] && \
    [[ "$output" =~ "DRY RUN" ]] && \
    [[ "$output" =~ "Would revert" ]]
}
run_test "Dry-run mode doesn't make changes" test_dry_run_mode

# Test 6: Single commit revert (requires interaction, skip in CI)
test_single_commit_revert() {
    if [ -n "$CI" ]; then
        echo "⊘  SKIP: Requires user interaction"
        ((TEST_SKIP++))
        return 0
    fi

    setup_test_repo

    # Create commit with issue reference
    echo "change1" > file.txt
    git add file.txt
    git commit -q -m "Add feature

Implements #456"

    # Auto-confirm with 'y'
    echo "y" | "$REVERT_SCRIPT" --issue 456 > /dev/null 2>&1
    local result=$?

    # Check that we have a revert commit
    local last_msg=$(git log -1 --format=%s)

    cleanup_test_repo

    [[ $result -eq 0 ]] && [[ "$last_msg" =~ "Revert" ]]
}
run_test "Single commit successfully reverted" test_single_commit_revert

# Test 7: Multiple commits reverted in correct order
test_multiple_commits_order() {
    if [ -n "$CI" ]; then
        echo "⊘  SKIP: Requires user interaction"
        ((TEST_SKIP++))
        return 0
    fi

    setup_test_repo

    # Create multiple commits for same issue
    echo "change1" > file.txt
    git add file.txt
    git commit -q -m "Part 1 of feature #789"

    echo "change2" > file.txt
    git add file.txt
    git commit -q -m "Part 2 of feature #789"

    echo "change3" > file.txt
    git add file.txt
    git commit -q -m "Part 3 of feature #789"

    # Auto-confirm with 'y'
    local output
    output=$(echo "y" | "$REVERT_SCRIPT" --issue 789 2>&1)

    # Verify success message shows correct count
    cleanup_test_repo

    [[ "$output" =~ "Successfully reverted 3 commit(s)" ]]
}
run_test "Multiple commits reverted with correct count" test_multiple_commits_order

# Test 8: Script finds commits with various reference formats
test_commit_reference_formats() {
    setup_test_repo

    # Create commits with different reference formats
    echo "change1" > file1.txt
    git add file1.txt
    git commit -q -m "Fix bug #111"

    echo "change2" > file2.txt
    git add file2.txt
    git commit -q -m "Update docs

Closes #111"

    echo "change3" > file3.txt
    git add file3.txt
    git commit -q -m "Add test

Fixes #111"

    # Check dry-run finds all 3 commits
    local output
    output=$("$REVERT_SCRIPT" --issue 111 --dry-run 2>&1)

    local found_count=$(echo "$output" | grep -c "Found 3 commit(s)")

    cleanup_test_repo

    [[ $found_count -eq 1 ]]
}
run_test "Finds commits with various reference formats" test_commit_reference_formats

# Test 9: Unknown option shows error
test_unknown_option() {
    local output
    output=$("$REVERT_SCRIPT" --unknown-flag 2>&1 || true)
    [[ "$output" =~ "Error: Unknown option" ]]
}
run_test "Unknown option shows error" test_unknown_option

# Test 10: Verify mapfile usage (no subshell issues)
test_no_subshell_issues() {
    setup_test_repo

    # Create commit
    echo "change" > file.txt
    git add file.txt
    git commit -q -m "Feature #222"

    # Run with auto-confirm and check the counter in output
    local output
    output=$(echo "y" | "$REVERT_SCRIPT" --issue 222 2>&1)

    cleanup_test_repo

    # Should show "Successfully reverted 1 commit(s)" - proves counter works
    [[ "$output" =~ "Successfully reverted 1 commit(s)" ]]
}
run_test "Revert counter works correctly (no subshell issues)" test_no_subshell_issues

# Summary
echo "========================================"
echo "TEST RESULTS"
echo "========================================"
echo "Passed: $TEST_PASS"
echo "Failed: $TEST_FAIL"
echo "Skipped: $TEST_SKIP"
echo "========================================"

if [ $TEST_FAIL -eq 0 ]; then
    echo "✅ All tests passed!"
    exit 0
else
    echo "❌ Some tests failed"
    exit 1
fi
