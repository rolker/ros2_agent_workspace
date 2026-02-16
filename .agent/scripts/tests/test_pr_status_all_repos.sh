#!/bin/bash
# .agent/scripts/tests/test_pr_status_all_repos.sh
# Smoke tests for pr_status.sh --all-repos mode
#
# Uses stubbed `gh` and `list_overlay_repos.py` to verify:
# - JSON output contains repo field and expected structure
# - Simple output is clean (no progress noise)
# - Dashboard mode shows progress messages
# - Empty repo list doesn't crash
# - Non-GitHub URLs are skipped

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PR_STATUS="$SCRIPT_DIR/../pr_status.sh"

TEST_PASS=0
TEST_FAIL=0
TMPDIR_ROOT=$(mktemp -d /tmp/test_pr_status.XXXXXX)

cleanup() {
    rm -rf "$TMPDIR_ROOT"
}
trap cleanup EXIT

pass() {
    echo "✅ PASS: $1"
    TEST_PASS=$((TEST_PASS + 1))
}

fail() {
    echo "❌ FAIL: $1"
    TEST_FAIL=$((TEST_FAIL + 1))
}

# --- Helpers to build stub directories ---

# Create a stub gh binary that returns canned PR data
# Args: target_dir [pr_json]
create_gh_stub() {
    local dir=$1
    local pr_json=${2:-'[{"number":42,"title":"Test PR","updatedAt":"2025-01-01T00:00:00Z","reviewDecision":"PENDING"}]'}

    cat > "$dir/gh" << STUBEOF
#!/bin/bash
# Stub gh for testing pr_status.sh
case "\$1" in
    pr)
        echo '$pr_json'
        ;;
    api)
        # Return empty array for review/comment API calls
        echo '[]'
        ;;
    *)
        echo '[]'
        ;;
esac
STUBEOF
    chmod +x "$dir/gh"
}

# Create a stub list_overlay_repos.py
# Args: target_dir [json_output]
create_overlay_stub() {
    local dir=$1
    local json_output=${2:-'[{"url":"https://github.com/testorg/testrepo.git"}]'}

    cat > "$dir/list_overlay_repos.py" << STUBEOF
#!/usr/bin/env python3
import sys
# Ignore any arguments
print('$json_output')
STUBEOF
    chmod +x "$dir/list_overlay_repos.py"
}

# Create a stub git that returns a GitHub origin URL
# Args: target_dir [url]
create_git_stub() {
    local dir=$1
    local url=${2:-'https://github.com/testorg/workspace.git'}

    cat > "$dir/git" << STUBEOF
#!/bin/bash
# Stub git for testing pr_status.sh
if [[ "\$*" == *"remote get-url"* ]]; then
    echo '$url'
else
    # Pass through to real git for other commands
    /usr/bin/git "\$@"
fi
STUBEOF
    chmod +x "$dir/git"
}

# Set up a test environment directory with stubs + pr_status.sh copy
# Args: test_name [overlay_json] [git_url]
# Outputs: sets TEST_ENV_DIR variable
setup_test_env() {
    local test_name=$1
    local overlay_json=${2:-'[{"url":"https://github.com/testorg/testrepo.git"}]'}
    local git_url=${3:-'https://github.com/testorg/workspace.git'}

    TEST_ENV_DIR="$TMPDIR_ROOT/$test_name"
    mkdir -p "$TEST_ENV_DIR/bin"

    create_gh_stub "$TEST_ENV_DIR/bin"
    create_overlay_stub "$TEST_ENV_DIR/bin" "$overlay_json"
    create_git_stub "$TEST_ENV_DIR/bin" "$git_url"

    # Place pr_status.sh alongside stubs so SCRIPT_DIR resolves to bin/
    cp "$PR_STATUS" "$TEST_ENV_DIR/bin/pr_status.sh"

    export PATH="$TEST_ENV_DIR/bin:$PATH"
    export REPO_ROOT="$TEST_ENV_DIR"
    mkdir -p "$TEST_ENV_DIR/.agent/scripts"
}

echo "=== Testing pr_status.sh --all-repos ==="
echo ""

# ---- Test 1: --all-repos --json produces valid JSON with repo field ----
echo "Test 1: --all-repos --json produces valid JSON with repo field"
(
    setup_test_env "test1"

    output=$(bash "$TEST_ENV_DIR/bin/pr_status.sh" --all-repos --json 2>/dev/null)

    # Check it's valid JSON
    if ! echo "$output" | jq . > /dev/null 2>&1; then
        echo "Output is not valid JSON: $output"
        exit 1
    fi

    # Check structure has summary and prs fields
    has_summary=$(echo "$output" | jq 'has("summary")' 2>/dev/null)
    has_prs=$(echo "$output" | jq 'has("prs")' 2>/dev/null)
    if [ "$has_summary" != "true" ] || [ "$has_prs" != "true" ]; then
        echo "Missing summary or prs field: $output"
        exit 1
    fi

    # Check prs have repo field
    repo_field=$(echo "$output" | jq -r '.prs[0].repo // "MISSING"' 2>/dev/null)
    if [ "$repo_field" = "MISSING" ] || [ -z "$repo_field" ]; then
        echo "PR missing repo field: $output"
        exit 1
    fi

    exit 0
)
if [ $? -eq 0 ]; then pass "JSON output valid with repo field"; else fail "JSON output valid with repo field"; fi
echo ""

# ---- Test 2: --all-repos --simple has no progress messages on stdout ----
echo "Test 2: --all-repos --simple produces clean output (no progress noise)"
(
    setup_test_env "test2"

    stdout_output=$(bash "$TEST_ENV_DIR/bin/pr_status.sh" --all-repos --simple 2>/dev/null)

    # stdout should NOT contain "Fetching PRs from"
    if echo "$stdout_output" | grep -q "Fetching PRs from"; then
        echo "stdout contains progress messages: $stdout_output"
        exit 1
    fi

    # stdout should contain SUMMARY line
    if ! echo "$stdout_output" | grep -q "SUMMARY:"; then
        echo "stdout missing SUMMARY line: $stdout_output"
        exit 1
    fi

    exit 0
)
if [ $? -eq 0 ]; then pass "Simple output clean"; else fail "Simple output clean"; fi
echo ""

# ---- Test 3: --all-repos --dashboard shows progress on stderr ----
echo "Test 3: --all-repos dashboard shows progress messages on stderr"
(
    setup_test_env "test3"

    stderr_output=$(bash "$TEST_ENV_DIR/bin/pr_status.sh" --all-repos 2>&1 1>/dev/null)

    if ! echo "$stderr_output" | grep -q "Fetching PRs from"; then
        echo "stderr missing progress messages: $stderr_output"
        exit 1
    fi

    exit 0
)
if [ $? -eq 0 ]; then pass "Dashboard shows progress on stderr"; else fail "Dashboard shows progress on stderr"; fi
echo ""

# ---- Test 4: Empty repo list doesn't crash ----
echo "Test 4: Empty repo list (no GitHub URLs) doesn't crash"
(
    setup_test_env "test4" '[]' 'https://gitlab.com/someorg/somerepo.git'

    # Should not crash even with no repos found
    output=$(bash "$TEST_ENV_DIR/bin/pr_status.sh" --all-repos --json 2>/dev/null)

    # Should produce valid JSON with 0 PRs
    total=$(echo "$output" | jq -r '.summary.total // -1' 2>/dev/null)
    if [ "$total" != "0" ]; then
        echo "Expected 0 total PRs, got: $total (output: $output)"
        exit 1
    fi

    exit 0
)
if [ $? -eq 0 ]; then pass "Empty repo list doesn't crash"; else fail "Empty repo list doesn't crash"; fi
echo ""

# ---- Test 5: Non-GitHub URLs are skipped ----
echo "Test 5: Non-GitHub URLs are skipped in discover_repos"
(
    setup_test_env "test5" \
        '[{"url":"https://gitlab.com/org/repo.git"},{"url":"https://github.com/testorg/goodrepo.git"}]'

    output=$(bash "$TEST_ENV_DIR/bin/pr_status.sh" --all-repos --json 2>/dev/null)

    # Should produce valid JSON
    if ! echo "$output" | jq . > /dev/null 2>&1; then
        echo "Output is not valid JSON: $output"
        exit 1
    fi

    # The gitlab URL should not appear in any PR repo field
    if echo "$output" | grep -q "gitlab.com"; then
        echo "GitLab URL leaked into output: $output"
        exit 1
    fi

    exit 0
)
if [ $? -eq 0 ]; then pass "Non-GitHub URLs skipped"; else fail "Non-GitHub URLs skipped"; fi
echo ""

# ---- Test 6: --interactive --all-repos is rejected ----
echo "Test 6: --interactive --all-repos is rejected"
(
    setup_test_env "test6"

    if bash "$TEST_ENV_DIR/bin/pr_status.sh" --interactive --all-repos 2>/dev/null; then
        echo "Expected non-zero exit code"
        exit 1
    fi

    exit 0
)
if [ $? -eq 0 ]; then pass "--interactive --all-repos rejected"; else fail "--interactive --all-repos rejected"; fi
echo ""

# Summary
echo "========================================"
echo "TEST RESULTS"
echo "========================================"
echo "Passed: $TEST_PASS"
echo "Failed: $TEST_FAIL"
echo "========================================"

if [ $TEST_FAIL -eq 0 ]; then
    echo "✅ All tests passed!"
    exit 0
else
    echo "❌ Some tests failed"
    exit 1
fi
