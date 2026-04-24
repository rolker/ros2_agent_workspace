#!/bin/bash
# .agent/scripts/tests/test_field_mode.sh
# Tests for is_field_mode() and describe_mode() in field_mode.sh
#
# Stubs `git remote get-url origin` by initializing throwaway repos with
# specific origin URLs. Covers GitHub SSH/HTTPS variants, several
# non-GitHub origins, and substring traps (mygithub.com etc.) that the
# bare `*github.com*` pattern would mishandle.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT="$SCRIPT_DIR/../field_mode.sh"
TEST_PASS=0
TEST_FAIL=0

# Source the script under test
# shellcheck source=../field_mode.sh
source "$SCRIPT"

make_test_repo() {
    local origin="$1"
    local repo
    repo=$(mktemp -d /tmp/field_mode_test.XXXXXX)
    git -C "$repo" init -q
    if [ -n "$origin" ]; then
        git -C "$repo" remote add origin "$origin"
    fi
    echo "$repo"
}

assert_field_mode() {
    local origin="$1"
    local repo
    repo=$(make_test_repo "$origin")
    if is_field_mode "$repo"; then
        echo "✅ PASS: '$origin' → field mode"
        TEST_PASS=$((TEST_PASS + 1))
    else
        echo "❌ FAIL: '$origin' → expected field mode, got dev mode"
        TEST_FAIL=$((TEST_FAIL + 1))
    fi
    rm -rf "$repo"
}

assert_dev_mode() {
    local origin="$1"
    local repo
    repo=$(make_test_repo "$origin")
    if is_field_mode "$repo"; then
        echo "❌ FAIL: '$origin' → expected dev mode, got field mode"
        TEST_FAIL=$((TEST_FAIL + 1))
    else
        echo "✅ PASS: '$origin' → dev mode"
        TEST_PASS=$((TEST_PASS + 1))
    fi
    rm -rf "$repo"
}

echo "=== is_field_mode: github.com origins (dev mode) ==="
assert_dev_mode "git@github.com:rolker/ros2_agent_workspace.git"
assert_dev_mode "https://github.com/rolker/ros2_agent_workspace.git"
assert_dev_mode "ssh://git@github.com/rolker/ros2_agent_workspace.git"
assert_dev_mode "git@github.com:rolker/repo"  # no .git suffix
echo ""

echo "=== is_field_mode: non-github origins (field mode) ==="
assert_field_mode "git@gitcloud:field/test-repo.git"
assert_field_mode "https://gitlab.com/foo/bar.git"
assert_field_mode "git@forgejo.example.com:org/repo.git"
assert_field_mode "ssh://git@gitea.internal/team/repo.git"
echo ""

echo "=== is_field_mode: substring traps (must be field mode, not dev) ==="
# These hostnames CONTAIN 'github.com' as a substring but aren't actually github.com
assert_field_mode "git@mygithub.com:user/repo.git"
assert_field_mode "https://notgithub.com/foo/bar.git"
assert_field_mode "git@github.company.internal:org/repo.git"
echo ""

echo "=== is_field_mode: path-position leaks (must be field mode) ==="
# The host is example.com / gitcloud / etc. — 'github.com' only appears in
# the URL path. A naive regex against the full URL would misclassify these.
assert_field_mode "https://example.com/github.com/foo.git"
assert_field_mode "https://example.com/mirrors/github.com/foo"
assert_field_mode "git@gitcloud:team/github.com-mirror.git"
echo ""

echo "=== is_field_mode: edge cases ==="
# No origin remote at all → dev mode (returns 1, the safer default)
NO_ORIGIN_REPO=$(mktemp -d /tmp/no_origin.XXXXXX)
git -C "$NO_ORIGIN_REPO" init -q
if is_field_mode "$NO_ORIGIN_REPO"; then
    echo "❌ FAIL: no origin → expected dev mode (return 1)"
    TEST_FAIL=$((TEST_FAIL + 1))
else
    echo "✅ PASS: no origin → dev mode (return 1, safer default)"
    TEST_PASS=$((TEST_PASS + 1))
fi
rm -rf "$NO_ORIGIN_REPO"

# Non-existent path → dev mode (return 1)
if is_field_mode "/nonexistent/path/$$"; then
    echo "❌ FAIL: nonexistent path → expected return 1"
    TEST_FAIL=$((TEST_FAIL + 1))
else
    echo "✅ PASS: nonexistent path → return 1"
    TEST_PASS=$((TEST_PASS + 1))
fi
echo ""

echo "=== describe_mode output sanity ==="
DEV_REPO=$(make_test_repo "git@github.com:rolker/test.git")
DESC=$(describe_mode "$DEV_REPO")
if [[ "$DESC" == *"dev mode"* ]] && [[ "$DESC" == *"github.com"* ]]; then
    echo "✅ PASS: describe_mode dev → '$DESC'"
    TEST_PASS=$((TEST_PASS + 1))
else
    echo "❌ FAIL: describe_mode dev → '$DESC'"
    TEST_FAIL=$((TEST_FAIL + 1))
fi
rm -rf "$DEV_REPO"

FIELD_REPO=$(make_test_repo "git@gitcloud:field/test.git")
DESC=$(describe_mode "$FIELD_REPO")
if [[ "$DESC" == *"field mode"* ]] && [[ "$DESC" == *"gitcloud"* ]]; then
    echo "✅ PASS: describe_mode field → '$DESC'"
    TEST_PASS=$((TEST_PASS + 1))
else
    echo "❌ FAIL: describe_mode field → '$DESC'"
    TEST_FAIL=$((TEST_FAIL + 1))
fi
rm -rf "$FIELD_REPO"
echo ""

echo "=== Results ==="
echo "Passed: $TEST_PASS"
echo "Failed: $TEST_FAIL"
exit "$TEST_FAIL"
