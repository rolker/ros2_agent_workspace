#!/bin/bash
# Regression test for .agent/hooks/check-commit-identity.py.
#
# Verifies the branch-and-env-aware gate from #468:
# - Strict mode fires only when AGENT_NAME is set AND the branch matches
#   the agent-branch convention (feature/issue-N, feature/ISSUE-N-desc,
#   skill/<name>-<timestamp>).
# - Permissive mode applies otherwise (AGENT_NAME unset, non-agent branch,
#   detached HEAD).
#
# Standalone-runnable; no Makefile target required. Run from anywhere:
#     bash .agent/scripts/test_check_commit_identity.sh

if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
    echo "Error: This script should be executed, not sourced."
    return 1
fi

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
HOOK="$WORKSPACE_ROOT/hooks/check-commit-identity.py"

if [[ ! -f "$HOOK" ]]; then
    echo "❌ Hook not found at $HOOK"
    exit 2
fi

TMPDIR="$(mktemp -d)"
trap 'rm -rf "$TMPDIR"' EXIT

cd "$TMPDIR" || exit 2
git init --quiet --initial-branch=main
git config user.name "Test Setup"
git config user.email "test-setup@example.com"
git commit --allow-empty --quiet -m "init"

PASS=0
FAIL=0
FAIL_DETAILS=()

# run_case <desc> <branch> <AGENT_NAME> <GIT_AUTHOR_EMAIL> <expected_exit>
#
# branch:           literal branch name to checkout (or "__DETACHED__" to
#                   detach HEAD instead of creating/switching a branch)
# AGENT_NAME:       passed via env; empty string means unset
# GIT_AUTHOR_EMAIL: passed via env; takes precedence over git config
# expected_exit:    0 for accept, 1 for reject
run_case() {
    local desc="$1"
    local branch="$2"
    local agent_name="$3"
    local email="$4"
    local expected_exit="$5"

    if [[ "$branch" == "__DETACHED__" ]]; then
        git checkout --detach --quiet HEAD
    else
        # -B creates or resets the branch and switches to it (works for any name)
        git checkout -B "$branch" --quiet 2>/dev/null
    fi

    local actual_exit=0
    local output
    output=$(
        AGENT_NAME="$agent_name" GIT_AUTHOR_EMAIL="$email" \
        python3 "$HOOK" 2>&1
    ) || actual_exit=$?

    if [[ "$actual_exit" == "$expected_exit" ]]; then
        echo "✅ $desc"
        PASS=$((PASS + 1))
    else
        echo "❌ $desc"
        echo "   branch=$branch AGENT_NAME='$agent_name' email='$email'"
        echo "   expected exit=$expected_exit, got=$actual_exit"
        FAIL=$((FAIL + 1))
        FAIL_DETAILS+=("$desc")
        # Indented hook output for diagnostic context
        while IFS= read -r line; do echo "     $line"; done <<<"$output"
    fi
}

echo "=== check-commit-identity.py regression test ==="
echo "Hook: $HOOK"
echo ""

# --- Strict-mode cases (agent branch + AGENT_NAME set) ---
run_case "strict: feature/issue-N + AGENT_NAME + human email → reject" \
    "feature/issue-100" "Claude Code Agent" "roland@ccom.unh.edu" 1

run_case "strict: feature/issue-N + AGENT_NAME + agent email → accept" \
    "feature/issue-100" "Claude Code Agent" "roland+claude-code@ccom.unh.edu" 0

run_case "strict: feature/ISSUE-N-desc (uppercase) + AGENT_NAME + human email → reject" \
    "feature/ISSUE-100-some-description" "Claude Code Agent" "roland@ccom.unh.edu" 1

run_case "strict: skill/<name>-<timestamp> + AGENT_NAME + human email → reject" \
    "skill/research-20260518-120000-123456789" "Claude Code Agent" "roland@ccom.unh.edu" 1

run_case "strict: feature/issue-N + AGENT_NAME + rolker.net (other human) → reject" \
    "feature/issue-100" "Claude Code Agent" "roland@rolker.net" 1

# --- Permissive-mode cases (AGENT_NAME unset OR non-agent branch) ---
run_case "permissive: agent branch + AGENT_NAME unset + human email → accept" \
    "feature/issue-100" "" "roland@ccom.unh.edu" 0

run_case "permissive: agent branch + AGENT_NAME unset + agent email → accept" \
    "feature/issue-100" "" "roland+claude-code@ccom.unh.edu" 0

run_case "permissive: non-agent branch (main) + AGENT_NAME + human email → accept" \
    "main" "Claude Code Agent" "roland@ccom.unh.edu" 0

run_case "permissive: non-agent branch (fix/foo) + AGENT_NAME + human email → accept" \
    "fix/something" "Claude Code Agent" "roland@ccom.unh.edu" 0

run_case "permissive: detached HEAD + AGENT_NAME + human email → accept" \
    "__DETACHED__" "Claude Code Agent" "roland@ccom.unh.edu" 0

# --- Reject-in-both-modes cases (email matches no accepted pattern) ---
run_case "permissive: unknown email → reject (current behavior preserved)" \
    "main" "" "stranger@example.com" 1

run_case "strict: unknown email → reject" \
    "feature/issue-100" "Claude Code Agent" "stranger@example.com" 1

echo ""
echo "=== Results: $PASS passed, $FAIL failed ==="

if [[ "$FAIL" -gt 0 ]]; then
    echo ""
    echo "Failures:"
    for d in "${FAIL_DETAILS[@]}"; do echo "  - $d"; done
    exit 1
fi

exit 0
