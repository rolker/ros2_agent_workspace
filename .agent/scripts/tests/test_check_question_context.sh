#!/bin/bash
# .agent/scripts/tests/test_check_question_context.sh
# Smoke test for the warn-only AskUserQuestion context hook
# (.agent/hooks/check_question_context.py, #592).
#
# The hook is warn-only and fail-safe by contract: it must never break an
# AskUserQuestion call (always exit 0), nudge only when NO question opens with a
# repo-qualified <repo>#<N> token, and stay silent on conforming or malformed
# input. These cases pin exactly that.
#
# Run: bash .agent/scripts/tests/test_check_question_context.sh

set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HOOK="$SCRIPT_DIR/../../hooks/check_question_context.py"
TEST_PASS=0
TEST_FAIL=0

pass() { echo "PASS: $1"; TEST_PASS=$((TEST_PASS + 1)); }
fail() { echo "FAIL: $1"; TEST_FAIL=$((TEST_FAIL + 1)); }

# Run the hook with the given stdin; capture stdout + exit code into globals.
run_hook() {
    HOOK_OUT="$(printf '%s' "$1" | python3 "$HOOK" 2>/dev/null)"
    HOOK_RC=$?
}

# 1. Conforming question (opens with <repo>#<N>) → silent allow, exit 0.
run_hook '{"tool_name":"AskUserQuestion","tool_input":{"questions":[{"question":"project11_navigation#466 (PR #467): Recover GPS — phase 6 of 7; how to proceed?"}]}}'
if [ "$HOOK_RC" -eq 0 ] && [ -z "$HOOK_OUT" ]; then
    pass "conforming question → silent allow, exit 0"
else
    fail "conforming question (rc=$HOOK_RC, out=$HOOK_OUT)"
fi

# 2. Non-conforming question (bare 'PR #25', no <repo>#<N>) → warn present, exit 0.
run_hook '{"tool_name":"AskUserQuestion","tool_input":{"questions":[{"question":"How should we handle the four open findings on PR #25?"}]}}'
if [ "$HOOK_RC" -eq 0 ] && printf '%s' "$HOOK_OUT" | grep -q 'systemMessage' \
    && printf '%s' "$HOOK_OUT" | grep -qi 're-orientation header'; then
    pass "non-conforming question → warn present, exit 0"
else
    fail "non-conforming question (rc=$HOOK_RC, out=$HOOK_OUT)"
fi

# 2b. The warn must NOT carry a blocking decision (warn-only contract).
if printf '%s' "$HOOK_OUT" | grep -qiE '"(permissionDecision|decision)"[[:space:]]*:[[:space:]]*"(deny|block)"'; then
    fail "warn output must not contain a deny/block decision"
else
    pass "warn output carries no deny/block decision"
fi

# 3. One conforming among several → silent allow (at least one satisfies).
run_hook '{"tool_name":"AskUserQuestion","tool_input":{"questions":[{"question":"plain question with no token"},{"question":"ros2_agent_workspace#592: fix hook — phase 4; ok?"}]}}'
if [ "$HOOK_RC" -eq 0 ] && [ -z "$HOOK_OUT" ]; then
    pass "at least one conforming question → silent allow"
else
    fail "mixed questions with one conforming (rc=$HOOK_RC, out=$HOOK_OUT)"
fi

# 4. Malformed JSON → silent allow, exit 0 (fail safe).
run_hook 'not json at all'
if [ "$HOOK_RC" -eq 0 ] && [ -z "$HOOK_OUT" ]; then
    pass "malformed stdin → silent allow, exit 0"
else
    fail "malformed stdin (rc=$HOOK_RC, out=$HOOK_OUT)"
fi

# 5. Empty stdin → silent allow, exit 0 (fail safe).
run_hook ''
if [ "$HOOK_RC" -eq 0 ] && [ -z "$HOOK_OUT" ]; then
    pass "empty stdin → silent allow, exit 0"
else
    fail "empty stdin (rc=$HOOK_RC, out=$HOOK_OUT)"
fi

# 6. Valid JSON but no questions list → silent allow (unexpected schema, fail safe).
run_hook '{"tool_name":"AskUserQuestion","tool_input":{}}'
if [ "$HOOK_RC" -eq 0 ] && [ -z "$HOOK_OUT" ]; then
    pass "missing questions list → silent allow, exit 0"
else
    fail "missing questions list (rc=$HOOK_RC, out=$HOOK_OUT)"
fi

echo
echo "check_question_context.py tests: $TEST_PASS passed, $TEST_FAIL failed"
[ "$TEST_FAIL" -eq 0 ]
