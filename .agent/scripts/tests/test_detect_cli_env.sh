#!/bin/bash
# .agent/scripts/tests/test_detect_cli_env.sh
# Tests the framework-detection contract of detect_cli_env.sh.
#
# detect_cli_env.sh keys ONLY on session-specific signals — deliberately NOT on
# API credentials (GEMINI_API_KEY / ANTHROPIC_API_KEY / CLAUDE_API_KEY), which
# users commonly have in their profile and which do not indicate an active
# session. The negative cases below pin that intentional behavior.
#
# Each case runs detection in a fresh `env -i` subshell so exactly one signal is
# present — the suite itself runs inside an agent session (e.g. Claude Code sets
# CLAUDECODE), which would otherwise leak in and mask non-claude expectations.
#
# Run: bash .agent/scripts/tests/test_detect_cli_env.sh

set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DETECT="$SCRIPT_DIR/../detect_cli_env.sh"
TEST_PASS=0
TEST_FAIL=0

# assert_framework <desc> <expected> [VAR=val ...]
# Runs detection with ONLY the given vars set (clean env + PATH for `command -v`).
# `source ... || true` guards the unknown-case `return 1`; AGENT_FRAMEWORK is
# always assigned ("unknown") before that return, so the value is meaningful.
assert_framework() {
    local desc="$1" expected="$2"; shift 2
    local got
    got=$(env -i PATH="$PATH" DETECT="$DETECT" "$@" bash -c 'source "$DETECT" 2>/dev/null || true; printf "%s" "$AGENT_FRAMEWORK"')
    if [ "$got" = "$expected" ]; then
        echo "  ✅ $desc"; TEST_PASS=$((TEST_PASS + 1))
    else
        echo "  ❌ $desc (expected '$expected', got '$got')"; TEST_FAIL=$((TEST_FAIL + 1))
    fi
}

echo "=== Testing detect_cli_env.sh detection contract ==="
echo ""

echo "Positive cases (session-specific signals):"
assert_framework "Copilot via COPILOT_API_URL"            copilot-cli  COPILOT_API_URL="https://example"
assert_framework "Copilot via COPILOT_AGENT_CALLBACK_URL" copilot-cli  COPILOT_AGENT_CALLBACK_URL="https://example"
assert_framework "Gemini via GEMINI_SESSION"              gemini-cli   GEMINI_SESSION="1"
assert_framework "Gemini via gemini-ish USER"             gemini-cli   USER="gemini-bot"
assert_framework "Antigravity via USER"                   antigravity  USER="antigravity"
assert_framework "Antigravity via ANTIGRAVITY_SESSION"    antigravity  ANTIGRAVITY_SESSION="1"
assert_framework "Claude via CLAUDECODE"                  claude-code  CLAUDECODE="1"
assert_framework "Claude via CLAUDE_CODE"                 claude-code  CLAUDE_CODE="1"
assert_framework "Claude via CLAUDE_CODE_ENTRYPOINT"      claude-code  CLAUDE_CODE_ENTRYPOINT="cli"
assert_framework "Generic via AI_AGENT_NAME (lowercased)" foobot       AI_AGENT_NAME="FooBot"
echo ""

echo "Negative cases (no session signal → unknown):"
assert_framework "Bare environment"                       unknown
assert_framework "GEMINI_API_KEY is NOT a session signal" unknown      GEMINI_API_KEY="test_key"
assert_framework "GOOGLE_API_KEY is NOT a session signal" unknown      GOOGLE_API_KEY="test_key"
assert_framework "ANTHROPIC_API_KEY is NOT a session signal" unknown   ANTHROPIC_API_KEY="test_key"
assert_framework "CLAUDE_API_KEY is NOT a session signal" unknown      CLAUDE_API_KEY="test_key"
echo ""

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
