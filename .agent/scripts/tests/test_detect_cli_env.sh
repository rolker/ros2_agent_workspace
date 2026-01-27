#!/bin/bash
# .agent/scripts/tests/test_detect_cli_env.sh
# Basic tests for framework detection

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEST_PASS=0
TEST_FAIL=0

echo "=== Testing Framework Detection ==="
echo ""

# Test 1: Copilot CLI detection (if gh is available)
echo "Test 1: Copilot CLI detection"
if command -v gh &> /dev/null && gh copilot --version &> /dev/null 2>&1; then
    source "$SCRIPT_DIR/../detect_cli_env.sh"
    if [ "$AGENT_FRAMEWORK" == "copilot-cli" ]; then
        echo "✅ PASS: Detected copilot-cli"
        ((TEST_PASS++))
    else
        echo "❌ FAIL: Expected copilot-cli, got $AGENT_FRAMEWORK"
        ((TEST_FAIL++))
    fi
else
    echo "⊘  SKIP: GitHub CLI not available or copilot extension not installed"
fi
echo ""

# Test 2: Gemini detection via environment variable
echo "Test 2: Gemini CLI detection via GEMINI_API_KEY"
(
    export GEMINI_API_KEY="test_key"
    source "$SCRIPT_DIR/../detect_cli_env.sh"
    if [ "$AGENT_FRAMEWORK" == "gemini-cli" ]; then
        echo "✅ PASS: Detected gemini-cli via API key"
        exit 0
    else
        echo "❌ FAIL: Expected gemini-cli, got $AGENT_FRAMEWORK"
        exit 1
    fi
)
if [ $? -eq 0 ]; then
    ((TEST_PASS++))
else
    ((TEST_FAIL++))
fi
echo ""

# Test 3: Antigravity detection via USER
echo "Test 3: Antigravity detection via USER variable"
(
    export USER="antigravity"
    source "$SCRIPT_DIR/../detect_cli_env.sh"
    if [ "$AGENT_FRAMEWORK" == "antigravity" ]; then
        echo "✅ PASS: Detected antigravity via USER"
        exit 0
    else
        echo "❌ FAIL: Expected antigravity, got $AGENT_FRAMEWORK"
        exit 1
    fi
)
if [ $? -eq 0 ]; then
    ((TEST_PASS++))
else
    ((TEST_FAIL++))
fi
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
