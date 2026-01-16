#!/bin/bash
# scripts/test.sh
# Unified test script for ROS2 Agent Workspace
#
# Usage: ./scripts/test.sh
#
# This script will:
# 1. Check for workspace locks (multi-agent coordination)
# 2. Run tests on all workspace layers in order
# 3. Generate a detailed test report in ai_workspace/test_report.md
# 4. Continue testing other layers even if one fails (for full status)
#
# Exit codes:
#   0 - All tests passed
#   1 - Tests failed in one or more layers

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
REPORT_FILE="$ROOT_DIR/ai_workspace/test_report.md"

# Ensure ai_workspace exists
mkdir -p "$ROOT_DIR/ai_workspace"
LOCK_FILE="$ROOT_DIR/ai_workspace/workspace.lock"

# Check for Lock
if [ -f "$LOCK_FILE" ]; then
    echo "❌ Workspace is LOCKED."
    echo "This script cannot run because another agent/task is active."
    echo "Lock details:"
    cat "$LOCK_FILE"
    echo ""
    echo "If you are sure this is a stale lock, run: ./scripts/unlock.sh"
    exit 1
fi

# Header for the Report
echo "# Test Report - $(date)" > "$REPORT_FILE"
echo "" >> "$REPORT_FILE"
echo "| Layer | Result | Summary |" >> "$REPORT_FILE"
echo "|---|---|---|" >> "$REPORT_FILE"

# Load Layer Definitions
source "$SCRIPT_DIR/env.sh" > /dev/null

echo "========================================"
echo "Starting Test Sequence"
echo "========================================"

TOTAL_FAILURES=0
FAILED_LAYERS=""

for layer in "${LAYERS[@]}"; do
    WORKSPACE_DIR="$ROOT_DIR/workspaces/${layer}_ws"
    
    if [ ! -d "$WORKSPACE_DIR/src" ]; then
        echo "Skipping layer $layer (no src directory)"
        echo "| $layer | ⏭️ Skipped | No source |" >> "$REPORT_FILE"
        continue
    fi

    echo "----------------------------------------"
    echo "Testing Layer: $layer"
    echo "----------------------------------------"

    # Go to workspace
    cd "$WORKSPACE_DIR" || exit 1

    # Source environment to ensure tests can run (requires install/setup.bash)
    if [ -f "install/setup.bash" ]; then
        source "install/setup.bash"
    else
        echo "⚠️ Warning: No install/setup.bash found. Tests might fail if not built."
    fi

    # Run Colcon Test
    # --event-handlers console_direct+ shows output in real-time
    # --return-code-on-test-failure guarantees non-zero exit if tests fail
    colcon test --event-handlers console_direct+ --return-code-on-test-failure
    TEST_RC=$?

    # Get Test Results Summary
    # colcon test-result --all shows all cases
    RESULT_SUMMARY=$(colcon test-result --verbose | grep -E "Total|Errors|Failures" | paste -sd " " -)
    if [ -z "$RESULT_SUMMARY" ]; then
        RESULT_SUMMARY="No tests found or ran."
    fi

    if [ $TEST_RC -eq 0 ]; then
        echo "✅ Tests Passed for $layer"
        echo "| $layer | ✅ Passed | $RESULT_SUMMARY |" >> "$REPORT_FILE"
    else
        echo "❌ Tests Failed for $layer"
        echo "| $layer | ❌ Failed | $RESULT_SUMMARY |" >> "$REPORT_FILE"
        TOTAL_FAILURES=$((TOTAL_FAILURES + 1))
        FAILED_LAYERS="$FAILED_LAYERS $layer"
        
        # We generally continue testing other layers unless critical, 
        # but for CI-like behavior we might want to stop. 
        # Here we continue to gather full status.
    fi

done

echo "========================================"
echo "Test Summary"
echo "========================================"
cat "$REPORT_FILE"

if [ $TOTAL_FAILURES -eq 0 ]; then
    echo "All tests passed successfully."
    exit 0
else
    echo "Tests failed in layers: $FAILED_LAYERS"
    exit 1
fi
