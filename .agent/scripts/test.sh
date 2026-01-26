#!/bin/bash
# .agent/scripts/test.sh
# Unified test script for ROS2 Agent Workspace.
# Adds JSON/CSV reporting for agents.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Go up two levels: .agent/scripts -> .agent -> root
ROOT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"
REPORT_FILE="$ROOT_DIR/ai_workspace/test_report.md"
HISTORY_FILE="$ROOT_DIR/ai_workspace/test_history.csv"
SUMMARY_JSON="$ROOT_DIR/ai_workspace/test_summary.json"

mkdir -p "$ROOT_DIR/ai_workspace"
LOCK_FILE="$ROOT_DIR/ai_workspace/workspace.lock"

# Check for Lock
if [ -f "$LOCK_FILE" ]; then
    echo "❌ Workspace is LOCKED by $(cat $LOCK_FILE)"
    exit 1
fi

# Initialize Report and JSON parts
echo "# Test Report - $(date)" > "$REPORT_FILE"
echo "" >> "$REPORT_FILE"
echo "| Layer | Result | Summary |" >> "$REPORT_FILE"
echo "|---|---|---|" >> "$REPORT_FILE"

# Init CSV header if new
if [ ! -f "$HISTORY_FILE" ]; then
    echo "timestamp,layer,result" > "$HISTORY_FILE"
fi

# Load Layers
if [ -f "$SCRIPT_DIR/env.sh" ]; then
    source "$SCRIPT_DIR/env.sh" > /dev/null
else
    # Fallback if env.sh missing
    LAYERS=("core") 
fi

echo "========================================"
echo "Starting Test Sequence"
echo "========================================"

OVERALL_SUCCESS=true
TOTAL_FAILURES=0
FAILED_LAYERS=""
JSON_LAYERS_ARRAY=()
TIMESTAMP="$(date -Iseconds)"

for layer in "${LAYERS[@]}"; do
    WORKSPACE_DIR="$ROOT_DIR/workspaces/${layer}_ws"
    LAYER_RESULT="skipped"
    LAYER_SUMMARY="No source"
    
    if [ -d "$WORKSPACE_DIR/src" ]; then
        echo "----------------------------------------"
        echo "Testing Layer: $layer"
        
        cd "$WORKSPACE_DIR" || continue
        
        # Source existing install if available
        if [ -f "install/setup.bash" ]; then
            source "install/setup.bash"
        fi

        # Run Tests
        colcon test --event-handlers console_direct+ --return-code-on-test-failure
        TEST_RC=$?
        
        # Summary
        RESULT_SUMMARY=$(colcon test-result --verbose | grep -E "Total|Errors|Failures" | paste -sd " " -)
        [ -z "$RESULT_SUMMARY" ] && RESULT_SUMMARY="No tests run"

        if [ $TEST_RC -eq 0 ]; then
            echo "✅ Tests Passed for $layer"
            LAYER_RESULT="passed"
            echo "| $layer | ✅ Passed | $RESULT_SUMMARY |" >> "$REPORT_FILE"
        else
            echo "❌ Tests Failed for $layer"
            LAYER_RESULT="failed"
            echo "| $layer | ❌ Failed | $RESULT_SUMMARY |" >> "$REPORT_FILE"
            OVERALL_SUCCESS=false
            TOTAL_FAILURES=$((TOTAL_FAILURES + 1))
            FAILED_LAYERS="$FAILED_LAYERS $layer"
        fi
        LAYER_SUMMARY="$RESULT_SUMMARY"
    else
        echo "Skipping $layer (no source)"
        echo "| $layer | ⏭️ Skipped | No source |" >> "$REPORT_FILE"
    fi

    # Append to CSV
    echo "$TIMESTAMP,$layer,$LAYER_RESULT" >> "$HISTORY_FILE"

    # Construct JSON Object for this layer (using python for safety if complex, but simple string here)
    # Escape quotes in summary just in case
    SAFE_SUMMARY=$(echo "$LAYER_SUMMARY" | sed 's/"/\\"/g')
    JSON_LAYERS_ARRAY+=("{\"name\": \"$layer\", \"result\": \"$LAYER_RESULT\", \"summary\": \"$SAFE_SUMMARY\"}")
done

# Build JSON Output
JSON_STR="{ \"timestamp\": \"$TIMESTAMP\", \"overall_success\": $OVERALL_SUCCESS, \"layers\": ["
FIRST=true
for item in "${JSON_LAYERS_ARRAY[@]}"; do
    if [ "$FIRST" = true ]; then FIRST=false; else JSON_STR+=","; fi
    JSON_STR+="$item"
done
JSON_STR+="] }"

echo "$JSON_STR" > "$SUMMARY_JSON"

echo "========================================"
echo "Test Summary"
cat "$REPORT_FILE"

if [ $TOTAL_FAILURES -eq 0 ]; then
    echo "All tests passed."
    exit 0
else
    echo "Tests failed in: $FAILED_LAYERS"
    exit 1
fi
