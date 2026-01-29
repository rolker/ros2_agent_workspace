#!/bin/bash
# .agent/scripts/test.sh
# Unified test script for ROS2 Agent Workspace.
# Adds JSON/CSV reporting for agents.
#
# Worktree Support:
#   When run from a worktree, tests are isolated and reports go to the
#   worktree's scratchpad.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Go up two levels: .agent/scripts -> .agent -> root
ROOT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

# Detect worktree context and set paths accordingly
LAYERS_BASE="$ROOT_DIR/layers/main"
SCRATCHPAD_DIR="$ROOT_DIR/.agent/scratchpad"
WORKTREE_INFO=""

# Check if we're in a layer worktree
if [[ "$ROOT_DIR" == *"/layers/worktrees/"* ]]; then
    WORKTREE_INFO="layer worktree"
    SCRATCHPAD_DIR="$ROOT_DIR/.scratchpad"
    # In layer worktree, ROOT_DIR IS the layers base (contains *_ws dirs/symlinks)
    LAYERS_BASE="$ROOT_DIR"
elif [[ "$ROOT_DIR" == *"/.workspace-worktrees/"* ]]; then
    WORKTREE_INFO="workspace worktree"
    SCRATCHPAD_DIR="$ROOT_DIR/.agent/scratchpad"
    # Workspace worktree has symlinked layers/main
    LAYERS_BASE="$ROOT_DIR/layers/main"
fi

REPORT_FILE="$SCRATCHPAD_DIR/test_report.md"
HISTORY_FILE="$SCRATCHPAD_DIR/test_history.csv"
SUMMARY_JSON="$SCRATCHPAD_DIR/test_summary.json"

mkdir -p "$SCRATCHPAD_DIR"
LOCK_FILE="$SCRATCHPAD_DIR/workspace.lock"

# Check for Lock (only in main workspace, worktrees are isolated)
if [ -z "$WORKTREE_INFO" ] && [ -f "$LOCK_FILE" ]; then
    echo "❌ Workspace is LOCKED by $(cat $LOCK_FILE)"
    exit 1
fi

# Initialize Report and JSON parts
{
    echo "# Test Report - $(date)"
    [ -n "$WORKTREE_INFO" ] && echo "**Context**: $WORKTREE_INFO"
    echo ""
    echo "| Layer | Result | Summary |"
    echo "|---|---|---|"
} > "$REPORT_FILE"

# Init CSV header if new
if [ ! -f "$HISTORY_FILE" ]; then
    echo "timestamp,layer,result" > "$HISTORY_FILE"
fi

# Load Layers (this also sets LAYERS array and LAYERS_BASE via env.sh)
if [ -f "$SCRIPT_DIR/env.sh" ]; then
    source "$SCRIPT_DIR/env.sh" > /dev/null
else
    # Fallback if env.sh missing
    LAYERS=("core") 
fi

echo "========================================"
echo "Starting Test Sequence"
[ -n "$WORKTREE_INFO" ] && echo "Context: $WORKTREE_INFO"
echo "========================================"

OVERALL_SUCCESS=true
TOTAL_FAILURES=0
FAILED_LAYERS=""
JSON_LAYERS_ARRAY=()
TIMESTAMP="$(date -Iseconds)"

for layer in "${LAYERS[@]}"; do
    LAYER_DIR="$LAYERS_BASE/${layer}_ws"
    LAYER_RESULT="skipped"
    LAYER_SUMMARY="No source"
    
    if [ -d "$LAYER_DIR/src" ]; then
        echo "----------------------------------------"
        echo "Testing Layer: $layer"
        

        # Safety Check: Never run in root
        if [ "$LAYER_DIR" == "$ROOT_DIR" ]; then
             echo "❌ Error: Layer path resolves to root directory. Skipping."
             continue
        fi

        cd "$LAYER_DIR" || continue
        
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

    # Append to CSV (quote all fields)
    echo "\"$TIMESTAMP\",\"$layer\",\"$LAYER_RESULT\"" >> "$HISTORY_FILE"

    # Construct JSON Object for this layer using Python for safety
    LAYER_JSON=$(python3 -c "import json, sys; print(json.dumps({'name': sys.argv[1], 'result': sys.argv[2], 'summary': sys.argv[3]}))" "$layer" "$LAYER_RESULT" "$LAYER_SUMMARY")
    JSON_LAYERS_ARRAY+=("$LAYER_JSON")
done

# Build JSON Output
JSON_STR="{ \"timestamp\": \"$TIMESTAMP\", \"overall_success\": $([ "$OVERALL_SUCCESS" = true ] && echo "true" || echo "false"), \"layers\": ["
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
