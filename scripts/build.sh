#!/bin/bash
# scripts/build.sh
# Unified build script for ROS2 Agent Workspace
#
# Usage: ./scripts/build.sh
#
# This script will:
# 1. Check for workspace locks (multi-agent coordination)
# 2. Build all workspace layers in dependency order
# 3. Generate a detailed build report in ai_workspace/build_report.md
# 4. Source each layer after successful build for cascading overlays
#
# The build order follows the layer hierarchy defined in scripts/env.sh:
#   underlay -> core -> platforms -> sensors -> simulation -> ui
#
# Exit codes:
#   0 - All layers built successfully
#   1 - Build failed in one or more layers

# Exit on any error to prevent cascading failures in strict mode, 
# though we want to handle build failures gracefully for reporting.
# set -u removed to allow sourcing upstream setup.bash which has unbound variables

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
REPORT_FILE="$ROOT_DIR/ai_workspace/build_report.md"

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
{
    echo "# Build Report - $(date)"
    echo ""
    echo "| Layer | Packages (Total/OK) | Output (Warnings/Errors) | Status |"
    echo "|---|---|---|---|"
} > "$REPORT_FILE"

# Load Layer Definitions
source "$SCRIPT_DIR/env.sh" > /dev/null

echo "========================================"
echo "Starting Build Sequence"
echo "========================================"

TOTAL_FAILED=0
FAILED_LAYERS=""

for layer in "${LAYERS[@]}"; do
    WORKSPACE_DIR="$ROOT_DIR/workspaces/${layer}_ws"
    
    if [ ! -d "$WORKSPACE_DIR/src" ]; then
        echo "Skipping layer $layer (no src directory)"
        echo "| $layer | 0 | - | ⏭️ Skipped |" >> "$REPORT_FILE"
        continue
    fi

    echo "----------------------------------------"
    echo "Building Layer: $layer"
    echo "----------------------------------------"

    # Go to workspace
    cd "$WORKSPACE_DIR" || exit 1

    # Run Colcon Build
    # We rely on the log configuration to write to log/latest_build/events.log
    # --symlink-install is standard for dev
    # We allow stdout to flow to the user's terminal
    
    colcon build --symlink-install

    BUILD_RC=$?
    
    # Generate Report Row
    # python3 scripts/build_report_generator.py ...
    # We need to be careful about where the log is. default is install/../log/latest_build
    LOG_DIR="$WORKSPACE_DIR/log/latest_build"
    
    # Run the generator and append to report
    if [ -d "$LOG_DIR" ]; then
        python3 "$SCRIPT_DIR/build_report_generator.py" --log-dir "$LOG_DIR" --layer-name "$layer" >> "$REPORT_FILE"
    else
        echo "| $layer | - | - | ⚠️ No Log |" >> "$REPORT_FILE"
    fi

    # Check result
    if [ $BUILD_RC -eq 0 ]; then
        # Sourcing ensures the next layer can see this one
        if [ -f "install/setup.bash" ]; then
            source "install/setup.bash"
        fi
    else
        echo "!! Build Failed for layer: $layer"
        TOTAL_FAILED=$((TOTAL_FAILED + 1))
        FAILED_LAYERS="$FAILED_LAYERS $layer"
        # We stop early on failure usually
        break
    fi
done

echo "========================================"
echo "Build Summary"
echo "========================================"
cat "$REPORT_FILE"

if [ $TOTAL_FAILED -eq 0 ]; then
    echo "All layers built successfully."
    exit 0
else
    echo "Build failed in layers: $FAILED_LAYERS"
    exit 1
fi
