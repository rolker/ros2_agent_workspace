---
description: Clean all build artifacts then build all layers
---
// turbo-all
# Rebuild All Workspaces

This workflow combines the cleanup of build artifacts and the sequential building of all workspace layers. It is useful for ensuring a completely fresh build state.

## Steps

1. **Clean Build Artifacts**: Remove `build`, `install`, and `log` directories from all workspace layers.
   ```bash
   rm -rf workspaces/*_ws/build workspaces/*_ws/install workspaces/*_ws/log
   ```

2. **Build All Layers**: Iterate through all defined layers, building them in order and sourcing the environment incrementally.
   ```bash
   # 0. Initialize Report
   REPORT_FILE="ai_workspace/build_report.md"
   mkdir -p ai_workspace
   echo "# Build Report - $(date)" > "$REPORT_FILE"
   echo "" >> "$REPORT_FILE"
   echo "| Layer | Packages | Output (Warnings/Errors) | Status |" >> "$REPORT_FILE"
   echo "|---|---|---|---|" >> "$REPORT_FILE"

   # 1. Load the layer definitions
   source scripts/env.sh
   
   TOTAL_FINISHED=0
   TOTAL_FAILED=0
   FAILED_PACKAGES=""

   # 2. Iterate and build
   for layer in "${LAYERS[@]}"; do
       WORKSPACE_DIR="workspaces/${layer}_ws"
       
       if [ -d "$WORKSPACE_DIR/src" ]; then
           echo "----------------------------------------"
           echo "Building layer: $layer"
           echo "----------------------------------------"
           
           # Navigate and build
           cd "$WORKSPACE_DIR"
           
           OUTPUT_FILE="/tmp/colcon_output_${layer}.txt"
           
           # Build with output capture
           # set -o pipefail ensures we get the exit code of colcon, not tee
           set -o pipefail
           if colcon build --symlink-install 2>&1 | tee "$OUTPUT_FILE"; then
               BUILD_RESULT=0
               STATUS="✅ Success"
           else
               BUILD_RESULT=1
               STATUS="❌ Failed"
           fi
           set +o pipefail
           
           # Parse Report Data
           FINISHED_COUNT=$(grep "Finished <<<" "$OUTPUT_FILE" | wc -l)
           # Extract package names with stderr output (warnings/errors)
           STDERR_PKGS=$(grep "stderr output:" "$OUTPUT_FILE" | sed 's/.*stderr output: //' | tr '\n' ' ')
           [ -z "$STDERR_PKGS" ] && STDERR_PKGS="-"
           
           # Append to report
           echo "| $layer | $FINISHED_COUNT | $STDERR_PKGS | $STATUS |" >> "$REPORT_FILE"

           if [ $BUILD_RESULT -eq 0 ]; then
               # 3. Source the successful build so the next layer can see it
               if [ -f "install/setup.bash" ]; then
                   source "install/setup.bash"
               fi
               cd - > /dev/null
           else
               echo "Error: Build failed for layer $layer."
               TOTAL_FAILED=$((TOTAL_FAILED + 1))
               FAILED_PACKAGES="$FAILED_PACKAGES $layer"
               cd - > /dev/null
               break
           fi
       else
           echo "Skipping layer $layer (source directory not found)."
           echo "| $layer | 0 | - | ⏭️ Skipped |" >> "$REPORT_FILE"
       fi
   done

   echo "========================================"
   echo "REBUILD ALL SUMMARY"
   echo "========================================"
   echo "Build report generated at: $REPORT_FILE"
   cat "$REPORT_FILE"
   
   if [ $TOTAL_FAILED -eq 0 ]; then
       echo "All layers built successfully."
   else
       echo "Failed layers:$FAILED_PACKAGES"
       echo "Please review the errors above and the report."
   fi
   ```
