---
description: Build all workspace layers in the correct order
---
// turbo-all
# Build All Workspace Layers

This workflow dynamically builds all workspace layers in the order defined in `scripts/env.sh`. It handles "from scratch" builds by sourcing each layer as soon as it is built.

## Steps

1. **Build all layers**: Run the following logic to iterate through all defined layers.
   ```bash
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
           # Capture output to find package counts if needed, or rely on exit code
           if colcon build --symlink-install; then
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
               # Continue to next layer or exit? 
               # Given dependency chain, we should probably stop but show what happened.
               break
           fi
       else
           echo "Skipping layer $layer (source directory not found)."
       fi
   done

   echo "========================================"
   echo "BUILD ALL SUMMARY"
   echo "========================================"
   if [ $TOTAL_FAILED -eq 0 ]; then
       echo "All layers built successfully."
   else
       echo "Failed layers:$FAILED_PACKAGES"
       echo "Please check the logs for the specific failed packages within those layers."
   fi
   ```

## Verification
- Check that each workspace now has `build`, `install`, and `log` directories.
- Run `source scripts/env.sh` to load the final entire workspace chain.
