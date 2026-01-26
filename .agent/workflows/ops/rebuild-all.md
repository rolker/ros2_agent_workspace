---
description: Clean all build artifacts then build all layers
---
// turbo-all
# Rebuild All Workspaces

This workflow combines the cleanup of build artifacts and the sequential building of all workspace layers using the unified build script.

## Steps

1. **Clean Build Artifacts**: Remove `build`, `install`, and `log` directories from all workspace layers.
   ```bash
   rm -rf workspaces/*_ws/build workspaces/*_ws/install workspaces/*_ws/log
   ```

2. **Run Build Script**: Execute the consolidated build script.
   ```bash
   ./.agent/scripts/build.sh
   ```

## Verification
- Verification is automatic via the script's reporting mechanism.
