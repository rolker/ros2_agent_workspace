---
description: Clean build artifacts and prune removed repositories
---
// turbo-all
# Workspace Cleanup and Pruning

This workflow cleans up build artifacts and removes repository directories that have been deleted from the `.repos` configurations.

## Steps

### 1. Clean Build Artifacts
Run this to remove all build, install, and log directories across all workspace layers.
```bash
rm -rf workspaces/*_ws/build workspaces/*_ws/install workspaces/*_ws/log
```

### 2. Prune Removed Repositories
To remove directories in `src/` that are no longer tracked in your `.repos` files, you can use the following approach for each layer:

```bash
# Example for underlay layer
cd workspaces/underlay_ws/src
vcs status | grep "^?" | awk '{print $2}' | xargs rm -rf
cd - > /dev/null
```

*Note: `vcs status` marks untracked directories with `?`.*

### 3. Full Reset (Optional)
If you want to completely re-import everything and ensure a clean state:
```bash
# 1. Clean everything
rm -rf workspaces/*/build workspaces/*/install workspaces/*/log

# 2. Run setup for all layers to ensure configs match local src
source scripts/env.sh
for layer in "${LAYERS[@]}"; do
    ./scripts/setup.sh "$layer"
done
```

## Verification
- Run `ls workspaces` to ensure build directories are gone.
- Run `ls workspaces/*_ws/src` to verify only configured repos remain.
