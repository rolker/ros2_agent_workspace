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
To safely remove directories in `src/` that are no longer tracked in your `.repos` files:

```bash
# Identify untracked repositories by comparing src/ with the .repos config
source .agent/scripts/env.sh
for layer in "${LAYERS[@]}"; do
    [ -z "$layer" ] && continue
    WORKSPACE_DIR="workspaces/${layer}_ws"
    SRC_DIR="$WORKSPACE_DIR/src"
    REPO_CONFIG="configs/${layer}.repos"
    
    if [ -d "$SRC_DIR" ] && [ -f "$REPO_CONFIG" ]; then
        # Extract tracked repo names from config
        grep "^  [a-zA-Z0-9_\-]*:" "$REPO_CONFIG" | sed 's/  \(.*\):/\1/' | sort > /tmp/tracked.txt
        # List actual directories in src
        ls -d "$SRC_DIR"/*/ 2>/dev/null | xargs -n 1 basename | sort > /tmp/current.txt
        
        UNTRACKED=$(comm -23 /tmp/current.txt /tmp/tracked.txt)
        
        if [ ! -z "$UNTRACKED" ]; then
            echo "----------------------------------------"
            echo "Found untracked directories in $SRC_DIR:"
            echo "$UNTRACKED"
            echo "----------------------------------------"
            read -p "Are you sure you want to remove these directories? (y/n) " -n 1 -r
            echo
            if [[ $REPLY =~ ^[Yy]$ ]]; then
                for d in $UNTRACKED; do
                    rm -rf "$SRC_DIR/$d"
                done
                echo "Removed."
            else
                echo "Skipped."
            fi
        fi
    fi
done
```

### 3. Full Reset (Optional)
If you want to completely re-import everything and ensure a clean state:
```bash
# 1. Clean everything
rm -rf workspaces/*/build workspaces/*/install workspaces/*/log

# 2. Run setup for all layers to ensure configs match local src
source .agent/scripts/env.sh
for layer in "${LAYERS[@]}"; do
    ./.agent/scripts/setup.sh "$layer"
done
```

## Verification
- Run `ls workspaces` to ensure build directories are gone.
- Run `ls workspaces/*_ws/src` to verify only configured repos remain.
