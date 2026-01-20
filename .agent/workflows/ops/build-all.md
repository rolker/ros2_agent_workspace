---
description: Build all workspace layers in the correct order
---
// turbo-all
# Build All Workspace Layers

This workflow dynamically builds all workspace layers in the correct order using the unified build script.

## Steps

1. **Run Build Script**: Execute the consolidated build script.
   ```bash
   ./.agent/scripts/build.sh
   ```

## Verification
- Review the summary table printed at the end of the script execution.
- Check `ai_workspace/build_report.md` for a persistent record.
