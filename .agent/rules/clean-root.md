---
trigger: always_on
---

# Keep Workspace Root Clean

- **No Builds in Root**: Do NOT run `colcon build` (or similar build commands) from the root of the workspace. Always build within specific layer directories (e.g., `workspaces/core_ws`).
- **No Temporary Files**: Do NOT generate temporary files, logs, or artifacts in the workspace root.
- **Preferred Temp Location**: If you need to create temporary files that are not project artifacts, use `./ai_workspace` (create it if it doesn't exist) or the system `/tmp` directory.
