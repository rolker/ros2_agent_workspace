---
trigger: always_on
---

Workspaces must be built using `colcon` from within their respective workspace directory (e.g., `layers/main/core_ws`). This ensures that `build`, `install`, and `log` folders are generated as siblings of the `src` folder, keeping layers isolated.
