# Build ROS2 Workspaces

To build the workspaces, use the standard `colcon` workflow.

## Prerequisites

Before building, ensure ROS environment is sourced:

```bash
# Check if ROS commands available
if ! command -v colcon &> /dev/null; then
    echo "⚠️  ROS environment not sourced. Sourcing now..."
    source .agent/scripts/env.sh
fi
```

## Steps

1. **Determine Workspace**: Identify if you are building `underlay_ws`, a specific overlay (e.g., `core_ws`), or all of them.
2. **Navigate**: Go to the workspace root: `layers/<name>_ws`.
3. **Build Command**:
   Navigate to the workspace directory and run build:
   ```bash
   cd layers/<name>_ws
   colcon build --symlink-install
   ```
   *Note: This keeps build, install, and log directories contained within the workspace.*
4. **Error Handling**: 
   - If packages are missing dependencies, run:
     ```bash
     rosdep install --from-paths src --ignore-src -r -y
     ```
   - If CMake errors occur, check `package.xml` for missing dependencies.

## Verification
- **Check Summary**: Look for the final output: `Summary: <N> packages finished [s]`.
- **Identify Failures**: If the summary says `<M> packages failed`, search the log for `--- stderr` to identify the specific error.
- **Source**: Source the setup file: `source install/setup.bash`.
