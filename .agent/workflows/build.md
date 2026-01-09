# Build ROS2 Workspaces

To build the workspaces, use the standard `colcon` workflow.

## Steps

1. **Determine Workspace**: Identify if you are building `underlay_ws`, a specific overlay (e.g., `core_ws`), or all of them.
2. **Navigate**: Go to the workspace root: `workspaces/<name>_ws`.
3. **Build Command**:
   ```bash
   colcon build --symlink-install
   ```
   *Note: Using `--symlink-install` allows changes in Python scripts and launch files to take effect without rebuilding.*
4. **Error Handling**: 
   - If packages are missing dependencies, run:
     ```bash
     rosdep install --from-paths src --ignore-src -r -y
     ```
   - If CMake errors occur, check `package.xml` for missing dependencies.

## Verification
- Check for `Summary: <N> packages finished [s]`.
- Source the setup file: `source install/setup.bash`.
