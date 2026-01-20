---
description: Adding a new repository to a workspace layer
---
// turbo-all
# Add Repository to Workspace Layer

This workflow guides you through adding a new Git repository to an existing or new workspace layer defined in `configs/`.

## Steps

1. **Identify the Layer**: Choose which `.repos` file in `configs/` to update (e.g., `configs/core.repos` or `configs/underlay.repos`). Create a new one if necessary.

2. **Verify Repository and Branch**: Before adding the config, verify that the repository and branch exist using `git ls-remote`.
   ```bash
   git ls-remote --heads <git_url> <branch_or_tag>
   ```
   *If the command returns nothing or an error, the repository or branch may not exist.*

3. **Update the config**: Add the repository entry to the `repositories` section of the chosen `.repos` file.
   ```yaml
   repositories:
     <repo_name>:
       type: git
       url: <git_url>
       version: <branch_or_tag>
   ```

4. **Register Layer (If New)**: If you created a new `.repos` file, the layer will be automatically registered when you run setup. The layer configuration is managed dynamically.

5. **Run Setup**: Execute the setup script for that layer to import the code.
   ```bash
   ./.agent/scripts/setup.sh <layer_name>
   ```

6. **Install Dependencies**: Use `rosdep` to install any missing system dependencies for the new code.
   ```bash
   rosdep install --from-paths workspaces/<layer_name>_ws/src --ignore-src -r -y
   ```

7. **Verify**: Check that the repository was cloned into `workspaces/<layer_name>_ws/src/<repo_name>`.
   ```bash
   ls workspaces/<layer_name>_ws/src/<repo_name>
   ```

8. **Build (Optional)**: Build the workspace to ensure the new package is compatible.
   ```bash
   cd workspaces/<layer_name>_ws
   colcon build --symlink-install
   ```
