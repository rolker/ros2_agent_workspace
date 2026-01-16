# Check Workspace Status

Workflow to identify modified files in the root configuration or in any of the ROS2 source packages.

## Use Case
Run this before starting a task to ensure a clean slate, or after a task to verify what has changed before committing.

## Steps

1.  **Run the Report Script**:
    ```bash
    ./scripts/status_report.sh
    ```

2.  **Interpret Output**:
    - **ROOT REPOSITORY**: Shows changes to `configs/`, `scripts/`, or agent artifacts.
        - *Action*: If unexpected changes appear, verify if they should be committed.
    - **ROS2 WORKSPACES**: Shows changes in the underlying source code (e.g., `Project11`, `marine_ais`).
        - *Action*: `vcs status` output usually shows the branch and modification state.
        - `S`: Staged
        - `M`: Modified
        - `?`: Untracked

3.  **Resolve Issues**:
    - If a repo is dirty (`M`), go to that directory and use `git diff` to inspect.
    - If a repo is on the wrong branch, use `vcs custom --args checkout <branch>`.
