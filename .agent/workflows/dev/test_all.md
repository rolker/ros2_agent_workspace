# Run All Tests

Execute tests across all workspace layers in the correct order.

## Steps

1.  **Run the Test Script**:
    ```bash
    ./.agent/scripts/test.sh
    ```

2.  **Analyze Report**:
    - The script outputs a summary table to stdout and saves it to `.agent/scratchpad/test_report.md`.
    - Check for `❌ Failed` layers.

3.  **Debug Failures**:
    - If a layer fails, navigate to `layers/<layer>_ws`.
    - View specific failure details:
      ```bash
      colcon test-result --verbose
      ```
    - Run individual test for faster debugging:
      ```bash
      colcon test --packages-select <package_name> --event-handlers console_direct+
      ```
