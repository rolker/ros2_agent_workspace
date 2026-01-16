# Review ROS2 Code

Perform a code review on a package to ensure quality and adherence to ROS2 standards.

## Steps

1. **Linter Check**:
   - Run `ament_lint` if available: `colcon test --packages-select <package_name> --ctest-args -R lint`
   - Check against `clang-format` or `flake8` standards.

2. **Best Practices Check**:
   - **Node Composition**: Verify nodes are created as components (inherit from `rclcpp::Node`) and are composable, rather than standalone `main()` functions where possible.
   - **Parameters**: Check that parameters are declared and used correctly.
   - **Lifecycle**: For driver-like nodes, check if `rclcpp_lifecycle::LifecycleNode` is used.
   - **Logging**: Ensure `RCLCPP_INFO` (or `get_logger()`) is used instead of `std::cout`.

3. **Report**:
   - Summarize findings.
   - Suggest refactors for "modern" ROS2 patterns if legacy ROS1 patterns are found.
