# Manage ROS2 Tests

Workflow for adding, updating, or removing tests.

## Steps

1. **Identify Missing Tests**:
   - Check if `test/` directory exists.
   - Look for standard tests: `copyright`, `flake8`, `pep257` (Python) or `lint` (C++).

2. **Add Tests**:
   - **Unit Tests**:
     - *Python*: Add `pytest` files in `test/test_*.py`.
     - *C++*: Add `gtest` files in `test/`. Modify `CMakeLists.txt` to include `ament_add_gtest`.
   - **Integration Tests**:
     - Add launch tests using `launch_testing` to verify node startup and communication.

3. **Update Tests**:
   - If API changes, update corresponding test signatures.
   - Ensure tests pass locally with `colcon test`.

4. **Remove Tests**:
   - If code is deprecated, remove associated tests.
   - Disable flaky tests if they cannot be fixed immediately (comment out with TODO).
