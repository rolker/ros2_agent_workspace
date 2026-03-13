---
name: test-engineering
description: Scaffold, debug, and analyze test coverage for ROS 2 packages. Supports GTest, PyTest, and launch_testing.
---

# Test Engineering

## Usage

```
/test-engineering [<package-name>]
```

If no package name is given, operate on the package in the current directory.

## Overview

**Lifecycle position**: Utility — use after `audit-project` flags test gaps,
or when asked to "write tests", "add test coverage", "debug test failures",
or "scaffold tests".

Covers the full test development lifecycle for ROS 2 packages: identifying
coverage gaps, scaffolding test files, writing mock interfaces, and debugging
failures. References existing templates rather than embedding them.

## Test Types

| Type | Framework | When to Use | Template |
|------|-----------|-------------|----------|
| C++ unit tests | GTest | Testing functions, classes, algorithms in isolation | `.agent/templates/testing/gtest_template.cpp` |
| Python unit tests | PyTest | Testing Python node logic, utilities, data processing | `.agent/templates/testing/pytest_template.py` |
| Integration tests | launch_testing | Testing multi-node interactions, topic communication | `.agent/templates/testing/launch_test_template.py` |

## Procedures

### 1. Identify missing test coverage

**Trigger**: "What tests are missing?" or "Analyze test coverage"

1. **Scan package structure**:
   - Check for `test/` directory
   - List existing test files
   - Identify nodes/libraries without corresponding tests

2. **Analyze code**:
   - List public APIs (classes, functions) in C++ headers or Python modules
   - Identify publishers, subscribers, services, and actions
   - Check for complex logic that needs validation

3. **Gap analysis**:
   - Compare code modules with test files
   - Identify untested nodes
   - Flag untested communication patterns (topics, services)
   - Note missing integration tests for multi-node scenarios

4. **Report**:

```
Test Coverage Analysis for <package_name>:

Tested:
  - component_a (test/test_component_a.cpp)
  - utils module (test/test_utils.py)

Missing:
  - node_b (no unit tests)
  - ServiceHandler class (untested API)
  - Integration: node_a -> node_b communication

Recommendation:
  1. Add test/test_node_b.cpp for unit tests
  2. Add test/test_integration.py for multi-node scenario
```

### 2. Scaffold new tests

**Trigger**: "Create tests for X" or "Scaffold test file"

Read the appropriate template from `.agent/templates/testing/` and adapt it:

#### C++ GTest

1. Create `test/test_<component>.cpp` using `gtest_template.cpp`
2. Update `CMakeLists.txt`:
   ```cmake
   if(BUILD_TESTING)
     find_package(ament_cmake_gtest REQUIRED)
     ament_add_gtest(test_<component> test/test_<component>.cpp)
     target_link_libraries(test_<component> <library_name>)
   endif()
   ```
3. Update `package.xml`:
   ```xml
   <test_depend>ament_cmake_gtest</test_depend>
   ```

#### Python PyTest

1. Create `test/test_<module>.py` using `pytest_template.py`
2. Update `package.xml`:
   ```xml
   <test_depend>python3-pytest</test_depend>
   ```

#### Integration tests (launch_testing)

1. Create `test/test_<scenario>.py` using `launch_test_template.py`
2. Update `CMakeLists.txt`:
   ```cmake
   if(BUILD_TESTING)
     find_package(launch_testing_ament_cmake REQUIRED)
     add_launch_test(test/test_<scenario>.py)
   endif()
   ```
3. Update `package.xml`:
   ```xml
   <test_depend>launch_testing_ament_cmake</test_depend>
   <test_depend>launch_testing_ros</test_depend>
   ```

For integration tests involving launch files, consider using
`ros2launch_session` for clean lifecycle management. See
`.agent/knowledge/launch_tooling.md`.

### 3. Debug test failures

**Trigger**: "Test X is failing" or "Debug this test"

#### Reproduce locally

**In a layer worktree** (preferred), use the generated convenience scripts:

```bash
./<layer>_ws/build.sh <package_name>
./<layer>_ws/test.sh <package_name>
```

**In the main workspace**, source the environment and use colcon directly:

```bash
source .agent/scripts/setup.bash && cd layers/main/<layer>_ws && \
  colcon test --packages-select <package_name> --event-handlers console_direct+ && \
  colcon test-result --verbose
```

#### Identify failure type

| Type | Symptoms | Common Fix |
|------|----------|------------|
| Build failure | Missing test dependencies, CMake errors | Add `<test_depend>` to `package.xml` |
| Timeout | Test hangs, "timeout" in output | Increase timeout, check for deadlocks |
| Assertion failure | Expected vs actual mismatch | Fix logic or update expectations |
| Communication failure | Topics not received, services unavailable | Check QoS settings, topic names, discovery time |
| Race condition | Flaky — passes sometimes, fails others | Use event-based waits instead of `time.sleep()` |

#### Common fixes

- **Timeout**: Increase test timeouts for CI environments. Use
  `rclpy.spin_until_future_complete()` with explicit timeouts.
- **Race conditions**: Use `wait_for_service()`, condition-based waits, or
  `rclpy.spin_once()` — never `time.sleep()`.
- **Message not received**: Verify QoS settings match between
  publisher/subscriber. Check topic names including namespaces.
- **Discovery timing**: Add explicit waits for node discovery in integration
  tests. Nodes need time to find each other.

### 4. Write mock interfaces

**Trigger**: "Create a mock publisher" or "Mock this service"

Common patterns for isolated testing:

- **Mock publisher** (for testing subscribers): Create a node that publishes
  known test data on the expected topic.
- **Mock subscriber** (for testing publishers): Create a node that records
  received messages for assertion.
- **Mock service server** (for testing clients): Create a node that returns
  predetermined responses.

Keep mocks minimal — just enough to test the component under test.

## References

- `.agent/templates/testing/gtest_template.cpp` — C++ unit test skeleton
- `.agent/templates/testing/pytest_template.py` — Python unit test skeleton
- `.agent/templates/testing/launch_test_template.py` — Integration test skeleton
- `.agent/knowledge/launch_tooling.md` — `ros2launch_session` API for
  integration tests
- `.agent/knowledge/documentation_verification.md` — Command cookbook for
  finding publishers, subscribers, parameters (useful for test planning)

## Guidelines

- **Test one thing** — each test case validates one specific behavior.
- **Descriptive names** — `test_publisher_sends_correct_message_type()`.
- **Clean setup/teardown** — fresh state for each test.
- **No flaky tests** — don't rely on timing; use event-based synchronization.
- **Test edge cases** — empty inputs, boundary conditions, error states.
- **Mock external dependencies** — isolate the component under test.
- **Document test intent** — add docstrings explaining what's being tested
  and why.
