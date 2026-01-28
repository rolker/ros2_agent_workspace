---
name: Test Engineering
description: Develop, scaffold, and debug ROS 2 tests including GTest, PyTest, and launch_testing integration tests.
---

# Test Engineering Skill

Use this skill when asked to "write tests", "add test coverage", "create integration tests", "debug test failures", or "scaffold tests for ROS 2 packages".

## Scope

This skill covers ROS 2 test development:
- **Unit Tests**: GTest (C++) and PyTest (Python) for node logic
- **Integration Tests**: launch_testing for multi-node scenarios
- **Test Coverage Analysis**: Identifying gaps and planning test strategies
- **Debugging**: Troubleshooting complex ROS-specific test failures

## Test Types in ROS 2

### 1. Unit Tests (C++ with GTest)
**Purpose**: Test individual functions, classes, or components in isolation.

**When to Use**:
- Testing utility functions or algorithms
- Validating class behavior without ROS middleware
- Testing pure logic components

**Template**: See `templates/gtest_template.cpp`

### 2. Unit Tests (Python with PyTest)
**Purpose**: Test individual Python modules, functions, or classes.

**When to Use**:
- Testing Python node logic
- Validating utility functions
- Testing data processing pipelines

**Template**: See `templates/pytest_template.py`

### 3. Integration Tests (launch_testing)
**Purpose**: Test multi-node interactions, topic communication, and system behavior.

**When to Use**:
- Testing node communication patterns
- Validating publisher/subscriber interactions
- Testing service/action servers with clients
- End-to-end workflow validation

**Template**: See `templates/launch_test_template.py`

## Procedures

### 1. Identify Missing Test Coverage

**Trigger**: "What tests are missing?" or "Analyze test coverage"

**Steps**:
1. **Scan Package Structure**:
   - Check for `test/` directory existence
   - List existing test files
   - Identify nodes/libraries without corresponding tests

2. **Analyze Code**:
   - List public APIs (classes, functions) in C++ headers or Python modules
   - Identify publishers, subscribers, services, and actions
   - Check for complex logic that needs validation

3. **Gap Analysis**:
   - Compare code modules with test files
   - Identify untested nodes
   - Flag untested communication patterns (topics, services)
   - Note missing integration tests for multi-node scenarios

4. **Report**:
   ```
   Test Coverage Analysis for <package_name>:
   
   ✓ Tested Components:
     - component_a (test/test_component_a.cpp)
     - utils module (test/test_utils.py)
   
   ✗ Missing Tests:
     - node_b (no unit tests)
     - ServiceHandler class (untested API)
     - Integration: node_a → node_b communication
   
   Recommendation: 
     1. Add test/test_node_b.cpp for node_b unit tests
     2. Add test/test_integration.py for multi-node scenario
   ```

### 2. Scaffold New Tests

**Trigger**: "Create tests for X" or "Scaffold test file for Y"

**Steps**:

#### For C++ GTest:
1. Create `test/test_<component>.cpp` using template
2. Update `CMakeLists.txt`:
   ```cmake
   if(BUILD_TESTING)
     find_package(ament_cmake_gtest REQUIRED)
     
     ament_add_gtest(test_<component> test/test_<component>.cpp)
     target_link_libraries(test_<component> <library_name>)
     ament_target_dependencies(test_<component> rclcpp)
   endif()
   ```
3. Update `package.xml`:
   ```xml
   <test_depend>ament_cmake_gtest</test_depend>
   ```

#### For Python PyTest:
1. Create `test/test_<module>.py` using template
2. Update `setup.py`:
   ```python
   tests_require=['pytest'],
   ```
3. Update `package.xml`:
   ```xml
   <test_depend>python3-pytest</test_depend>
   ```

#### For launch_testing:
1. Create `test/test_<scenario>.py` using template
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

### 3. Write Mock Interfaces

**Trigger**: "Create a mock publisher" or "Mock this service"

**Purpose**: Simulate ROS 2 interfaces for isolated testing.

**Common Mock Patterns**:

#### Mock Publisher (for testing subscribers):
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MockPublisher(Node):
    def __init__(self):
        super().__init__('mock_publisher')
        self.publisher = self.create_publisher(String, '/test_topic', 10)
        
    def publish_test_message(self, data):
        msg = String()
        msg.data = data
        self.publisher.publish(msg)
```

#### Mock Service Server (for testing clients):
```python
from std_srvs.srv import Trigger

class MockServiceNode(Node):
    def __init__(self):
        super().__init__('mock_service')
        self.srv = self.create_service(
            Trigger, 
            'test_service', 
            self.handle_request
        )
        
    def handle_request(self, request, response):
        response.success = True
        response.message = 'Mock response'
        return response
```

#### Mock Subscriber (for testing publishers):
```python
class MockSubscriber(Node):
    def __init__(self):
        super().__init__('mock_subscriber')
        self.received_messages = []
        self.subscription = self.create_subscription(
            String,
            '/test_topic',
            self.callback,
            10
        )
        
    def callback(self, msg):
        self.received_messages.append(msg.data)
```

### 4. Debug Test Failures

**Trigger**: "Test X is failing" or "Debug this test"

**Debugging Strategy**:

#### Step 1: Reproduce Locally
```bash
# Build with tests enabled
colcon build --packages-select <package_name>

# Run specific test
colcon test --packages-select <package_name> --event-handlers console_direct+

# View detailed results
colcon test-result --verbose
```

#### Step 2: Identify Failure Type

**Build Failures**:
- Missing test dependencies in `package.xml`
- Incorrect CMake configuration
- Header/import errors

**Runtime Failures**:
- Node initialization issues
- Timeout waiting for messages
- Incorrect message types
- Service/topic name mismatches
- Race conditions

**Assertion Failures**:
- Logic errors in code under test
- Incorrect test expectations
- Data type mismatches

#### Step 3: ROS-Specific Debugging

**Check Communication**:
```python
# In launch_testing, verify topics exist
import subprocess
result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True)
print(result.stdout.decode())
```

**Increase Timeouts**:
```python
# For slow CI environments
proc_output.assertWaitFor(
    expected_output='Node started',
    timeout=30.0  # Increase from default 10s
)
```

**Add Debug Logging**:
```python
# In test nodes
self.get_logger().info(f'Received message: {msg.data}')
```

**Isolate Components**:
- Run test node manually: `ros2 run <package> <test_node>`
- Check topic data: `ros2 topic echo /test_topic`
- Verify service availability: `ros2 service list`

#### Step 4: Common Fixes

**Timeout Issues**:
- Increase test timeouts for CI environments
- Add explicit waits for node initialization
- Use `rclpy.spin_once()` to process callbacks

**Race Conditions**:
- Use event-based waiting mechanisms (e.g., `rclpy.spin_until_future_complete()`, `wait_for_service()`)
- Add synchronization barriers using ROS mechanisms
- Implement custom condition-based waiting loops instead of polling
- Avoid `time.sleep()` as it leads to flaky tests

**Message Not Received**:
- Verify QoS settings match between publisher/subscriber
- Check topic names (namespaces, spelling)
- Ensure nodes have time to discover each other

## Testing Workflow

### Standard Test Development Flow

1. **Identify Component**: Determine what needs testing
2. **Choose Test Type**: Unit, integration, or both
3. **Scaffold Test File**: Use appropriate template
4. **Write Test Cases**: Start with happy path, add edge cases
5. **Update Build Files**: CMakeLists.txt, setup.py, package.xml
6. **Build and Test**: `colcon build --packages-select <pkg> && colcon test --packages-select <pkg>`
7. **Debug Failures**: Use debugging procedures above
8. **Iterate**: Refine until all tests pass

### Best Practices

1. **Test One Thing**: Each test case should validate one specific behavior
2. **Use Descriptive Names**: `test_publisher_sends_correct_message_type()`
3. **Clean Setup/Teardown**: Initialize fresh state for each test
4. **Avoid Flaky Tests**: Don't rely on timing assumptions
5. **Test Edge Cases**: Empty inputs, null values, boundary conditions
6. **Mock External Dependencies**: Isolate the component under test
7. **Document Test Intent**: Add docstrings explaining what's being tested

## Quick Reference

### Run Tests
```bash
# All packages
colcon test

# Specific package
colcon test --packages-select <package_name>

# With verbose output
colcon test --packages-select <package_name> --event-handlers console_direct+

# Show results
colcon test-result --all
colcon test-result --verbose
```

### CMake Test Configuration
```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  
  # C++ unit test
  ament_add_gtest(test_my_component test/test_my_component.cpp)
  target_link_libraries(test_my_component my_library)
  
  # Integration test
  add_launch_test(test/test_integration.py)
endif()
```

### Python Test Discovery
Tests are discovered automatically if they:
- Are in a `test/` directory
- Have filenames matching `test_*.py` or `*_test.py`
- Contain functions matching `test_*()`

## Templates

All templates are available in the `templates/` directory:
- `gtest_template.cpp` - C++ unit test skeleton
- `pytest_template.py` - Python unit test skeleton
- `launch_test_template.py` - Integration test skeleton

## References

Additional documentation in `references/`:
- Links to official ROS 2 testing documentation
- Common testing patterns and recipes
- Troubleshooting guides

## Example Usage

**User**: "Add tests for the gps_driver package"

**Agent Response**:
1. Analyze the package structure
2. Identify untested components (GpsNode class, parsing functions)
3. Scaffold `test/test_gps_node.cpp` for unit tests
4. Scaffold `test/test_gps_integration.py` for integration tests
5. Update CMakeLists.txt and package.xml
6. Build and run tests to verify scaffolding
7. Report: "Created test scaffolding. Next steps: implement test logic for GPS parsing and node communication."
