# ROS 2 Testing References

This document provides links and references to official ROS 2 testing documentation and resources.

## Official Documentation

### General Testing
- **ROS 2 Testing Guide**: https://docs.ros.org/en/rolling/Tutorials/Intermediate/Testing/Testing-Main.html
- **Quality Guide (Testing)**: https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#testing

### Unit Testing
- **ament_cmake_gtest**: https://github.com/ament/ament_cmake/tree/rolling/ament_cmake_gtest
  - GTest integration for C++ packages
  - CMake functions for adding tests

- **ament_cmake_pytest**: https://github.com/ament/ament_cmake/tree/rolling/ament_cmake_pytest
  - PyTest integration for Python packages
  - Test discovery and execution

### Integration Testing
- **launch_testing**: https://github.com/ros2/launch/tree/rolling/launch_testing
  - Framework for testing launch files and node interactions
  - Process output validation
  - Multi-node test scenarios

- **launch_testing_ament_cmake**: https://github.com/ros2/launch/tree/rolling/launch_testing_ament_cmake
  - CMake integration for launch_testing
  - Test registration and discovery

- **launch_testing_ros**: https://github.com/ros2/launch_ros/tree/rolling/launch_testing_ros
  - ROS-specific testing utilities
  - Topic, service, and action testing helpers

## Testing Tools

### Coverage Analysis
- **lcov**: http://ltp.sourceforge.net/coverage/lcov.php
  - Code coverage for C++
  - Generate HTML coverage reports

- **pytest-cov**: https://pytest-cov.readthedocs.io/
  - Coverage plugin for pytest
  - Python code coverage analysis

### Mock and Stub Libraries
- **google/googletest**: https://github.com/google/googletest
  - Google's C++ test framework
  - Includes GMock for mocking

- **unittest.mock**: https://docs.python.org/3/library/unittest.mock.html
  - Python's built-in mocking library
  - Create mock objects for testing

## Best Practices

### ROS 2 Specific
1. **Test in Isolation**: Use `--packages-select` to test individual packages
2. **Use Fixtures**: Set up and tear down ROS context properly
3. **Timeout Handling**: Account for CI environments being slower
4. **QoS Settings**: Match QoS between test nodes and nodes under test
5. **Node Lifecycle**: Properly initialize and shutdown nodes in tests

### General Testing
1. **AAA Pattern**: Arrange, Act, Assert
2. **Single Responsibility**: One test per behavior
3. **Descriptive Names**: Test names should describe what they test
4. **Independence**: Tests should not depend on each other
5. **Repeatability**: Tests should produce consistent results

## Common Testing Patterns

### Testing Publishers
```python
# Create a mock subscriber to capture messages
class MockSubscriber:
    def __init__(self, node, topic, msg_type):
        self.messages = []
        self.sub = node.create_subscription(
            msg_type, topic, self.callback, 10
        )
    
    def callback(self, msg):
        self.messages.append(msg)
```

### Testing Subscribers
```python
# Create a test publisher to send messages
def test_subscriber():
    test_node = Node('test_node')
    pub = test_node.create_publisher(String, '/test_topic', 10)
    
    # Give time for subscription to be established
    time.sleep(0.5)
    
    # Publish test message
    msg = String()
    msg.data = 'test'
    pub.publish(msg)
    
    # Verify subscriber received and processed it
    # ... assertion logic ...
```

### Testing Services
```python
# Test service server
def test_service_server():
    node = Node('test_node')
    client = node.create_client(Trigger, '/test_service')
    
    assert client.wait_for_service(timeout_sec=5.0)
    
    request = Trigger.Request()
    future = client.call_async(request)
    
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    
    response = future.result()
    assert response.success
```

## Troubleshooting

### Common Issues

#### Test Discovery Failures
- **Issue**: Tests not found by colcon
- **Solution**: Ensure test files match naming pattern (`test_*.py` or `*_test.py`)
- **Solution**: Verify `CMakeLists.txt` or `setup.py` is configured correctly

#### Timeout Failures
- **Issue**: Tests timeout waiting for messages/services
- **Solution**: Increase timeout values for CI environments
- **Solution**: Add debug logging to identify where timeout occurs
- **Solution**: Verify topic/service names are correct

#### Race Conditions
- **Issue**: Tests pass/fail inconsistently
- **Solution**: Add explicit synchronization barriers
- **Solution**: Use event-based waiting instead of sleep()
- **Solution**: Ensure proper node discovery time

#### Import Errors
- **Issue**: Cannot import package under test
- **Solution**: Source the workspace: `source install/setup.bash`
- **Solution**: Verify package is built: `colcon build --packages-select <pkg>`
- **Solution**: Check PYTHONPATH includes install directory

### Debugging Commands

```bash
# Run tests with verbose output
colcon test --packages-select <pkg> --event-handlers console_direct+

# Show detailed test results
colcon test-result --verbose

# Run single test file (Python)
python3 -m pytest test/test_file.py -v

# Run single test case (Python)
python3 -m pytest test/test_file.py::TestClass::test_method -v

# Run with coverage (Python)
python3 -m pytest test/ --cov=<package_name> --cov-report=html

# Build with tests enabled (default)
colcon build --packages-select <pkg>

# Build without tests (faster)
colcon build --packages-select <pkg> --cmake-args -DBUILD_TESTING=OFF
```

## Example Packages

Good examples of ROS 2 testing in official packages:
- **rclcpp**: https://github.com/ros2/rclcpp/tree/rolling/rclcpp/test
- **rclpy**: https://github.com/ros2/rclpy/tree/rolling/rclpy/test
- **demo_nodes_cpp**: https://github.com/ros2/demos/tree/rolling/demo_nodes_cpp/test
- **demo_nodes_py**: https://github.com/ros2/demos/tree/rolling/demo_nodes_py/test

## Additional Resources

- **ROS 2 Design**: http://design.ros2.org/
  - Architecture and design decisions
  - Testing philosophy and requirements

- **ROS Discourse**: https://discourse.ros.org/
  - Community questions and answers
  - Testing tips and tricks

- **GitHub Issues**: Search for "test" in ROS 2 repositories
  - Real-world testing challenges and solutions
  - Bug reports with test cases
