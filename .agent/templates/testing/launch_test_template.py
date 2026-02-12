#!/usr/bin/env python3
"""
Integration test using launch_testing.

This test launches multiple ROS 2 nodes and validates their interactions.
Replace placeholders with actual node names, topics, and test logic.
"""

import unittest
import pytest

import launch_testing
import launch_testing.actions
import launch_testing.markers
from launch import LaunchDescription
from launch_ros.actions import Node

import rclpy
from rclpy.node import Node as RclpyNode
from std_msgs.msg import String  # Replace with your message type


# This function generates the test launch description
@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """
    Generate the launch description for the integration test.

    Returns:
        LaunchDescription: Launch description with nodes to test
    """
    # Define the nodes to launch for testing
    node_under_test = Node(
        package="<package_name>",
        executable="<node_executable>",
        name="node_under_test",
        output="screen",
        parameters=[
            {
                # Add node parameters here
                # 'param_name': 'param_value',
            }
        ],
    )

    # Optional: Launch additional nodes for integration testing
    # helper_node = Node(
    #     package='<package_name>',
    #     executable='<helper_executable>',
    #     name='helper_node',
    #     output='screen',
    # )

    return LaunchDescription(
        [
            node_under_test,
            # helper_node,
            # This action tells launch_testing to start the test
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestNodeStartup(unittest.TestCase):
    """Test that nodes start up correctly."""

    def test_node_starts(self, proc_info, proc_output):
        """
        Test that the node starts without errors.

        Args:
            proc_info: Process info fixture
            proc_output: Process output fixture
        """
        # Wait for specific output indicating successful startup
        proc_output.assertWaitFor(
            "Node started", timeout=10.0, stream="stdout"  # Replace with actual startup message
        )


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    """Test process output after shutdown."""

    def test_exit_codes(self, proc_info):
        """
        Test that all processes exited cleanly.

        Args:
            proc_info: Process info fixture
        """
        # Check that all processes exited with code 0
        launch_testing.asserts.assertExitCodes(proc_info)


class TestNodeCommunication(unittest.TestCase):
    """Test communication between nodes."""

    @classmethod
    def setUpClass(cls):
        """Set up test class - runs once before all tests."""
        cls._rclpy_initialized_here = False
        if not rclpy.ok():
            rclpy.init()
            cls._rclpy_initialized_here = True

    @classmethod
    def tearDownClass(cls):
        """Clean up test class - runs once after all tests."""
        if getattr(cls, "_rclpy_initialized_here", False) and rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        """Set up each test - runs before each test method."""
        self.node = RclpyNode("test_node")
        self.received_messages = []

    def tearDown(self):
        """Clean up each test - runs after each test method."""
        self.node.destroy_node()

    def test_topic_published(self):
        """
        Test that the node publishes to the expected topic.

        This test subscribes to a topic and verifies messages are received.
        """
        # Create a subscription to the topic under test
        subscription = self.node.create_subscription(
            String,  # Replace with your message type
            "/test_topic",  # Replace with actual topic name
            self.message_callback,
            10,
        )

        # Spin the node to process callbacks
        # Wait for messages with a timeout
        timeout_sec = 5.0
        start_time = self.node.get_clock().now()

        while len(self.received_messages) == 0:
            rclpy.spin_once(self.node, timeout_sec=0.1)

            # Check timeout
            if (self.node.get_clock().now() - start_time).nanoseconds / 1e9 > timeout_sec:
                break

        # Assert that we received at least one message
        self.assertGreater(len(self.received_messages), 0, "No messages received on /test_topic")

        # Clean up
        self.node.destroy_subscription(subscription)

    def message_callback(self, msg):
        """
        Callback for received messages.

        Args:
            msg: Received message
        """
        self.received_messages.append(msg)

    def test_message_content(self):
        """
        Test the content of published messages.

        Verify that messages contain expected data.
        """
        # Similar to test_topic_published, but also check message content
        subscription = self.node.create_subscription(
            String, "/test_topic", self.message_callback, 10
        )

        # Wait for message
        timeout_sec = 5.0
        start_time = self.node.get_clock().now()

        while len(self.received_messages) == 0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if (self.node.get_clock().now() - start_time).nanoseconds / 1e9 > timeout_sec:
                break

        # Verify message content
        self.assertGreater(len(self.received_messages), 0)

        # Example: Check message data
        # first_msg = self.received_messages[0]
        # self.assertEqual(first_msg.data, 'expected_value')

        self.node.destroy_subscription(subscription)

    def test_service_available(self):
        """
        Test that a service is available.

        Verify that the node provides expected services.
        """
        from std_srvs.srv import Trigger  # Replace with your service type

        # Create a service client
        client = self.node.create_client(Trigger, "/test_service")

        # Wait for service to become available
        service_available = client.wait_for_service(timeout_sec=5.0)

        self.assertTrue(service_available, "Service /test_service not available")

        # Clean up
        self.node.destroy_client(client)

    def test_service_call(self):
        """
        Test calling a service and verifying the response.

        Make a service call and check the result.
        """
        from std_srvs.srv import Trigger

        client = self.node.create_client(Trigger, "/test_service")

        # Wait for service
        self.assertTrue(client.wait_for_service(timeout_sec=5.0))

        # Create and send request
        request = Trigger.Request()
        future = client.call_async(request)

        # Wait for response
        timeout_sec = 5.0
        start_time = self.node.get_clock().now()

        while not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if (self.node.get_clock().now() - start_time).nanoseconds / 1e9 > timeout_sec:
                break

        self.assertTrue(future.done(), "Service call timed out")

        # Verify response
        response = future.result()
        self.assertIsNotNone(response, "Service response is None")
        # self.assertTrue(response.success)
        # self.assertEqual(response.message, 'expected_message')

        self.node.destroy_client(client)


class TestNodeParameters(unittest.TestCase):
    """Test node parameter handling."""

    @classmethod
    def setUpClass(cls):
        """Set up test class."""
        cls._rclpy_initialized_here = False
        if not rclpy.ok():
            rclpy.init()
            cls._rclpy_initialized_here = True

    @classmethod
    def tearDownClass(cls):
        """Clean up test class."""
        if getattr(cls, "_rclpy_initialized_here", False) and rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        """Set up each test."""
        self.node = RclpyNode("test_node")

    def tearDown(self):
        """Clean up each test."""
        self.node.destroy_node()

    def test_parameters_set(self):
        """
        Test that parameters are set correctly.

        Verify node accepts and stores parameters.
        """
        # This is a placeholder - actual parameter testing depends on
        # your node's parameter interface

        # Example: Declare and get a parameter
        # self.node.declare_parameter('test_param', 'default_value')
        # param_value = self.node.get_parameter('test_param').value
        # self.assertEqual(param_value, 'default_value')

        pass


# Additional test utilities


def wait_for_topic_publish(node, topic_name, msg_type, timeout_sec=5.0):
    """
    Utility function to wait for a topic to publish a message.

    Args:
        node: ROS 2 node to use for subscription
        topic_name: Name of topic to monitor
        msg_type: Message type class
        timeout_sec: Maximum time to wait

    Returns:
        First message received, or None if timeout
    """
    received_msg = []

    def callback(msg):
        received_msg.append(msg)

    subscription = node.create_subscription(msg_type, topic_name, callback, 10)

    start_time = node.get_clock().now()

    while len(received_msg) == 0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if (node.get_clock().now() - start_time).nanoseconds / 1e9 > timeout_sec:
            break

    node.destroy_subscription(subscription)

    return received_msg[0] if received_msg else None
