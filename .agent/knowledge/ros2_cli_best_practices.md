# ROS 2 CLI Guide for AI Agents

This guide outlines how to use the `ros2` command-line interface (CLI) effectively and reliably as an automated agent. The CLI provides a powerful, built-in alternative to running a dedicated MCP server.

## Core Principles

1.  **Avoid Infinite Loops**: Default `ros2` behavior often streams data indefinitely (e.g., `topic echo`). You **MUST** use flags like `--once` or `--timeout` to ensure your command returns control to you.
2.  **Structured Output**: Use formatting flags (`--csv`) or specific filtering (`--field`) to reduce the complexity of text parsing.
3.  **Type Awareness**: Always query for types (`-t`) so you can construct valid YAML inputs for services and publishers.

## Command Cookbook

### 1. Reading Sensor Data (Topics)

**Goal**: Get the current state of a sensor or robot property.

*   **Best Command**:
    ```bash
    ros2 topic echo <topic_name> --once --timeout 2
    ```
*   **For Flat Data** (easier parsing):
    ```bash
    ros2 topic echo <topic_name> --csv --once
    ```
*   **Specific Field**:
    ```bash
    ros2 topic echo <topic_name> --field <field_name> --once
    ```
    *Example*: `ros2 topic echo /odometry --field pose.pose.position.x --once`

> **Note**: If the command times out, the topic might not be publishing.

### 2. Discovering the System

**Goal**: Find out what is running.

*   **List Topics with Types**:
    ```bash
    ros2 topic list -t
    ```
    *Output*: ` /topic_name [message_type]` — parses easily to find "what can I read?"

*   **List Nodes**:
    ```bash
    ros2 node list
    ```

*   **Node Details** (Subscribers/Publishers):
    ```bash
    ros2 node info <node_name>
    ```

### 3. Controlling the Robot (Publishing)

**Goal**: Send a command/signal.

*   **Command**:
    ```bash
    ros2 topic pub <topic_name> <message_type> "<yaml_data>" --once
    ```
    *Example*: `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}" --once`

### 4. Invoking Services

**Goal**: Trigger an immediate action or computation.

*   **Command**:
    ```bash
    ros2 service call <service_name> <service_type> "<yaml_data>"
    ```

### 5. Understanding Data Structures

**Goal**: Know what fields are available in a message type.

*   **Command**:
    ```bash
    ros2 interface show <message_type>
    ```
    *Output*: Shows the definition (e.g., `float32 x`, `float32 y`). Use this to construct your YAML input or understand `topic echo` output.

## Troubleshooting

*   **Command not found?**: Ensure you source the environment: `source /opt/ros/jazzy/setup.bash`
*   **Missing messages?**: The topic might be using a different QoS profile. Try adding `--qos-reliability best_effort`.
*   **Hanging command?**: You likely forgot `--once` or `--timeout`.

## Agent Workflow Example

1.  **Discovery**: `ros2 topic list -t` -> find `/battery_state` [sensor_msgs/msg/BatteryState].
2.  **Introspection**: `ros2 interface show sensor_msgs/msg/BatteryState` -> see `float32 percentage`.
3.  **Action**: `ros2 topic echo /battery_state --field percentage --once` -> get `0.85`.
