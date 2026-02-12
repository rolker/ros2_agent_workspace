# Documentation Verification Workflow

## Why This Exists

In PR [rolker/unh_marine_autonomy#37](https://github.com/rolker/unh_marine_autonomy/pull/37),
AI-generated package documentation contained 13 factual errors: fabricated parameters,
wrong message types, non-existent API signatures, and incorrect topic names. The root
cause was that the agent documented from assumptions instead of reading source code.

This workflow prevents that class of error.

## Cardinal Rule

> **No fact without a `file:line` reference.**
>
> Every parameter name, topic name, message type, service type, action type, default
> value, and API signature in documentation must trace back to a specific line in the
> source code. If you cannot point to the line, do not write the claim.

## Step-by-Step Process

### 1. Inventory the Package

```bash
# List all source files in the package
find <package_path> -name '*.py' -o -name '*.cpp' -o -name '*.hpp' -o -name '*.h' | sort

# List message, service, and action definitions
find <package_path> -name '*.msg' -o -name '*.srv' -o -name '*.action' | sort

# List launch files
find <package_path> -name '*.launch.py' -o -name '*.launch.xml' -o -name '*.launch.yaml' | sort

# Read package.xml for dependencies and description
cat <package_path>/package.xml
```

### 2. Extract Facts via Grep

Use the command cookbook below to find every parameter, topic, service, and action
declared in the source. Record each finding with its file and line number.

### 3. Cross-Reference Launch Files

Launch files may remap topics, set parameter overrides, or compose multiple nodes.
Read every launch file in the package and verify that your documentation matches
the launch-time configuration, not just the source defaults.

### 4. Self-Review Checklist

Before submitting documentation, verify each row in every table:

- [ ] Parameter name matches `declare_parameter` call exactly
- [ ] Default value matches the source (or "none" if no default is provided)
- [ ] Topic name matches `create_publisher` / `create_subscription` call exactly
- [ ] Message type matches the template argument or string, including the package prefix
- [ ] Service/action types match their definition files
- [ ] Every row has a `Source` column with `file:line`
- [ ] No section is present for a category the package does not use (omit, don't leave empty)

## Command Cookbook

### Parameters (C++)

```bash
# Find all declared parameters
grep -rn 'declare_parameter' <package_path>/src/ <package_path>/include/
```

### Parameters (Python)

```bash
# Find all declared parameters
grep -rn 'declare_parameter\|declare_parameters' <package_path>/<package_name>/
```

### Publishers and Subscribers (C++)

```bash
# Publishers
grep -rn 'create_publisher' <package_path>/src/ <package_path>/include/

# Subscribers
grep -rn 'create_subscription' <package_path>/src/ <package_path>/include/
```

### Publishers and Subscribers (Python)

```bash
# Publishers
grep -rn 'create_publisher' <package_path>/<package_name>/

# Subscribers
grep -rn 'create_subscription' <package_path>/<package_name>/
```

### Services

```bash
# Service servers
grep -rn 'create_service' <package_path>/src/ <package_path>/include/ <package_path>/<package_name>/

# Service clients
grep -rn 'create_client' <package_path>/src/ <package_path>/include/ <package_path>/<package_name>/
```

### Actions

```bash
# Action servers
grep -rn 'create_server\|rclcpp_action::create_server' <package_path>/src/ <package_path>/include/

# Action clients
grep -rn 'create_client\|rclcpp_action::create_client' <package_path>/src/ <package_path>/include/
```

### Message / Service / Action Definitions

```bash
# List all interface definitions
find <package_path> -name '*.msg' -o -name '*.srv' -o -name '*.action'

# Check CMakeLists.txt for generated interfaces
grep -n 'rosidl_generate_interfaces\|find_package.*_interfaces' <package_path>/CMakeLists.txt
```

## Common Hallucination Anti-Patterns

These are the mistakes agents make most often. Check your documentation against each one.

| Anti-Pattern | Example | How to Avoid |
|---|---|---|
| **Fabricated parameters** | Documenting `max_speed` when the code only declares `speed_limit` | Grep for `declare_parameter`; use exact names |
| **Assumed topic names** | Writing `/cmd_vel` when the code publishes on `helm/cmd_vel` | Grep for `create_publisher`; use exact strings |
| **Wrong message types** | Claiming `std_msgs/Float64` when the code uses `geometry_msgs/Twist` | Check the template argument in the publisher call |
| **Omitted interfaces** | Skipping a service that the node actually provides | Grep for `create_service`; document all matches |
| **Invented default values** | Writing "default: 1.0" when the parameter has no default | Copy the default from the `declare_parameter` call |
| **Wrong package prefix** | Writing `sensor_msgs/Imu` when it is `sensor_msgs/msg/Imu` | Use the full qualified type from the include/import |

## Workspace-Specific Notes

- **`layers/` is gitignored** -- standard glob/find from the workspace root will not
  find source files. Use `ls` to explore layer directories, or search within a specific
  layer path (e.g., `layers/main/core_ws/src/<package>/`).
- **Interface packages** may live in a different layer than the node package. Check
  `package.xml` dependencies to find where message/service/action types are defined.
