# [Package Name]

> Brief one-sentence description from `package.xml`.

## Overview

[2--3 sentences describing what the package does, when it is used, and how it fits
into the larger system. Cite the source of any architectural claims.]

## Nodes

### `node_name`

[One sentence describing the node's role.]

#### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `param_name` | `double` | `1.0` | What it controls |

#### Subscribed Topics

| Topic | Message Type | Description |
|---|---|---|
| `input_topic` | `std_msgs/msg/String` | What it receives |

#### Published Topics

| Topic | Message Type | Description |
|---|---|---|
| `output_topic` | `geometry_msgs/msg/Twist` | What it publishes |

#### Services (Provided)

| Service | Type | Description |
|---|---|---|
| `service_name` | `std_srvs/srv/SetBool` | What it does |

#### Services (Called)

| Service | Type | Description |
|---|---|---|
| `service_name` | `std_srvs/srv/Trigger` | Why it calls this |

#### Actions (Provided)

| Action | Type | Description |
|---|---|---|
| `action_name` | `pkg/action/Type` | What it does |

#### Actions (Called)

| Action | Type | Description |
|---|---|---|
| `action_name` | `pkg/action/Type` | Why it calls this |

<!-- Repeat the "### `node_name`" block for each node in the package. -->

## Message Definitions

| Message | File | Description |
|---|---|---|
| `PackageName/msg/MessageName` | `msg/MessageName.msg` | What it represents |

## Service Definitions

| Service | File | Description |
|---|---|---|
| `PackageName/srv/ServiceName` | `srv/ServiceName.srv` | What it does |

## Action Definitions

| Action | File | Description |
|---|---|---|
| `PackageName/action/ActionName` | `action/ActionName.action` | What it does |

## Launch Files

| Launch File | Description | Key Arguments |
|---|---|---|
| `launch/file.launch.py` | What it launches | `arg1`, `arg2` |

---

## Instructions for Use

1. Copy this template and fill in each section by reading the actual source code.
2. **Omit** any section that does not apply (e.g., if the package has no actions,
   remove the Actions sections entirely). Do not leave empty tables.
3. Follow the verification workflow in
   [`.agent/knowledge/documentation_verification.md`](../knowledge/documentation_verification.md)
   before submitting.

## Verification Checklist

- [ ] Every parameter name matches a `declare_parameter` call in the source
- [ ] Every topic name matches a `create_publisher` or `create_subscription` call
- [ ] Every message/service/action type matches the source include or import
- [ ] Every default value is copied from the source, not assumed
- [ ] Sections with no applicable content have been removed (not left empty)
- [ ] Launch file arguments match the `DeclareLaunchArgument` calls
