---
name: document-package
description: Generate or update ROS 2 package documentation (README and API docs) by reading source code. Enforces the documentation verification workflow.
---

# Document Package

## Usage

```
/document-package [<package-path>]
```

If no path is given, use the current directory.

## Overview

**Lifecycle position**: Utility — use after `audit-project` flags documentation
gaps, or when asked to "document a package", "update the README", or "write
API docs".

Generates both user-facing package documentation (README with nodes, topics,
parameters) and developer-facing API documentation (class/function reference)
by reading the actual source code. Merges what were previously two separate
skills (`ros-documentation` and `ros-code-documentation`).

**Cardinal rule**: No fact without reading the source. Every parameter name,
topic name, message type, and default value must come from the code, not
assumptions. See `.agent/knowledge/documentation_verification.md`.

## Steps

### 1. Inventory the package

```bash
# Package metadata
cat <package_path>/package.xml

# Source files
find <package_path> -name '*.py' -o -name '*.cpp' -o -name '*.hpp' -o -name '*.h' | sort

# Interface definitions
find <package_path> -name '*.msg' -o -name '*.srv' -o -name '*.action' | sort

# Launch files
find <package_path> -name '*.launch.py' -o -name '*.launch.xml' -o -name '*.launch.yaml' | sort
```

Record `name`, `description`, `maintainer`, `license`, and dependencies from
`package.xml`.

### 2. Extract facts from source

Follow the command cookbook in `.agent/knowledge/documentation_verification.md`
to grep for every:

- **Parameter**: `declare_parameter` / `declare_parameters`
- **Publisher**: `create_publisher`
- **Subscriber**: `create_subscription`
- **Service server/client**: `create_service` / `create_client`
- **Action server/client**: `rclcpp_action::create_server` / `create_client`

Record each finding with its file path and line number.

### 3. Analyze launch files

Read every launch file. Note:
- `DeclareLaunchArgument` entries (name, default, description)
- Topic remappings
- Parameter overrides
- Node composition

### 4. Analyze API (libraries and modules)

For C++ libraries (headers in `include/<package_name>/`):
- Public class definitions
- Public methods and their signatures
- Type definitions (`struct`, `enum`, `typedef`)

For Python modules (in `<package_name>/` or `src/<package_name>/`):
- Class definitions and `__init__` signatures
- Public methods
- Module-level functions

### 5. Generate package README

Use the template at `.agent/templates/package_documentation.md`:

- Fill in each section from the facts gathered in steps 1–3.
- **Omit** sections that don't apply (no empty tables).
- If a README already exists, preserve custom sections (e.g., "Theory of
  Operation") while standardizing the nodes/API/usage sections.

### 6. Generate API documentation (if applicable)

If the package contains libraries or modules with public APIs (not just
ROS nodes):

1. Create `docs/API.md` in the package root.
2. Document each public class with its constructor, public methods, and
   usage example.
3. Include type definitions and enums.

Skip this step if the package only contains ROS node executables with no
reusable library code.

### 7. Self-review

Run through the verification checklist from
`.agent/templates/package_documentation.md`:

- [ ] Every parameter name matches a `declare_parameter` call
- [ ] Every topic name matches a `create_publisher` or `create_subscription` call
- [ ] Every message/service/action type matches the source import
- [ ] Every default value is copied from the source
- [ ] Sections with no applicable content have been removed
- [ ] Launch file arguments match `DeclareLaunchArgument` calls

## References

- `.agent/knowledge/documentation_verification.md` — Verification workflow
  and command cookbook
- `.agent/templates/package_documentation.md` — README template with
  verification checklist

## Guidelines

- **Read before writing** — complete steps 1–4 before generating any text.
- **Omit, don't leave empty** — if a package has no services, remove the
  services section entirely.
- **Preserve existing work** — if a README exists, update rather than replace.
  Keep custom sections that add value.
- **One package at a time** — don't batch across packages. Each package gets
  its own focused documentation pass.
- **Cite line numbers** — when documenting a parameter or topic, note the
  source file and line so reviewers can verify.
