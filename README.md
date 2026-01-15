# ROS2 Agent Workspace

This repository allows for the management of multiple layered ROS2 workspaces, configured specifically for use with AI agents like Antigravity.

**It is pre-configured to host the UNH Marine Autonomy Framework.**

The core configuration (`configs/core.repos`) includes repositories for:
- **Project11**: Main autonomy system.
- **Marine AIS**: AIS message decoding and handling.
- **UNH Marine Navigation**: Navigation tools and utilities.
- **Mission/Helm Managers**: Core logic for autonomy control.

## Structure

- **`.agent/`**: Contains agent-specific workflows and knowledge.
- **`configs/`**: Contains `.repos` files (YAML) defining the packages for each workspace layer.
- **`scripts/`**: Helper scripts for setup and environment sourcing.
- **`workspaces/`**: The actual ROS2 workspaces (source code and build artifacts).

## Usage

### 1. Setup a Layer
To initialize a workspace layer defined in `configs/<name>.repos`:

```bash
./scripts/setup.sh <name>
./scripts/setup.sh core
# Examples:
# ./scripts/setup.sh core   (Installs Project11, Marine AIS, etc.)
# ./scripts/setup.sh ui     (Installs RQT and visualization tools)
```

This will:
1. Create `workspaces/<name>_ws/src`
2. Import repositories from `configs/<name>.repos`

### 2. Sourcing the Environment
To source the workspaces in the correct order (Underlay -> ... -> Overlay):

```bash
source scripts/env.sh
```

## Adding New Layers
1. Create a `<new_layer>.repos` file in `configs/`.
2. Run `./scripts/setup.sh <new_layer>`.
