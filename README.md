# ROS2 Agent Workspace

This repository allows for the management of multiple layered ROS2 workspaces, configured specifically for use with AI agents like Antigravity.

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
# Example: ./scripts/setup.sh core
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
