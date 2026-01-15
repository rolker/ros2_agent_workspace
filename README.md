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

## Getting Started for Humans (Agentic Coding)

This workspace is designed to be used with an AI Agent. If you are new to agentic coding, here is how you should interact with this repository:

1.  **Just Ask**: The agent is capable of managing the entire lifecycle of the workspace. You can ask it to "add a repo", "build the workspace", or "fix this build error".
2.  **Slash Commands**: The agent has a set of workflows it can execute. You can trigger these by asking for them by name (e.g., "run the clean workflow") or by using the slash command directly if your interface supports it.
    *   `/add-repo`: Add a new repository to a layer.
    *   `/build-all`: Build all layers in the correct order.
    *   `/clean`: specific clean up of build artifacts.
    *   `/rebuild-all`: Clean and rebuild everything.
    *   `/submit-pr`: Create a PR for your changes.
    *   *See `.agent/workflows/` for the full list.*
3.  **Let the Agent Drive**: The agent is aware of the directory structure (layers in `workspaces/`, configs in `configs/`). Trust it to place files in the correct location.

## Adding New Layers
1. Create a `<new_layer>.repos` file in `configs/`.
2. Run `./scripts/setup.sh <new_layer>`.
