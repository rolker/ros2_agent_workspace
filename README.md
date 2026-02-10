# ROS2 Agent Workspace

This repository allows for the management of multiple layered ROS2 workspaces, configured specifically for use with AI agents like GitHub Copilot and Antigravity.

**It is pre-configured to host the UNH Marine Autonomy Framework.**

The workspace automatically imports repositories from the project's configuration files located in the `unh_marine_autonomy` repository:
- **Project11**: Main autonomy system.
- **Marine AIS**: AIS message decoding and handling.
- **UNH Marine Navigation**: Navigation tools and utilities.
- **Mission/Helm Managers**: Core logic for autonomy control.

## Quick Start

### For AI Agents

Each framework has a self-contained instruction file with all rules inline:
- **Claude Code**: [`CLAUDE.md`](CLAUDE.md) (auto-loaded)
- **GitHub Copilot**: [`.github/copilot-instructions.md`](.github/copilot-instructions.md)
- **Gemini CLI**: [`.agent/instructions/gemini-cli.instructions.md`](.agent/instructions/gemini-cli.instructions.md)
- **Other/Unknown**: [`.agent/AI_RULES.md`](.agent/AI_RULES.md)

### For Human Users

**New to this workspace?** See [QUICKSTART.md](QUICKSTART.md) for a step-by-step setup guide.

**Experienced users?** Jump to [Usage](#usage) below.

### For Containerized/Custom Agents

See [`.agent/AGENT_ONBOARDING.md`](.agent/AGENT_ONBOARDING.md) for setup instructions.

## Sandboxed Development (Recommended)

This workspace supports **DevContainers**, allowing you to run the entire environment in a Docker container. This is the **preferred method** for AI Agents and users who want a clean, safe setup.

- **GitHub Codespaces**: Open this repo in Codespaces for a zero-setup cloud desktop.
- **VS Code Local**: Install the "Dev Containers" extension and choose "Reopen in Container".

## Documentation

### For AI Agents
- 📋 [CLAUDE.md](CLAUDE.md) - Claude Code operational rules (auto-loaded)
- 📋 [AI Rules](.agent/AI_RULES.md) - Universal fallback rules
- 🌲 [Worktree Guide](.agent/WORKTREE_GUIDE.md) - Parallel development with git worktrees
- 📚 [Agent Onboarding](.agent/AGENT_ONBOARDING.md) - Onboarding for containerized agents

### For All Users
- 📚 [Quick Start Guide](QUICKSTART.md) - Get up and running in minutes
- 🏗️ [Architecture Guide](ARCHITECTURE.md) - Understanding the layered workspace system
- 🤝 [Contributing Guide](CONTRIBUTING.md) - Development workflow and best practices
- 🔒 [Security Policy](SECURITY.md) - Security guidelines and reporting

## Structure

- **`.agent/`**: Contains agent-specific workflows and knowledge.
- **`configs/`**: Contains `project_bootstrap.url` linking to project configuration.
- **`.agent/scripts/`**: Helper scripts for setup and environment sourcing.
- **`layers/`**: The ROS2 workspace layers (source code and build artifacts).

## Usage

### 1. Setup Layers
To initialize workspace layers:

```bash
# Auto-setup all layers (recommended)
./.agent/scripts/setup.sh

# Or setup specific layer
./.agent/scripts/setup.sh core
```

This will:
1. Bootstrap the project repository (if needed)
2. Read layer definitions from project's `config/layers.txt`
3. Import repositories from project's `config/repos/<layer>.repos` files

### 2. Sourcing the Environment
To source the layers in the correct order (Underlay -> ... -> Overlay):

```bash
source .agent/scripts/env.sh
```

## Getting Started for Humans (Agentic Coding)

This workspace is designed to be used with an AI Agent. If you are new to agentic coding, here is how you should interact with this repository:

1.  **Just Ask**: The agent is capable of managing the entire lifecycle of the workspace. You can ask it to "add a repo", "build the workspace", or "fix this build error".
2.  **Agent Instructions**: Each agent framework has a self-contained instruction file (e.g., `CLAUDE.md`) with all rules and a script reference table. Common operations include building (`make build`), testing (`make test`), and status checking (`.agent/scripts/status_report.sh`).
3.  **Let the Agent Drive**: The agent is aware of the directory structure (layers in `layers/`, key repository configs in `layers/main/core_ws/src/unh_marine_autonomy/config/repos/`). Trust it to place files in the correct location.

## Using with Custom Projects

This workspace can be configured to bootstrap from a "Key Repository" of your choice (e.g., your project's main repo).

1. Create a `configs/project_bootstrap.url` file containing the raw URL to your bootstrap configuration:
   ```text
   https://raw.githubusercontent.com/your-org/your-repo/main/config/bootstrap.yaml
   ```
   > [!CAUTION]
   > **Security Warning**: executes code (via `vcs import`) based on the configuration downloaded from this URL. Ensure you trust the source URL and that it is not compromised. Always use HTTPS.

2. Ensure your Key Repo has the following structure:
   - `config/bootstrap.yaml`:
     ```yaml
     git_url: https://github.com/your-org/your-repo.git
     branch: main
     ```
   - `config/repos/`: Directory containing `.repos` files (e.g., `core.repos`, `ui.repos`).

3. Run `./.agent/scripts/setup.sh` to import all repositories defined in your Key Repo.

## Adding New Layers
1. Create a `<new_layer>.repos` file in the key repository's `config/repos/` directory (e.g., `unh_marine_autonomy/config/repos/my_layer.repos`).
2. Add the layer name to `config/layers.txt` in the key repository.
3. Run `./.agent/scripts/setup.sh` to set up all layers (including the new one).

## Troubleshooting

### Common Issues

#### VCS Command Not Found
If you see "vcs command not found":
```bash
pip install vcstool
# or
sudo apt install python3-vcstool
```

#### ROS 2 Jazzy Not Found
Ensure ROS 2 Jazzy is installed:
```bash
./.agent/scripts/bootstrap.sh
```

#### Build Failures
1. Check the build report: `cat .agent/scratchpad/build_report.md`
2. Try building a single layer:
   ```bash
   cd layers/main/core_ws
   colcon build --symlink-install
   ```
3. Check for missing dependencies:
   ```bash
   rosdep install --from-paths layers/main/core_ws/src --ignore-src -r -y
   ```

#### Workspace Locked
If you see "Workspace is LOCKED":
```bash
./.agent/scripts/unlock.sh
```

#### Environment Not Sourced
If ROS commands aren't working:
```bash
source .agent/scripts/env.sh
```

#### Permission Denied
Make sure scripts are executable:
```bash
chmod +x .agent/scripts/*.sh
```

### Getting Help

- Review [ARCHITECTURE.md](ARCHITECTURE.md) for system design
- Check [CONTRIBUTING.md](CONTRIBUTING.md) for development guidelines
- See [SECURITY.md](SECURITY.md) for security policies
- Open an issue on GitHub for bugs or feature requests

## Development Tools

### Makefile
Common operations are available via make:
```bash
make help          # Show all available targets
make health-check  # Run comprehensive health check
make bootstrap     # Install ROS2 and dependencies
make setup-all     # Setup all workspace layers
make build         # Build all layers
make test          # Run all tests
make clean         # Clean build artifacts
make status        # Show workspace status
make lint          # Run linters

### Helper Scripts
- `verify_change.sh`: Targeted verification (unit/lint) for a specific package.
- `status_report.sh`: Comprehensive workspace status with test history.
```

### Pre-commit Hooks
Install pre-commit hooks for automatic validation:
```bash
pip install pre-commit
pre-commit install
```

### Validation
Validate configuration files:
```bash
python3 .agent/scripts/validate_repos.py
```
