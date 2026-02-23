# ROS 2 Agent Workspace

## Vision

This workspace exists to help develop robust, well-documented, and well-tested ROS 2 packages using AI agent assistance.

The immediate goal is practical: build reliable autonomous marine systems — starting with uncrewed surface vessels that can safely survey an area of the seafloor without operator intervention. That demands software that is robust in the face of real-world conditions, reliable enough to operate without intervention, and easy to deploy.

Agent-assisted development is the means, not the end. The workspace provides the infrastructure — worktree isolation, build tooling, agent instructions, quality enforcement — that makes AI agents effective contributors to ROS 2 projects. The workspace should improve over time, informed both by experience from real project use and by evolving practices in the agentic coding domain. The measure of improvement is whether projects get better faster — not whether the workspace itself is more sophisticated.

What "better packages" means in practice:
- Well-documented — useful to teammates, contributors, and the broader ROS 2 community
- Well-tested — reliable enough for real-world deployment
- Maintainable — clear code and documentation that others can work with
- Distribution-ready — packageable for rosdep installation, visible in the ROS ecosystem

The workspace infrastructure is project-agnostic by design. While its primary project is UNH marine autonomy, the tooling generalizes to any ROS 2 project. This is intentional — it makes the workspace useful beyond its origin, encourages contributions from other domains, and ensures the infrastructure doesn't become coupled to any single project.

## About

This repository provides management of multiple layered ROS 2 workspaces, configured for use with AI agents. It is developed by the Center for Coastal and Ocean Mapping / Joint Hydrographic Center (CCOM/JHC) at the University of New Hampshire (UNH) -- a national center of expertise in ocean mapping and hydrographic sciences.

As currently configured, the workspace hosts the **UNH Marine Autonomy Framework**, automatically importing repositories from the project's manifest repository (`unh_marine_autonomy`):
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

- **`.agent/`**: Agent infrastructure (scripts, hooks, knowledge, templates, identity).
- **`configs/`**: Bootstrap URL, `manifest` symlink to project config, optional `manifest_repo/` clone.
- **`.agent/scripts/`**: Helper scripts for setup and environment sourcing.
- **`layers/`**: The ROS 2 workspace layers (source code and build artifacts).

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
1. Bootstrap the manifest repository (if needed) and create `configs/manifest` symlink
2. Read layer definitions from `configs/manifest/layers.txt`
3. Import repositories from `configs/manifest/repos/<layer>.repos` files

### 2. Sourcing the Environment
To source the layers in the correct order (Underlay -> ... -> Overlay):

```bash
source .agent/scripts/env.sh
```

## Getting Started for Humans (Agentic Coding)

This workspace is designed to be used with an AI Agent. If you are new to agentic coding, here is how you should interact with this repository:

1.  **Just Ask**: The agent is capable of managing the entire lifecycle of the workspace. You can ask it to "add a repo", "build the workspace", or "fix this build error".
2.  **Agent Instructions**: Each agent framework has a self-contained instruction file (e.g., `CLAUDE.md`) with all rules and a script reference table. Common operations include building (`make build`), testing (`make test`), and status checking (`make status` or `make status-quick`).
3.  **Let the Agent Drive**: The agent is aware of the directory structure (layers in `layers/`, manifest config at `configs/manifest/`). Trust it to place files in the correct location.

## Using with Custom Projects

This workspace can be configured to bootstrap from a **manifest repository** of your choice. The manifest repo provides layer definitions (`.repos` files) and optionally project-specific agent context.

1. Create a `configs/project_bootstrap.url` file containing the raw URL to your bootstrap configuration:
   ```text
   https://raw.githubusercontent.com/your-org/your-repo/main/config/bootstrap.yaml
   ```
   > [!CAUTION]
   > **Security Warning**: executes code (via `vcs import`) based on the configuration downloaded from this URL. Ensure you trust the source URL and that it is not compromised. Always use HTTPS.

2. Ensure your manifest repo has the following structure:
   - `config/bootstrap.yaml`:
     ```yaml
     git_url: https://github.com/your-org/your-repo.git
     branch: main
     # layer: core          # If your repo also contains ROS packages (Pattern B)
     # config_path: config  # Defaults to "config" if omitted
     ```
   - `config/repos/`: Directory containing `.repos` files (e.g., `core.repos`, `ui.repos`).
   - `config/layers.txt`: Layer names in source order (one per line).
   - `config/agent_context/` (optional): Project-specific AI agent knowledge.

3. Run `./.agent/scripts/setup.sh` to import all repositories defined in your manifest repo.

**Pattern A vs Pattern B**: If your manifest repo is config-only (no ROS packages), omit the `layer` field. It will be cloned to `configs/manifest_repo/`. If it also contains ROS packages, set `layer` to the target layer (e.g., `core`) and it will be cloned into that layer's `src/` directory.

## Adding New Layers
1. Create a `<new_layer>.repos` file in the manifest repo's `config/repos/` directory.
2. Add the layer name to `config/layers.txt` in the manifest repo.
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
2. Try rebuilding all layers:
   ```bash
   make build
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
make bootstrap     # Install ROS 2 and dependencies
make setup-all     # Setup all workspace layers
make build         # Build all layers
make test          # Run all tests
make clean         # Clean build artifacts
make status        # Full status (sync + repos + GitHub PRs/issues)
make status-quick  # Quick local-only status (no sync, no GitHub)
make lint          # Run linters
```

**Claude Code users**: All `.PHONY` Makefile targets (except `help`) are available as `/make_*` slash commands
(e.g., `/make_build`, `/make_test`, `/make_status`). Run `make generate-skills` to
regenerate them after Makefile changes.

### Helper Scripts
- `verify_change.sh`: Targeted verification (unit/lint) for a specific package.
- `status_report.sh`: Comprehensive workspace status with `--quick` and `--pr-triage` flags.

### IDE Setup
- [VS Code Setup Guide](.agent/knowledge/vscode_setup.md): Multi-root workspace, build tasks, C++/Python IntelliSense configuration.

### Pre-commit Hooks
Set up pre-commit hooks for automatic validation:
```bash
make setup-dev    # creates venv, installs pre-commit, activates git hooks
make lint         # run all hooks manually
```

### Validation
Validate configuration files:
```bash
python3 .agent/scripts/validate_repos.py
```
