# Quick Start Guide

This guide covers setup for the ROS 2 Agent Workspace, developed by the Center for Coastal and Ocean Mapping / Joint Hydrographic Center (CCOM/JHC) at the University of New Hampshire (UNH).

## Prerequisites

- Ubuntu 22.04 or 24.04 (for ROS 2 Jazzy)
- Git installed
- Internet connection
- Sudo access for system package installation

## Step 1: Clone the Repository

```bash
git clone git@github.com:rolker/ros2_agent_workspace.git
cd ros2_agent_workspace
```

## Step 2: Bootstrap (First Time Only)

This installs ROS 2 Jazzy and all required tools:

```bash
./.agent/scripts/bootstrap.sh
```

This will:
- Set up Ubuntu locale for UTF-8
- Add ROS 2 apt repository
- Install ROS 2 Jazzy Base
- Install development tools (colcon, vcstool, rosdep)
- Initialize rosdep

**Note:** This requires sudo access and will take 5-10 minutes.

## Step 3: Set Up Development Tools

```bash
make setup-dev
```

This creates a Python venv, installs pre-commit, and activates git hooks.

## Step 4: Run Health Check

Verify your setup:

```bash
./.agent/scripts/health_check.sh
```

Or using make:
```bash
make health-check
```

This will check that all required tools are installed and the workspace is properly configured.

## Step 5: Setup Workspace Layers

Import and setup all workspace layers:

```bash
./.agent/scripts/setup.sh
```

Or using make:
```bash
make setup-all
```

This will:
- Bootstrap the project repository
- Read `config/layers.txt` to determine which layers to set up
- Create workspace directories (`layers/main/underlay_ws`, `layers/main/core_ws`, etc.)
- Import all repositories from project's `.repos` files
- Clone Project11, Marine AIS, Navigation packages, geodesy dependencies, etc.

**Note:** This requires SSH access to the repositories. Make sure your SSH key is configured on GitHub.

## Step 6: Build the Workspace

Build all packages:

```bash
./.agent/scripts/build.sh
```

Or using make:
```bash
make build
```

This will:
- Build all workspace layers in order
- Generate a build report in `.agent/scratchpad/build_report.md`
- Source each layer after successful build

**Note:** First build may take 10-20 minutes depending on your system.

## Step 7: Source the Environment

After building, source the workspace to use the packages:

```bash
source .agent/scripts/env.sh
```

You'll need to do this in every new terminal, or add it to your `~/.bashrc`:

```bash
echo "source ~/ros2_agent_workspace/.agent/scripts/env.sh" >> ~/.bashrc
```

## Step 8: Verify Installation

Check that ROS packages are available:

```bash
ros2 pkg list | grep project11
```

You should see various project11 packages listed.

## Next Steps

### Optional: Re-setup or Add Individual Layers

Step 5 above sets up all layers automatically. You can also re-run setup for
a specific layer if needed, or add a new layer that was added to the project
configuration after your initial setup:

```bash
# Re-setup a specific layer
./.agent/scripts/setup.sh core

# Or re-run auto-setup to get all layers
./.agent/scripts/setup.sh
```

### Run Tests

Ensure everything is working:

```bash
./.agent/scripts/test.sh
# or
make test
```

### Check Workspace Status

See the status of all repositories:

```bash
./.agent/scripts/status_report.sh
# or
make status
```

## Working with AI Agents

This workspace is designed for AI-assisted development. If using with an AI agent:

1. **Let the agent manage the workspace** - It knows the structure and conventions
2. **Use natural language** - "build the workspace", "add a new package", etc.
3. **Check your instruction file** — `CLAUDE.md`, `.github/copilot-instructions.md`, or `.agent/AI_RULES.md` for rules and script reference

## Common Tasks

### Add a New Repository

1. Edit the appropriate `.repos` file in `configs/`
2. Run `./.agent/scripts/setup.sh <layer>` to import the new repo
3. Build: `./.agent/scripts/build.sh`

### Clean Build Artifacts

```bash
make clean
```

### Update All Repositories

```bash
cd layers/main/core_ws/src
vcs pull
cd ../../..
./.agent/scripts/build.sh
```

### Create a Feature Branch

```bash
git checkout -b feature/TASK-001-my-feature
# Make changes
git commit -am "Add new feature"
```

## Troubleshooting

If you encounter issues, see the [Troubleshooting](README.md#troubleshooting) section in the main README or run:

```bash
./.agent/scripts/health_check.sh
```

## Getting Help

- Read [README.md](README.md) for detailed usage
- Review [ARCHITECTURE.md](ARCHITECTURE.md) for system design
- Check [CONTRIBUTING.md](CONTRIBUTING.md) for development guidelines
- Open an issue on GitHub

## Summary

You should now have:
- ✅ ROS 2 Jazzy installed
- ✅ Core workspace set up and built
- ✅ Environment sourced and ready to use

The workspace is ready for development.
