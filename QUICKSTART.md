# Quick Start Guide

This guide will help you get started with the ROS2 Agent Workspace in just a few minutes.

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

## Step 3: Install Python Dependencies

```bash
pip install -r requirements.txt
```

Or using make:
```bash
make install-deps
```

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

## Step 5: Setup Core Workspace

Import and setup the core autonomy packages:

```bash
./.agent/scripts/setup.sh core
```

Or using make:
```bash
make setup-core
```

This will:
- Create `workspaces/core_ws/src`
- Import all repositories from `configs/core.repos`
- Clone Project11, Marine AIS, Navigation packages, etc.

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
- Generate a build report in `ai_workspace/build_report.md`
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

### Setup Additional Layers

Depending on your needs, setup additional workspace layers:

```bash
# Platform-specific packages (e.g., for Ben robot)
./.agent/scripts/setup.sh platforms

# Sensor drivers and perception
./.agent/scripts/setup.sh sensors

# Simulation tools
./.agent/scripts/setup.sh simulation

# User interfaces and visualization
./.agent/scripts/setup.sh ui
```

Or setup all layers at once:
```bash
make setup-all
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
3. **Use workflows** - Trigger with slash commands like `/build-all`, `/clean`
4. **Check `.agent/workflows/`** for available workflows

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
cd workspaces/core_ws/src
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

Happy coding! 🚀
