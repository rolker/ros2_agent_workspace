# Agent Scripts Reference

Helper scripts in `.agent/scripts/` for workspace management. These scripts are called by workflows and can also be run directly.

## Setup & Initialization

### `setup.sh`

Initialize a workspace layer from a `.repos` configuration file.

**Usage:**
```bash
./.agent/scripts/setup.sh <layer_name>
```

**Examples:**
```bash
./.agent/scripts/setup.sh core        # Setup core layer (Project11, Marine AIS, etc.)
./.agent/scripts/setup.sh ui          # Setup UI layer (RQT, visualization tools)
./.agent/scripts/setup.sh sensors     # Setup sensors layer
```

**What it does:**
1. Creates `workspaces/<layer>_ws/src` directory
2. Imports repositories from `configs/<layer>.repos` using `vcs import`
3. Initializes git for each repository

**Dependencies:**
- `vcstool` (`pip install vcstool` or `apt install python3-vcstool`)
- `git`

**Outputs:**
- `workspaces/<layer>_ws/src/` directory with cloned repositories

---

### `bootstrap.sh`

Install ROS 2 Jazzy and system dependencies on Ubuntu 24.04.

**Usage:**
```bash
./.agent/scripts/bootstrap.sh
```

**What it does:**
1. Adds ROS 2 Jazzy apt repository
2. Installs ROS 2 base packages
3. Installs colcon build tools
4. Installs vcstool and other dependencies

**When to use:**
- First time setup on Ubuntu 24.04
- If ROS 2 commands are not found
- Before building workspace layers

**Dependencies:**
- Ubuntu 24.04
- sudo access

---

## Status & Reporting

### `status_report.sh`

Generate a comprehensive workspace status report.

**Usage:**
```bash
./.agent/scripts/status_report.sh
```

**Output example:**
```
# Workspace Status Report
**Date**: Wed Jan 21 11:47:12 AM EST 2026

## Root Repository
- **Status**: ✅ Clean
- **Branch**: main

## Workspace: **core** (Total: 5, Clean: 4, Attention: 1)
...
```

**What it shows:**
- Git status of root repository
- For each workspace layer:
  - Number of repositories
  - Clean vs. repositories needing attention
  - Repositories behind on commits
  - Repositories on non-Jazzy branches
  - Untracked files

**When to use:**
- Before committing changes
- Checking if workspace is ready for build
- Diagnosing repository state issues

**Note:** Some deprecation warnings from vcstool are harmless and can be ignored.

---

## Building & Compilation

### `build.sh`

Build ROS 2 workspace layers with colcon.

**Usage:**
```bash
./.agent/scripts/build.sh [layer_names...]
```

**Examples:**
```bash
./.agent/scripts/build.sh              # Build all layers
./.agent/scripts/build.sh core         # Build only core layer
./.agent/scripts/build.sh core ui      # Build core and ui layers
```

**What it does:**
1. Sources the ROS 2 environment
2. Runs `colcon build --symlink-install` on each layer
3. Generates build report in `ai_workspace/build_report.md`

**Dependencies:**
- ROS 2 Jazzy installed
- All workspace layers initialized (see `setup.sh`)
- System packages installed (see `bootstrap.sh`)

**Outputs:**
- Build artifacts in each `workspaces/<layer>_ws/build/`
- `ai_workspace/build_report.md` with build summary

---

## Environment Management

### `env.sh`

Source the ROS 2 environment for all workspace layers in correct order.

**Usage:**
```bash
source ./.agent/scripts/env.sh
```

**What it does:**
1. Sources ROS 2 Jazzy system installation
2. Sources each workspace layer in order (underlay → overlay)
3. Sets up environment variables for ROS tools

**When to use:**
- Before running ROS commands (rostopic, rosservice, colcon build)
- In new terminal/shell session
- After building a new layer

**Note:** This must be sourced (not executed) to affect current shell.

---

## Git & Configuration

### `configure_git_identity.sh`

Configure git identity for agent commits across all repositories.

**Usage:**
```bash
./.agent/scripts/configure_git_identity.sh "<Name>" "<email>"
```

**Example:**
```bash
./.agent/scripts/configure_git_identity.sh "Copilot CLI Agent" "roland+copilot-cli@ccom.unh.edu"
```

**What it does:**
1. Configures git in workspace repository
2. Configures git in all 19+ repositories under `workspaces/*/src/`
3. Sets git user.name and user.email in each `.git/config`

**Why it matters:**
- Distinguishes agent commits from human commits
- Satisfies audit trail requirements
- Prevents "ghost commits" from appearing user-authored

**Important:** Must be run BEFORE making any commits in a session.

**Dependencies:**
- git
- Write access to all repository `.git/config` files

---

### `lock.sh` / `unlock.sh`

Lock/unlock the workspace to prevent accidental modifications.

**Usage:**
```bash
./.agent/scripts/lock.sh              # Lock the workspace
./.agent/scripts/unlock.sh            # Unlock the workspace
```

**What it does:**
- Creates/removes a `.workspace.lock` file
- Workflows can check for this file to prevent operations on locked workspaces

**When to use:**
- `lock.sh`: Before long operations or to pause work
- `unlock.sh`: When resuming work or if locked unexpectedly

---

### `checkout_default_branch.sh`

Automatically detect and switch to the remote default branch (e.g. `main` or `jazzy`).
 
 **Usage:**
 ```bash
 ./.agent/scripts/checkout_default_branch.sh
 ```
 
 **What it does:**
 1. Detects `origin/HEAD`
 2. Checks for uncommitted changes (Safeguard)
 3. Switches to default branch
 4. Pulls latest changes
 
 **Safety:**
 - Will **EXIT with error** if you have uncommitted changes. This prevents accidental loss of work or "hiding" work in stashes.
 
 ---

---

## Testing & Validation

### `test.sh`

Run tests on ROS 2 workspace layers.

**Usage:**
```bash
./.agent/scripts/test.sh [layer_names...]
```

**Examples:**
```bash
./.agent/scripts/test.sh               # Test all layers
./.agent/scripts/test.sh core          # Test only core layer
```

**What it does:**
1. Sources ROS 2 environment
2. Runs `colcon test` on specified layers
3. Generates test report

**Dependencies:**
- ROS 2 Jazzy installed
- Workspace built (see `build.sh`)

**Outputs:**
- `ai_workspace/test_report.md` (Human readable)
- `ai_workspace/test_summary.json` (Machine readable)
- `ai_workspace/test_history.csv` (Historical data)

---

### `verify_change.sh`

Run targeted verification on a specific package.

**Usage:**
```bash
./.agent/scripts/verify_change.sh --package <package_name> [--type <unit|lint|all>]
```

**Examples:**
```bash
./.agent/scripts/verify_change.sh --package unh_marine_autonomy --type lint
./.agent/scripts/verify_change.sh --package marine_interfaces --type unit
```

**What it does:**
1. Auto-detects the workspace layer containing the package
2. Runs `colcon test` with specific filters (e.g. `-L unit`)
3. Returns 0 on success, 1 on failure

**When to use:**
- Before committing changes to a specific package
- When an agent is verifying a fix

---

### `validate_repos.py`

Validate `.repos` configuration files for syntax errors.

**Usage:**
```bash
python3 ./.agent/scripts/validate_repos.py
```

**What it does:**
1. Checks all `.repos` files in `configs/`
2. Validates YAML syntax
3. Reports any errors or warnings

**When to use:**
- Before running `setup.sh`
- When modifying `.repos` files
- Troubleshooting import failures

---

## Documentation & Knowledge

### `generate_knowledge.sh`

Index ROS 2 packages and generate knowledge files.

**Usage:**
```bash
./.agent/scripts/generate_knowledge.sh
```

**Outputs:**
- Knowledge index in `.agent/knowledge/`
- Package documentation summaries
- API reference files

---

### `doc_analyzer.py`

Analyze documentation quality of ROS 2 packages.

**Usage:**
```bash
python3 ./.agent/scripts/doc_analyzer.py
```

**Outputs:**
- Documentation coverage report
- Missing documentation warnings
- Quality metrics

---

## Utility Scripts

### `build_report_generator.py`

Generate detailed build reports.

**Usage:**
```bash
python3 ./.agent/scripts/build_report_generator.py
```

**Outputs:**
- Detailed build log summary
- Failed packages list
- Compilation warnings and errors
- Build performance metrics

---

### `list_overlay_repos.py`

List all repositories in overlay workspace layers.

**Usage:**
```bash
python3 ./.agent/scripts/list_overlay_repos.py
```

**Outputs:**
- Formatted list of all cloned repositories
- Directory structure
- Repository status

---

## Common Workflows

### Initialize workspace (first time)
```bash
./.agent/scripts/bootstrap.sh          # Install ROS 2 and tools
./.agent/scripts/setup.sh core         # Clone core repositories
./.agent/scripts/setup.sh ui           # Clone UI repositories
source ./.agent/scripts/env.sh         # Source environment
./.agent/scripts/build.sh              # Build all layers
```

### Daily development
```bash
source ./.agent/scripts/env.sh         # Each new terminal
./.agent/scripts/status_report.sh      # Check workspace state
./.agent/scripts/build.sh core         # Build changes
./.agent/scripts/test.sh core          # Run tests
```

### Before committing
```bash
./.agent/scripts/status_report.sh      # Verify clean state
# Commit changes to feature branch
```

### Troubleshooting
```bash
./.agent/scripts/status_report.sh      # See what's wrong
python3 ./.agent/scripts/validate_repos.py  # Check configs
./.agent/scripts/build_report_generator.py  # Detailed errors
```

---

## Error Messages & Solutions

### "vcs command not found"
```bash
pip install vcstool
# or
sudo apt install python3-vcstool
```

### "ROS 2 Jazzy not found"
```bash
./.agent/scripts/bootstrap.sh
source ./.agent/scripts/env.sh
```

### "Workspace is LOCKED"
```bash
./.agent/scripts/unlock.sh
```

### Script not executable
```bash
chmod +x ./.agent/scripts/*.sh
```

---

**Last updated**: Generated by workflow discoverability task  
**Location**: `.agent/scripts/`  
**Total scripts**: 13 helper scripts and tools
