# Git Worktree Guide

**Purpose**: Enable parallel development through isolated working directories. Each worktree has its own builds, tests, and uncommitted changes—completely separate from the main workspace.

## Quick Start

```bash
# Create a worktree for issue #42 to work on the core layer
.agent/scripts/worktree_create.sh --issue 42 --type layer --layer core

# Enter the worktree (sources ROS environment)
source .agent/scripts/worktree_enter.sh 42

# Work normally - build, test, commit
cd core_ws && colcon build --packages-select my_package
git add . && git commit -m "feat: my changes"

# When done, clean up
.agent/scripts/worktree_remove.sh 42
```

## Why Worktrees?

### The Problem

When multiple agents or developers work in the same repository:
- Switching branches discards uncommitted changes
- Builds interfere with each other
- Test results overwrite each other
- Lock contention slows everyone down

### The Solution

Git worktrees create separate checkouts of the same repository:
- Each worktree is a separate directory
- Each has its own branch, builds, and uncommitted files
- All worktrees share the same git history
- No conflicts, no locks needed

## Directory Structure

Main builds live in `layers/main/`, worktrees in `layers/worktrees/` and `.workspace-worktrees/`:

```
layers/
├── main/                      # Main builds (shared baseline)
│   ├── underlay_ws/
│   ├── core_ws/
│   │   └── src/
│   │       ├── unh_marine_autonomy/  # Git repos
│   │       ├── camp/
│   │       └── ...
│   ├── platforms_ws/
│   └── ...
└── worktrees/
    └── issue-42/              # Layer worktree for issue 42
        ├── core_ws/           # Hybrid structure (see below)
        │   └── src/
        │       ├── unh_marine_autonomy/  # Git worktree (modified)
        │       ├── camp/ -> ../../../../main/core_ws/src/camp/  # Symlink
        │       └── ...
        ├── underlay_ws -> ../../main/underlay_ws  # Symlink
        ├── platforms_ws -> ../../main/platforms_ws
        └── .scratchpad/       # Isolated scratchpad

.workspace-worktrees/
└── issue-137/                 # Workspace worktree (infrastructure work)
    ├── .agent/                # Real - root repo worktree
    ├── configs/               # Real - root repo worktree
    └── layers/                # Real - root repo worktree
        └── main -> ../../layers/main  # Symlink - read-only access to main builds
```

## Worktree Types

### Layer Worktrees (`--type layer --layer <name> --packages <pkg,...>`)

**Location**: `layers/worktrees/issue-<N>/`

**Use for**: ROS package development, code changes, feature work

**Requires**: 
- `--layer` to specify which layer to work on
- `--packages` to specify which package(s) to modify (comma-separated for multiple)

**What's created** (Hybrid Structure for Efficiency):
- Target layer packages:
  - **Modified packages**: Git worktrees (isolated changes, shared history)
  - **Unmodified packages**: Symlinks to `layers/main/` (reuse builds, save disk)
- Other layers: Symlinks to `layers/main/` workspaces for read-only source access
- Isolated scratchpad (`.scratchpad/`)

**Benefits**:
- ✅ Disk efficiency: Only modified packages have real checkouts
- ✅ Build efficiency: Symlinked packages reuse the main workspace's source tree; each worktree keeps its own build/install artifacts
- ✅ Isolation: Git worktrees allow independent changes

**Example workflow**:
```bash
# Create layer worktree for single package
.agent/scripts/worktree_create.sh --issue 42 --type layer --layer core --packages unh_marine_autonomy

# For multiple packages (coordinated changes)
.agent/scripts/worktree_create.sh --issue 42 --type layer --layer core --packages unh_marine_autonomy,camp

# Enter and work
source .agent/scripts/worktree_enter.sh --issue 42
cd core_ws/src/my_package
# ... make changes ...
colcon build --packages-select my_package
colcon test --packages-select my_package

# Commit and push
git add .
git commit -m "feat: add new feature"
git push -u origin feature/ISSUE-42-description
```

### Workspace Worktrees (`--type workspace`)

**Location**: `.workspace-worktrees/issue-<N>/`

**Use for**: Infrastructure changes, documentation, `.agent/` modifications

**What's created**:
- Full repository checkout
- Symlink: `layers/main` → main workspace's built layers
- All `.agent/` files and scripts

**Example workflow**:
```bash
# Create workspace worktree
.agent/scripts/worktree_create.sh --issue 99 --type workspace

# Enter and work
source .agent/scripts/worktree_enter.sh 99
# ... modify .agent/scripts, documentation, etc ...
# ROS environment sources from symlinked main builds

# Commit and push
git add .
git commit -m "docs: update agent workflows"
git push -u origin feature/ISSUE-99-description
```

## Commands Reference

### Create Worktree

```bash
.agent/scripts/worktree_create.sh --issue <N> --type <layer|workspace> [--layer <name>] [--branch <name>]
```

| Option | Required | Description |
|--------|----------|-------------|
| `--issue <N>` | Yes | Issue number (used for directory name) |
| `--type <type>` | Yes | `layer` or `workspace` |
| `--layer <name>` | For layer type | Which layer to work on (core, sensors, etc.) |
| `--branch <name>` | No | Custom branch name (default: `feature/ISSUE-<N>`) |

### List Worktrees

```bash
.agent/scripts/worktree_list.sh
```

Shows all active worktrees with:
- Issue number
- Type (layer/workspace)
- Branch name
- Status (clean/modified)

### Enter Worktree

```bash
source .agent/scripts/worktree_enter.sh <issue_number>
```

**Must be sourced** (not executed) because it:
- Changes current directory
- Sets environment variables
- Sources ROS environment for the worktree

Sets these variables:
- `WORKTREE_ISSUE` - The issue number
- `WORKTREE_TYPE` - "layer" or "workspace"
- `WORKTREE_ROOT` - Path to worktree directory

### Remove Worktree

```bash
.agent/scripts/worktree_remove.sh <issue_number> [--force]
```

| Option | Description |
|--------|-------------|
| `--force` | Remove even if uncommitted changes exist |

## Integration with Existing Workflows

### Starting Issue Work

The `start_issue_work.sh` script now supports worktrees:

```bash
# Traditional branch workflow
.agent/scripts/start_issue_work.sh 42 "Agent Name"

# Worktree workflow
.agent/scripts/start_issue_work.sh 42 "Agent Name" --worktree layer
```

### Build System

Build and test scripts automatically detect worktree context:

```bash
# In a layer worktree, build reports go to .scratchpad/
.agent/scripts/build.sh core_ws

# Reports stay isolated to this worktree
cat .scratchpad/build_report.md
```

### Status Report

```bash
.agent/scripts/status_report.sh
```

When run in a worktree, shows context:
```
# Workspace Status Report
**Date**: 2026-01-29
**Context**: Running in layer worktree
```

## Multi-Agent Coordination

### Recommended Pattern

1. **Each agent creates its own worktree** for assigned issues
2. **No lock contention** - worktrees are fully isolated
3. **Draft PRs** signal active work to other agents
4. **Clean up** when done to free disk space

### Example: Two Agents Working Simultaneously

**Agent A** (working on issue #42):
```bash
.agent/scripts/worktree_create.sh --issue 42 --type layer
source .agent/scripts/worktree_enter.sh 42
# Works on sensor driver...
```

**Agent B** (working on issue #43):
```bash
.agent/scripts/worktree_create.sh --issue 43 --type layer
source .agent/scripts/worktree_enter.sh 43
# Works on navigation...
```

Both agents can build, test, and commit without interference.

## Directory Layout

### Layer Worktree

```
layers/worktrees/issue-42/
├── .scratchpad/           # Isolated scratchpad for this worktree
│   ├── build_report.md
│   └── test_report.md
├── core_ws/
│   ├── src/
│   ├── build/
│   ├── install/
│   └── log/
├── sensors_ws/
│   └── ...
└── ...
```

### Workspace Worktree

```
.workspace-worktrees/issue-99/
├── .agent/
├── configs/
├── layers/               # Full layers structure
├── Makefile
├── README.md
└── ...
```

## Troubleshooting

### "Worktree already exists"

A worktree for this issue already exists. Either:
- Use it: `source .agent/scripts/worktree_enter.sh <N>`
- Remove it: `.agent/scripts/worktree_remove.sh <N>`

### "Branch already exists"

The branch is in use elsewhere. Use a custom branch name:
```bash
.agent/scripts/worktree_create.sh --issue 42 --type layer --branch feature/my-custom-name
```

### Uncommitted Changes Warning

When removing a worktree with changes:
```bash
# Check what would be lost
cd layers/worktrees/issue-42
git status
git diff

# Force removal if intended
.agent/scripts/worktree_remove.sh 42 --force
```

### ROS Environment Not Working

Make sure to **source** (not execute) worktree_enter.sh:
```bash
# Wrong - runs in subshell, changes don't persist
./agent/scripts/worktree_enter.sh 42

# Correct - sources in current shell
source .agent/scripts/worktree_enter.sh 42
```

## Best Practices

1. **One worktree per issue** - Keeps work organized
2. **Use layer worktrees for code** - Better isolation for builds
3. **Use workspace worktrees for infrastructure** - When you need to modify `.agent/`
4. **Clean up promptly** - Worktrees use disk space
5. **Create draft PR early** - Signals active work to others
6. **Commit frequently** - Worktrees don't protect against data loss

---

**Related Documentation**:
- [CLI Commands](.agent/CLI_COMMANDS.md) - Command reference
- [Workforce Protocol](.agent/WORKFORCE_PROTOCOL.md) - Multi-agent coordination
- [Git Hygiene](.agent/rules/common/git-hygiene.md) - Branch and commit practices

**Last Updated**: 2026-01-29
