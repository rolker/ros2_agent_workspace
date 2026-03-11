# Git Worktree Guide

**Purpose**: Enable parallel development through isolated working directories. Each worktree has its own builds, tests, and uncommitted changes—completely separate from the main workspace.

## Quick Start

```bash
# Create a worktree for issue #42 to work on the core layer
.agent/scripts/worktree_create.sh --issue 42 --type layer --layer core --packages my_package

# Enter the worktree (sources ROS environment)
source .agent/scripts/worktree_enter.sh 42

# Or set up manually (layer worktrees generate convenience scripts)
cd layers/worktrees/issue-workspace-42/
source setup.bash

# Build and test
./core_ws/build.sh my_package
./core_ws/test.sh my_package
git add . && git commit -m "feat: my changes"

# When done, clean up
.agent/scripts/worktree_remove.sh 42

# Skill worktrees (no GitHub issue needed)
.agent/scripts/worktree_create.sh --skill research --type workspace
source .agent/scripts/worktree_enter.sh --skill research
# ... make changes, commit, push, create PR ...
.agent/scripts/worktree_remove.sh --skill research
```

## Generated Convenience Scripts

When a **layer worktree** is created, `worktree_create.sh` generates self-contained
scripts in the **target (non-symlink) layer workspace** so you can build and test
without knowing about `worktree_enter.sh` or `ROS2_LAYERS_BASE`. These are generated
once at worktree creation time; only new worktrees get them. Symlinked layers (which
point to `layers/main/`) are skipped to avoid polluting the shared workspace.

### What's generated

| File | Purpose |
|------|---------|
| `setup.bash` | Sources ROS 2 base + all layer overlays, exports worktree env vars, installs git guardrails |
| `<target_layer>_ws/build.sh` | Sources lower layers, runs `colcon build` (pass package names to build selectively) |
| `<target_layer>_ws/test.sh` | Sources lower layers, runs `colcon test` + `colcon test-result --verbose` |
| `<target_layer>_ws/colcon/defaults.yaml` | Default colcon flags (`--symlink-install`, compile commands); auto-discovered by colcon from `$PWD/colcon/defaults.yaml` |

### Usage

```bash
cd layers/worktrees/issue-workspace-42/
source setup.bash                  # Set up ROS environment

# Build everything in the core layer
./core_ws/build.sh

# Build a specific package
./core_ws/build.sh my_package

# Test a specific package
./core_ws/test.sh my_package
```

**Note**: `colcon_defaults.yaml` is placed in the target layer workspace so raw
`colcon build` from that directory also picks up the correct flags.

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
    └── issue-workspace-42/    # Layer worktree for issue 42 (workspace repo)
        ├── core_ws/           # Hybrid structure (see below)
        │   └── src/
        │       ├── unh_marine_autonomy/  # Git worktree (modified)
        │       ├── camp/ -> ../../../../main/core_ws/src/camp/  # Symlink
        │       └── ...
        ├── underlay_ws -> ../../main/underlay_ws  # Symlink
        ├── platforms_ws -> ../../main/platforms_ws
        └── .scratchpad/       # Isolated scratchpad
    └── issue-marine_msgs-5/   # Layer worktree for issue 5 (marine_msgs repo)
        ├── core_ws/           # Hybrid structure
        └── ...

.workspace-worktrees/
└── issue-workspace-137/       # Workspace worktree (infrastructure work)
    ├── .agent/                # Real - root repo worktree
    ├── configs/               # Real - root repo worktree
    └── layers/                # Real - root repo worktree
        └── main -> ../../layers/main  # Symlink - read-only access to main builds
```

**Note**: Worktrees are now named `issue-{REPO_SLUG}-{NUMBER}` to prevent collisions between same issue numbers from different repositories. The workspace repo uses "workspace" as its slug.

## Worktree Types

### Layer Worktrees (`--type layer --layer <name> --packages <pkg,...>`)

**Location**: `layers/worktrees/issue-{REPO_SLUG}-{NUMBER}/`

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

**Naming**: The `{REPO_SLUG}` is auto-detected from the repository where the issue was created. For the main workspace repository, it uses "workspace". For package repositories (e.g., `marine_msgs`), it uses the package name.

**Benefits**:
- Disk efficiency: Only modified packages have real checkouts
- Build efficiency: Symlinked packages reuse the main workspace's source tree; each worktree keeps its own build/install artifacts
- Isolation: Git worktrees allow independent changes

**Example workflow**:
```bash
# Create layer worktree for single package (workspace issue #42)
.agent/scripts/worktree_create.sh --issue 42 --type layer --layer core --packages unh_marine_autonomy
# Creates: layers/worktrees/issue-workspace-42/

# For multiple packages (coordinated changes)
.agent/scripts/worktree_create.sh --issue 42 --type layer --layer core --packages unh_marine_autonomy,camp

# Or for a package repo issue (marine_msgs #5)
.agent/scripts/worktree_create.sh --issue 5 --type layer --layer core --packages marine_msgs --repo-slug marine_msgs
# Creates: layers/worktrees/issue-marine_msgs-5/

# Enter and work
source .agent/scripts/worktree_enter.sh --issue 42

# If multiple worktrees exist for issue #42 (from different repos),
# specify the repository slug:
source .agent/scripts/worktree_enter.sh --issue 42 --repo-slug marine_msgs
cd core_ws/src/my_package
# ... make changes ...
cd ../.. && ./build.sh my_package

# Test from the layer workspace
./test.sh my_package

# Commit and push
git add .
git commit -m "feat: add new feature"
git push -u origin feature/ISSUE-42-description
```

### Workspace Worktrees (`--type workspace`)

**Location**: `.workspace-worktrees/issue-{REPO_SLUG}-{NUMBER}/`

**Use for**: Infrastructure changes, documentation, `.agent/` modifications

**What's created**:
- Full repository checkout
- Symlink: `layers/main` → main workspace's built layers
- All `.agent/` files and scripts

**Naming**: Same as layer worktrees - includes repository slug to prevent collisions.

**Example workflow**:
```bash
# Create workspace worktree (workspace issue #99)
.agent/scripts/worktree_create.sh --issue 99 --type workspace
# Creates: .workspace-worktrees/issue-workspace-99/

# Enter and work
source .agent/scripts/worktree_enter.sh 99
# ... modify .agent/scripts, documentation, etc ...
# ROS environment sources from symlinked main builds

# Commit and push
git add .
git commit -m "docs: update agent workflows"
git push -u origin feature/ISSUE-99-description
```

### Skill Worktrees (`--skill <name>`)

Some skills (e.g., `research`) maintain living documents that need worktree isolation
and PR review but don't warrant a dedicated GitHub issue for each recurring update.
These use `--skill <name>` instead of `--issue <N>`.

**Location**: Same as issue worktrees — `.workspace-worktrees/skill-{REPO_SLUG}-{ID}/`
or `layers/worktrees/skill-{REPO_SLUG}-{ID}/`

**Allowed skills**: Only skills in the allowlist (defined in `worktree_create.sh`) can
create skill worktrees. Currently: `research`.

**Naming**: `skill-{REPO_SLUG}-{SKILL_NAME}-{TIMESTAMP}` (e.g.,
`skill-workspace-research-20260227-143022-123456789`)

**Branch naming**: `skill/{SKILL_NAME}-{TIMESTAMP}` (e.g.,
`skill/research-20260227-143022-123456789`)

**Environment variables**: `WORKTREE_SKILL` is set instead of `WORKTREE_ISSUE`.

**Example workflow**:
```bash
# Create skill worktree
.agent/scripts/worktree_create.sh --skill research --type workspace

# Enter it
source .agent/scripts/worktree_enter.sh --skill research

# Work, commit, push, create PR
git add . && git commit -m "docs: update research digest"
git push -u origin HEAD && gh pr create --fill

# Clean up
.agent/scripts/worktree_remove.sh --skill research
```

**Note**: `--issue` and `--skill` are mutually exclusive. All other worktree rules
(atomic commits, AI signature, pre-commit hooks, PR review) still apply.

### Sub-issue Worktrees (`--parent-issue <N>`)

When creating a worktree for a sub-task of an existing issue, use `--parent-issue`
to branch from the parent's feature branch and target the draft PR at it (stacked PR).

**Example workflow**:
```bash
# Parent issue #100 already has a worktree on feature/issue-100
# Create a sub-issue worktree that branches from the parent
.agent/scripts/worktree_create.sh --issue 101 --type workspace --parent-issue 100

# Enter the sub-issue worktree
source .agent/scripts/worktree_enter.sh --issue 101

# Work, commit, push — the draft PR targets feature/issue-100
```

**Environment**: `WORKTREE_PARENT_ISSUE` is set in the generated `setup.bash`,
making it available to scripts like `gh_create_issue.sh` (which auto-injects
"Part of #N" into new issues created from within the worktree).

**Fallback**: If the parent branch (`feature/issue-<N>`) doesn't exist locally or
on the remote, the worktree falls back to branching from HEAD with a warning.

## Commands Reference

### Create Worktree

```bash
# For layer worktrees (required: --layer and --packages)
.agent/scripts/worktree_create.sh --issue <N> --type layer --layer <name> --packages <pkg1,pkg2,...>

# For workspace worktrees
.agent/scripts/worktree_create.sh --issue <N> --type workspace [--branch <name>]

# For skill worktrees (no issue needed)
.agent/scripts/worktree_create.sh --skill <name> --type workspace
```

| Option | Required | Description |
|--------|----------|-------------|
| `--issue <N>` | Yes (unless `--skill`) | Issue number (used for directory name) |
| `--skill <name>` | Yes (unless `--issue`) | Skill name from allowlist (alternative to `--issue`) |
| `--type <type>` | Yes | `layer` or `workspace` |
| `--layer <name>` | For layer type | Which layer to work on (core, sensors, etc.) |
| `--packages <pkg1,pkg2,...>` | For layer type | Comma-separated list of packages to include as worktrees |
| `--parent-issue <N>` | No | Parent issue number; branches from parent's feature branch and targets PR at it |
| `--branch <name>` | No | Custom branch name (default: `feature/ISSUE-<N>` or `skill/<id>`) |

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
source .agent/scripts/worktree_enter.sh --skill <name>
```

**Must be sourced** (not executed) because it:
- Changes current directory
- Sets environment variables
- Sources ROS environment for the worktree

Sets these variables:
- `WORKTREE_ISSUE` - The issue number (issue mode only)
- `WORKTREE_SKILL` - The skill name (skill mode only)
- `WORKTREE_PARENT_ISSUE` - Parent issue number (when `--parent-issue` was used)
- `WORKTREE_TYPE` - "layer" or "workspace"
- `WORKTREE_ROOT` - Path to worktree directory

### Remove Worktree

```bash
.agent/scripts/worktree_remove.sh <issue_number> [--force]
.agent/scripts/worktree_remove.sh --skill <name> [--force]
```

| Option | Description |
|--------|-------------|
| `--force` | Remove even if uncommitted changes exist |

## Integration with Existing Workflows

### Starting Issue Work

Use the worktree scripts to start work on an issue:

```bash
# Workspace infrastructure work
.agent/scripts/worktree_create.sh --issue 42 --type workspace
source .agent/scripts/worktree_enter.sh --issue 42

# ROS package work
.agent/scripts/worktree_create.sh --issue 42 --type layer --layer core --packages <pkg>
source .agent/scripts/worktree_enter.sh --issue 42
```

### Build System

Build and test scripts automatically detect worktree context:

```bash
# In a layer worktree, build reports go to .scratchpad/
.agent/scripts/build.sh core_ws

# Reports stay isolated to this worktree
cat .scratchpad/build_report.md
```

### Dashboard

```bash
.agent/scripts/dashboard.sh              # Full status (sync + GitHub)
.agent/scripts/dashboard.sh --quick      # Fast local-only check
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
.agent/scripts/worktree_create.sh --issue 42 --type layer --layer core --packages sensor_driver
source .agent/scripts/worktree_enter.sh 42
# Works on sensor driver...
```

**Agent B** (working on issue #43):
```bash
.agent/scripts/worktree_create.sh --issue 43 --type layer --layer core --packages navigation
source .agent/scripts/worktree_enter.sh 43
# Works on navigation...
```

Both agents can build, test, and commit without interference.

## Directory Layout

### Layer Worktree

```
layers/worktrees/issue-workspace-42/
├── setup.bash             # Generated: sources ROS env for this worktree
├── .scratchpad/           # Isolated scratchpad for this worktree
│   ├── build_report.md
│   └── test_report.md
├── core_ws/
│   ├── build.sh           # Generated: build this layer
│   ├── test.sh            # Generated: test this layer
│   ├── colcon/
│   │   └── defaults.yaml  # Generated: colcon flags (auto-discovered)
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
.agent/scripts/worktree_create.sh --issue 42 --type layer --layer core --packages my_package --branch feature/my-custom-name
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

## Migration from Old Naming Scheme

**Old format**: `issue-{NUMBER}` (e.g., `issue-42`)  
**New format**: `issue-{REPO_SLUG}-{NUMBER}` (e.g., `issue-workspace-42`, `issue-marine_msgs-5`)

### Why the Change?

The old naming scheme (`issue-{NUMBER}`) caused collisions when working on issues from different repositories with the same issue number. For example, issue #5 from `marine_msgs` and issue #5 from `sensor_driver` would try to use the same worktree path.

The new naming scheme (`issue-{REPO_SLUG}-{NUMBER}`) eliminates this problem by including the repository context in the directory name.

### Repository Slug Sanitization

Repository names may contain characters (like hyphens) that are not suitable for directory names or regex parsing. The scripts automatically sanitize repository slugs by replacing all non-alphanumeric characters (except underscores) with underscores.

**Examples**:
- `my-repo-name` → `my_repo_name`
- `ros2-driver` → `ros2_driver`

### Handling Multiple Worktrees for the Same Issue

When multiple worktrees exist for the same issue number from different repositories, the helper scripts will detect the ambiguity:

```bash
$ source .agent/scripts/worktree_enter.sh 42
Error: Multiple worktrees found for issue 42:
  - issue-workspace-42
  - issue-marine_msgs-42

Use --repo-slug to specify which one:
  source .agent/scripts/worktree_enter.sh --issue 42 --repo-slug workspace
  source .agent/scripts/worktree_enter.sh --issue 42 --repo-slug marine_msgs
```

Use the `--repo-slug` parameter to disambiguate:

```bash
source .agent/scripts/worktree_enter.sh --issue 42 --repo-slug marine_msgs
.agent/scripts/worktree_remove.sh --issue 42 --repo-slug workspace
```

---

**Related Documentation**:
- [AI Rules](AI_RULES.md) - Universal agent rules
- [Workforce Protocol](WORKFORCE_PROTOCOL.md) - Multi-agent coordination

**Last Updated**: 2026-03-06
