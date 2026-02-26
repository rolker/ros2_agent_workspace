# Architecture Documentation

## Overview

The ROS 2 Agent Workspace is a **layered workspace management system** designed for complex ROS 2 projects. It is developed by the Center for Coastal and Ocean Mapping / Joint Hydrographic Center (CCOM/JHC) at the University of New Hampshire (UNH) and pre-configured for the UNH Marine Autonomy Framework. The workspace supports AI-driven development through structured workflows and knowledge management.

## Core Concepts

### Layered Workspaces

The workspace uses a hierarchical overlay system where each layer builds upon previous layers:

```
┌─────────────────────────────────────┐
│         UI Layer (ui_ws)            │  ← Top overlay (visualization)
├─────────────────────────────────────┤
│    Simulation Layer (simulation_ws) │  ← Simulation tools
├─────────────────────────────────────┤
│     Sensors Layer (sensors_ws)      │  ← Sensor drivers & perception
├─────────────────────────────────────┤
│    Platforms Layer (platforms_ws)   │  ← Platform-specific code
├─────────────────────────────────────┤
│      Core Layer (core_ws)           │  ← Main autonomy framework
├─────────────────────────────────────┤
│    Underlay Layer (underlay_ws)     │  ← Additional dependencies
├─────────────────────────────────────┤
│         ROS 2 Jazzy Base            │  ← Foundation
└─────────────────────────────────────┘
```

**Benefits of layered approach:**
- **Modularity**: Each layer can be developed/tested independently
- **Incremental Building**: Build only what you need
- **Clear Dependencies**: Lower layers don't depend on upper layers
- **Team Organization**: Different teams can own different layers

### Source Order

The `.agent/scripts/env.sh` sources workspaces in this order:
1. ROS 2 Jazzy base (`/opt/ros/jazzy`)
2. Underlay workspace
3. Core workspace
4. Platforms workspace
5. Sensors workspace
6. Simulation workspace
7. UI workspace

This ensures that packages in higher layers can override or extend packages in lower layers.

## Directory Structure

```
ros2_agent_workspace/
├── .agent/                    # Agent infrastructure
│   ├── scripts/              # Automation scripts (setup, build, test, etc.)
│   ├── templates/            # Templates for issues and tests
│   ├── knowledge/            # ROS 2 patterns and CLI best practices
│   ├── hooks/                # Git hooks (pre-commit)
│   ├── instructions/         # Framework-specific instruction files
│   └── project_knowledge     # -> manifest repo's agent_context/ (symlink, gitignored)
│
├── configs/                   # Configuration and bootstrap
│   ├── project_bootstrap.url # URL to the manifest repo's bootstrap.yaml
│   ├── manifest              # -> manifest repo's config dir (symlink, gitignored)
│   │   ├── layers.txt        #   Layer ordering
│   │   └── repos/            #   .repos files per layer
│   └── manifest_repo/        # Manifest repo clone for Pattern A (gitignored)
│
├── layers/                    # Generated layer directories (gitignored)
│   └── main/
│       ├── underlay_ws/      # ROS dependencies overlay
│       ├── core_ws/          # Core autonomy packages
│       ├── platforms_ws/     # Platform-specific code
│       ├── sensors_ws/       # Sensor packages
│       ├── simulation_ws/    # Simulation tools
│       └── ui_ws/            # Visualization & user interfaces
│           ├── src/          # Source repositories (from vcs import)
│           ├── build/        # Build artifacts
│           ├── install/      # Install space
│           └── log/          # Build/test logs
│
└── .agent/scratchpad/         # Agent working directory (gitignored)
    ├── build_report.md        # Latest build status
    ├── test_report.md         # Latest test results
    ├── workspace.lock         # Multi-agent coordination lock
    └── *.md                   # Other generated reports
```

### Manifest Repo

The **manifest repo** is the repository that provides layer definitions (`.repos` files) and `layers.txt`. It is referenced via `configs/project_bootstrap.url` and cloned during `setup.sh`. The workspace supports two patterns:

- **Pattern A (standalone manifest)**: The manifest repo contains only configuration (no ROS packages). It is cloned to `configs/manifest_repo/<name>/` and `configs/manifest` symlinks to its config directory.
- **Pattern B (manifest with ROS packages)**: The manifest repo also contains ROS packages (e.g., `unh_marine_autonomy`). It is cloned into a layer's `src/` directory (e.g., `layers/main/core_ws/src/`) so colcon discovers the packages. `configs/manifest` symlinks to the config subdirectory within that clone.

The `layer` field in `bootstrap.yaml` controls the pattern: present for Pattern B, absent for Pattern A.

## Agent System

### Framework Instruction Files

Shared workspace rules live in **`AGENTS.md`** at the repo root (see ADR-0006). Each
AI agent framework has a thin adapter file that imports `AGENTS.md` and adds only
framework-specific configuration:

- **`CLAUDE.md`**: Claude Code adapter (auto-loaded, imports `AGENTS.md` via `@` syntax)
- **`.github/copilot-instructions.md`**: GitHub Copilot adapter
- **`.agent/instructions/gemini-cli.instructions.md`**: Gemini CLI adapter
- **`.agent/AGENT_ONBOARDING.md`**: Onboarding for unknown/containerized agents

Rules in `AGENTS.md` are organized using a three-tier boundary system
(Always / Ask First / Never) to make agent autonomy boundaries explicit.

## Build System

### Colcon Integration

Each workspace uses `colcon` as the build tool with:
- `--symlink-install`: Allows in-place editing of Python/launch files
- Event-based logging for detailed build reports

### Build Reports

The `build.sh` script generates a markdown report (`.agent/scratchpad/build_report.md`) showing:
- Number of packages per layer
- Success/failure status
- Warnings and errors
- Failed package names

This is parsed from colcon's `events.log` by `build_report_generator.py`.

## Knowledge Management

### Two-Tier Knowledge Model

Agent knowledge is split into workspace-level and project-level tiers:

1. **Workspace knowledge** (`.agent/knowledge/`): Version-controlled files that apply to all projects using this workspace. Contains ROS 2 development patterns, CLI best practices, and documentation verification workflows.

2. **Project knowledge** (`.agent/project_knowledge`): A symlink (gitignored) pointing to the manifest repo's `agent_context/` directory, if it exists. Contains project-specific conventions, architecture documentation, and component guides. This is created automatically by `setup.sh` during bootstrap.

## Coordination & Locking

### Git Worktrees (Recommended for Parallel Work)

For scenarios where multiple agents or developers work simultaneously, the workspace supports **git worktrees** for complete isolation:

```
ros2_agent_workspace/           # Main workspace
├── layers/
│   ├── core_ws/               # Shared layers
│   └── worktrees/             # Layer worktrees
│       ├── issue-42/          # Isolated checkout for issue 42
│       └── issue-43/          # Isolated checkout for issue 43
└── .workspace-worktrees/      # Workspace worktrees
    └── issue-99/              # Full repo checkout for infrastructure work
```

**Benefits of worktrees**:
- Each worktree has isolated build artifacts
- No lock contention between agents
- Switch between issues without stashing
- Parallel builds and tests

**Commands**:
- Create: `.agent/scripts/worktree_create.sh --issue N --type layer|workspace`
- List: `.agent/scripts/worktree_list.sh`
- Enter: `source .agent/scripts/worktree_enter.sh N`
- Remove: `.agent/scripts/worktree_remove.sh N`

See [WORKTREE_GUIDE.md](.agent/WORKTREE_GUIDE.md) for full documentation.

### Multi-Agent Locking (Fallback)

The workspace supports a locking mechanism (`.agent/scripts/lock.sh`, `.agent/scripts/unlock.sh`) to prevent conflicts when multiple agents or processes might work simultaneously:

- Creates `.agent/scratchpad/workspace.lock` with:
  - User/Agent ID
  - Reason for lock
  - Timestamp
- `build.sh` and `test.sh` check for locks before running

### Status Reporting

`.agent/scripts/status_report.sh` generates a comprehensive report using `vcs` and `gh` to show:
- Root repository status
- All sub-repository statuses
- Modified files
- Branch information
- Ahead/behind tracking
- Open GitHub PRs and issues (use `--skip-github` or `--quick` for offline mode)
- PR comment triage (with `--pr-triage`)

## VCS Tool Integration

The workspace heavily uses `vcstool` (`vcs` command) for managing multiple repositories:

- `.repos` files define repositories in YAML format
- `vcs import`: Clone/update repositories from `.repos` file
- `vcs status`: Check status across all repos
- `vcs custom`: Run custom git commands across repos

## Extension Points

### Adding a New Layer

1. Add a `new_layer.repos` file to the manifest repo's config directory (default: `config/repos/`)
2. Add the layer name to `config/layers.txt` in the manifest repo
3. Run `./.agent/scripts/setup.sh new_layer`
4. The layer will be automatically sourced by `env.sh`

### Bootstrapping from a Custom Project

1. Create a `bootstrap.yaml` in your project repo:
   ```yaml
   git_url: https://github.com/your-org/your-repo.git
   branch: main
   # layer: core          # Include if repo has ROS packages (Pattern B)
   # config_path: config  # Defaults to "config" if omitted
   ```
2. Update `configs/project_bootstrap.url` to point to your `bootstrap.yaml`
3. Run `./.agent/scripts/setup.sh`

### Adding New Scripts

- Place in `.agent/scripts/` directory
- Make executable: `chmod +x .agent/scripts/new_script.sh`
- Follow existing patterns (lock checking, error handling)
- Update CONTRIBUTING.md with usage

## Design Principles

1. **Separation of Concerns**: Configuration (configs/), automation (.agent/scripts/), intelligence (.agent/)
2. **GitOps**: Workspace structure defined in version control, generated artifacts are gitignored
3. **Reproducibility**: Bootstrap and setup scripts ensure consistent environments
4. **Agent-Friendly**: Clear structure, comprehensive documentation, automation scripts
5. **Developer-Friendly**: Works for humans too, not just agents
6. **Layered Complexity**: Simple tasks are simple, complex tasks are possible

## Technology Stack

- **ROS 2 Jazzy**: Base robotics framework
- **Colcon**: Build tool for ROS 2 packages
- **VCS Tool**: Multi-repository management
- **Bash**: Automation scripts
- **Python 3**: Utilities and parsers
- **YAML**: Configuration format for .repos files
- **Markdown**: Documentation and agent instruction files

## Future Considerations

Potential areas for enhancement:
- CI/CD integration (GitHub Actions, GitLab CI)
- Docker containers for reproducible builds
- Cross-platform support (currently Ubuntu/Debian focused)
- Automated dependency tracking between layers
- Performance metrics and build time optimization
