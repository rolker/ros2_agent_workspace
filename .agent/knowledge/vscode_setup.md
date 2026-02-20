# VS Code Setup Guide

How to use VS Code with the ROS 2 Agent Workspace. All VS Code configuration
files are gitignored — this guide describes how to generate them locally.

## Quick Start

```bash
code /path/to/ros2_agent_workspace
```

## Multi-Root Workspace (Recommended)

A multi-root workspace lets you work across the root and all layers with
per-layer settings. Create a `.code-workspace` file at the repo root:

```jsonc
// ros2_agent.code-workspace
{
  "folders": [
    { "path": ".", "name": "Workspace Root" },
    { "path": "layers/main/underlay_ws", "name": "Underlay" },
    { "path": "layers/main/core_ws", "name": "Core" },
    { "path": "layers/main/platforms_ws", "name": "Platforms" },
    { "path": "layers/main/sensors_ws", "name": "Sensors" },
    { "path": "layers/main/simulation_ws", "name": "Simulation" },
    { "path": "layers/main/ui_ws", "name": "UI" }
  ],
  "settings": {},
  "tasks": {
    "version": "2.0.0",
    "tasks": []
  }
}
```

Open it with `code ros2_agent.code-workspace`. Each layer appears as a
top-level folder in the Explorer sidebar. Layers that haven't been set up
yet will appear grayed out.

> **Note**: The `.code-workspace` file references paths under `layers/`
> which is gitignored. This file is local to your machine.

> **Tip**: The layer list is project-specific — it comes from
> `configs/manifest/layers.txt`. Adjust the folders list to match your workspace.

## Makefile Tasks

The workspace Makefile wraps common operations. To use them from VS Code,
create `.vscode/tasks.json` at the workspace root:

```jsonc
// .vscode/tasks.json
{
  "version": "2.0.0",
  "tasks": [
    {
      "label": "Build All",
      "type": "shell",
      "command": "make build",
      "group": { "kind": "build", "isDefault": true },
      "problemMatcher": "$gcc"
    },
    {
      "label": "Test All",
      "type": "shell",
      "command": "make test",
      "group": "test"
    },
    {
      "label": "Lint",
      "type": "shell",
      "command": "make lint",
      "problemMatcher": []
    },
    {
      "label": "Status",
      "type": "shell",
      "command": "make status",
      "problemMatcher": []
    },
    {
      "label": "Clean",
      "type": "shell",
      "command": "make clean",
      "problemMatcher": []
    }
  ]
}
```

- **Ctrl+Shift+B** runs the default build task (`make build`)
- **Ctrl+Shift+P > Tasks: Run Task** shows all available tasks

## C++ IntelliSense

The build script generates `compile_commands.json` for each CMake-based **package**
(via `CMAKE_EXPORT_COMPILE_COMMANDS=ON`). After building, CMake packages will have files at
`build/<package>/compile_commands.json` (Python-only packages will not).

To merge them into a single file for IntelliSense, run from a layer
workspace (e.g., `layers/main/core_ws`):

```bash
mapfile -t compdb_files < <(find build -name compile_commands.json -print)
if [ "${#compdb_files[@]}" -gt 0 ]; then
  jq -s 'add' "${compdb_files[@]}" > build/compile_commands.json
else
  echo "No compile_commands.json files found. Build CMake packages first."
fi
```

Then create `.vscode/c_cpp_properties.json` in the layer directory
(e.g., `layers/main/core_ws/.vscode/c_cpp_properties.json`):

```jsonc
{
  "configurations": [
    {
      "name": "ROS 2",
      "compileCommands": "${workspaceFolder}/build/compile_commands.json",
      "intelliSenseMode": "linux-gcc-x64"
    }
  ],
  "version": 4
}
```

Re-run the `jq` merge after each build, or add it as a post-build
VS Code task. Verify per-package files exist after building:

```bash
ls layers/main/core_ws/build/*/compile_commands.json
```

## Python IntelliSense

For Python ROS packages, add the install space to your analysis paths.
With `colcon build --symlink-install` (the default), packages are installed
at `install/<package>/lib/python3.XX/site-packages/`. In the layer's
`.vscode/settings.json`:

```jsonc
{
  "python.analysis.extraPaths": [
    "${workspaceFolder}/install/*/lib/python3.12/site-packages"
  ]
}
```

To discover the correct Python version for your environment:

```bash
ls install/*/lib/python*/site-packages/ 2>/dev/null | head -1
```

Adjust `python3.12` to match your distribution if different.

## Claude Code Integration

The [Claude Code VS Code extension](https://marketplace.visualstudio.com/items?itemName=anthropic.claude-code)
integrates directly into the editor:

1. Install the **Claude Code** extension from the VS Code marketplace
2. The extension shares configuration and history with the CLI
3. Use `Ctrl+Esc` to toggle focus between the editor and Claude

Both the extension and the CLI terminal can be used simultaneously.
Conversation history is shared — use `claude --resume` in the terminal
to continue an extension conversation.

## Recommended Extensions

- **C/C++** (`ms-vscode.cpptools`) — IntelliSense, debugging
- **Python** (`ms-python.python`) — Python language support
- **Claude Code** (`anthropic.claude-code`) — AI assistant integration
- **XML** (`redhat.vscode-xml`) — For `package.xml` and launch files
- **YAML** (`redhat.vscode-yaml`) — For ROS config files

## Working in Worktrees

Open each worktree in its own VS Code window — don't add worktree
folders to the main workspace. VS Code's source control targets whichever
`.git` root it detects first, so mixing worktrees and the main checkout
in one window causes commits to land in the wrong place.

```bash
source .agent/scripts/worktree_enter.sh --issue <N>
code .
```

### Workspace worktrees

Workspace worktrees (`.workspace-worktrees/issue-workspace-<N>/`) are full
checkouts of the workspace repo with `layers/main` symlinked back to the
main tree. The `.code-workspace` file works here because the layer paths
resolve through the symlink.

### Layer worktrees

Layer worktrees (`layers/worktrees/issue-<slug>-<N>/`) use a hybrid layout:
real git worktrees for modified project repos and symlinks for the rest.
Open `code .` from the worktree root; the multi-root `.code-workspace` file
doesn't apply here since only a subset of packages are checked out.

### IntelliSense paths

`compile_commands.json` records absolute paths at build time. If you build
in the main tree but edit in a worktree (or vice versa), IntelliSense won't
resolve headers. Always build in the same context you're editing:

```bash
# Inside the worktree's layer directory
cd core_ws && colcon build --symlink-install
```

### File watching and search

VS Code follows symlinks by default, which causes duplicate search results
and high file-watcher load in worktrees. Add these to the layer-level
`.vscode/settings.json` (or the `.code-workspace` settings block):

```jsonc
{
  "files.watcherExclude": {
    "**/build/**": true,
    "**/install/**": true,
    "**/log/**": true
  },
  "search.exclude": {
    "**/build/**": true,
    "**/install/**": true,
    "**/log/**": true
  }
}
```
