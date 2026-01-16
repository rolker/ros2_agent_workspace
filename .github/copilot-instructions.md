# GitHub Copilot Instructions for ROS2 Agent Workspace

This repository manages multiple layered ROS2 workspaces configured for use with AI agents. It hosts the UNH Marine Autonomy Framework.

## Repository Structure

```
.
├── .agent/           # Agent-specific workflows, knowledge, and rules
├── configs/          # .repos files (YAML) defining workspace layers
├── scripts/          # Setup and environment sourcing scripts
└── workspaces/       # ROS2 workspaces (source code and build artifacts)
```

## Key Technologies

- **ROS 2** (Robot Operating System 2)
- **vcstool** for repository management
- **colcon** for building ROS2 workspaces
- **Layered workspace architecture** with underlay/overlay pattern

## Building and Testing

### Setup a Workspace Layer
```bash
./scripts/setup.sh <layer_name>
# Example: ./scripts/setup.sh core
```

### Build All Layers
```bash
./scripts/build.sh
```

### Source the Environment
```bash
source scripts/env.sh
```

### Check Status
```bash
./scripts/status_report.sh
```

## Important Conventions

### File Organization
- **Never place build artifacts in the root directory** - use `workspaces/<layer>_ws/`
- **Configuration files** go in `configs/` as `.repos` files
- **Helper scripts** go in `scripts/`
- **Agent-specific content** goes in `.agent/` subdirectories

### Git Hygiene
- Always use feature branches (e.g., `feature/description`)
- Make atomic commits with descriptive messages
- Do not commit build artifacts or temporary files
- Agent commits should use:
  - Name: `Antigravity Agent`
  - Email: `roland+antigravity@ccom.unh.edu`

### Code Boundaries
- **Do not modify** files within `workspaces/*/src/` unless explicitly asked
- **Do not remove** working code or tests
- **Keep changes minimal** and focused on the specific task

## Workflow Integration

This repository uses custom agent workflows defined in `.agent/workflows/`. Common workflows:
- `/add-repo` - Add a new repository to a layer
- `/build-all` - Build all layers in correct order
- `/clean` - Clean build artifacts
- `/rebuild-all` - Clean and rebuild everything
- `/submit-pr` - Create a PR for changes

See `.agent/workflows/` for the complete list.

## Layer Management

Workspace layers are defined in `configs/*.repos` files:
- `underlay.repos` - Base ROS2 dependencies
- `core.repos` - Project11, Marine AIS, Navigation tools
- `ui.repos` - RQT and visualization tools
- `sensors.repos` - Sensor-specific packages
- `platforms.repos` - Platform-specific packages
- `simulation.repos` - Simulation environments

Layers build on each other in the order defined in `scripts/env.sh`.

## Best Practices

1. **Check the ROADMAP** - Review `.agent/ROADMAP.md` before starting work to avoid conflicts
2. **Use existing tools** - Leverage vcstool and colcon commands rather than manual operations
3. **Test in layers** - Build and test individual layers before building everything
4. **Document changes** - Update relevant documentation when making structural changes
5. **Follow ROS2 conventions** - Use standard ROS2 package structure and naming

## Security

- Never commit credentials or secrets
- Do not modify security-sensitive configurations without explicit approval
- Review changes carefully before committing to shared repositories
