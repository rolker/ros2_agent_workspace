# Contributing to ROS2 Agent Workspace

Thank you for your interest in contributing! This document provides guidelines for contributing to the workspace.

## Code of Conduct

We are committed to providing a welcoming and inclusive environment. Please be respectful and professional in all interactions.

## Getting Started

1. **Fork the repository** on GitHub
2. **Clone your fork** locally:
   ```bash
   git clone git@github.com:YOUR-USERNAME/ros2_agent_workspace.git
   cd ros2_agent_workspace
   ```
3. **Set up the development environment**:
   ```bash
   ./.agent/scripts/bootstrap.sh
   ./.agent/scripts/setup.sh core
   ```

## Agent Rules
> [!IMPORTANT]
> Since we use AI Agents extensively, we have codified rules in `.agent/rules/`.
> - **Planning**: See [PLANNING_MODE.md](.agent/rules/PLANNING_MODE.md) before starting complex tasks.
> - **Execution**: See [EXECUTION_MODE.md](.agent/rules/EXECUTION_MODE.md) for coding and verification standards.

## Development Workflow

### Creating a Feature Branch

Follow the git hygiene rules in `.agent/rules/git-hygiene.md` and [EXECUTION_MODE.md](.agent/rules/EXECUTION_MODE.md):

- Features: `feature/TASK-<ID>-<description>` (e.g., `feature/TASK-001-multi-distro`)
- Fixes: `fix/<description>`
- Never commit directly to `main` or `jazzy`

```bash
git checkout -b feature/TASK-123-add-new-layer
```

### Making Changes

1. **Keep the workspace root clean** - See `.agent/rules/clean-root.md`
2. **Run builds in layer directories**, not the root
3. **Test your changes**:
   ```bash
   ./.agent/scripts/build.sh
   ./.agent/scripts/verify_change.sh --package <modified_pkg> --type lint
   ./.agent/scripts/test.sh
   ```

### Code Quality

#### Shell Scripts

- All shell scripts should pass `shellcheck`
- Use `set -e` for error handling where appropriate
- Add descriptive comments for complex logic
- Include usage examples in script headers

#### Python Scripts

- Follow PEP 8 style guidelines
- Add docstrings to functions and classes
- Use type hints where appropriate
- Handle errors gracefully

### Testing

- Add tests for new functionality when appropriate
- Ensure existing tests pass: `./.agent/scripts/test.sh`
- Test on a clean workspace if possible

### Documentation

- Update README.md if adding new features
- Add/update workflow documentation in `.agent/workflows/` if needed
- Include usage examples for new scripts
- Update this CONTRIBUTING.md if changing development processes

## Submitting Changes

1. **Commit your changes** with clear, descriptive messages:
   ```bash
   git add .
   git commit -m "Add support for new sensor layer"
   ```

2. **Push to your fork**:
   ```bash
   git push origin feature/TASK-123-add-new-layer
   ```

3. **Create a Pull Request** on GitHub:
   - Provide a clear title and description
   - Reference any related issues
   - Explain what changed and why
   - Include testing steps if applicable

4. **Respond to review feedback** promptly and professionally

## Pull Request Guidelines

- Keep PRs focused on a single feature or fix
- Include tests if adding new functionality
- Update documentation as needed
- Ensure CI checks pass (once CI is set up)
- Link to related issues using GitHub keywords (Fixes #123, Closes #456)

## Repository Structure

```
.
├── .agent/              # Agent-specific workflows and knowledge
│   ├── workflows/       # Agentic workflows (slash commands)
│   ├── rules/          # Always-on rules for agents
│   └── knowledge/      # Generated knowledge links
├── configs/            # .repos files defining workspace layers
├── scripts/            # Helper scripts for setup, build, test
└── workspaces/         # ROS2 workspaces (gitignored, generated)
```

## Adding New Layers

To add a new workspace layer:

1. Create `configs/<layer-name>.repos` with repository definitions
2. Update `LAYERS` array in `.agent/scripts/env.sh` if the layer should be auto-sourced (note: this is now configured via `layers.txt`)
3. Document the layer in README.md
4. Test setup: `./.agent/scripts/setup.sh <layer-name>`

## Questions or Issues?

- Check existing issues on GitHub
- Review the README.md and documentation
- Ask questions in pull request discussions
- For security issues, see SECURITY.md

## License

By contributing, you agree that your contributions will be licensed under the same license as the project (see LICENSE file if present, or consult repository owner).
