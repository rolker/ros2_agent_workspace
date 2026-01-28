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

### For AI CLI Agents (Copilot CLI, Gemini CLI)
- **Quick Start**: See [`.agent/AI_CLI_QUICKSTART.md`](.agent/AI_CLI_QUICKSTART.md) for 5-minute setup
- **Universal Rules**: [`.agent/AI_RULES.md`](.agent/AI_RULES.md) - Single source of truth for all agents
- **Command Reference**: [`.agent/CLI_COMMANDS.md`](.agent/CLI_COMMANDS.md) - Workflow discovery

### For All Agents
- **Planning**: See [PLANNING_MODE.md](.agent/rules/PLANNING_MODE.md) before starting complex tasks.
- **Execution**: See [EXECUTION_MODE.md](.agent/rules/EXECUTION_MODE.md) for coding and verification standards.

### Documentation Maintenance

**Important**: The agent documentation follows a layered structure to avoid duplication:

1. **`.agent/AI_RULES.md`** - Universal rules for ALL agents (single source of truth)
2. **`.agent/AI_CLI_QUICKSTART.md`** - Fast path for CLI agents
3. **`.agent/CLI_COMMANDS.md`** - Command mapping and discovery
4. **Framework-specific overlays**:
   - `.github/copilot-instructions.md` - Copilot CLI repository-wide instructions
   - `.github/instructions/*.instructions.md` - Path-specific instructions with YAML frontmatter
   - `.agent/instructions/gemini-cli.instructions.md` - Gemini CLI references
   - `.agent/AGENT_ONBOARDING.md` - Specialized/container agents

**GitHub Copilot Instruction Structure** (follows [best practices](https://docs.github.com/en/copilot/how-tos/configure-custom-instructions/add-repository-instructions)):
- **Repository-wide**: `.github/copilot-instructions.md` - applies to all files
- **Path-specific**: `.github/instructions/*.instructions.md` - use `applies_to:` glob patterns in YAML frontmatter

**When updating agent documentation**:
- ✅ Update `AI_RULES.md` for universal changes (applies to all agents)
- ✅ Update framework-specific files only for platform-specific features
- ✅ Include concrete code examples (not just descriptions)
- ✅ Include build/test commands and version information
- ✅ Verify YAML frontmatter is valid
- ❌ Don't duplicate universal rules across files - link to `AI_RULES.md` instead

**Documentation Owner**: Framework Engineering Team

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
│   ├── scripts/        # Helper scripts for setup, build, test
│   └── knowledge/      # Generated knowledge links
├── configs/            # .repos files defining workspace layers
└── workspaces/         # ROS2 workspaces (gitignored, generated)
```

## Adding New Layers

To add a new workspace layer:

1. Create `configs/<layer-name>.repos` with repository definitions
2. Update `LAYERS` array in `.agent/scripts/env.sh` if the layer should be auto-sourced (note: this is now configured via `layers.txt`)
3. Document the layer in README.md
4. Test setup: `./.agent/scripts/setup.sh <layer-name>`

## Reporting Infrastructure Friction

**For AI Agents**: Help improve the workspace by reporting friction points you encounter!

If you experience issues with agent infrastructure (documentation, workflows, scripts), use the **Continuous Improvement workflow**:

```bash
# See the workflow guide
cat .agent/workflows/ops/continuous-improvement.md

# Use the issue template
cat .agent/templates/improvement_issue.md
```

**What to report:**
- 📚 Unclear or missing documentation
- 🔧 Scripts that fail or require workarounds
- 🔍 Hard-to-discover tools or workflows
- 🔄 Repetitive manual tasks that could be automated
- ⚠️ Confusing error messages
- 🌍 Environment/setup issues

**When to report:**
- At the end of work sessions
- After completing major tasks
- When you notice repeated friction

**How to report:**
1. Check for existing issues: `gh issue list --search "your topic"`
2. Draft issue using `.agent/templates/improvement_issue.md`
3. Include concrete examples and session evidence
4. Create issue: `gh issue create --label "enhancement,agent-infrastructure"`

See [`.agent/workflows/ops/continuous-improvement.md`](.agent/workflows/ops/continuous-improvement.md) for the complete process.

## Questions or Issues?

- Check existing issues on GitHub
- Review the README.md and documentation
- Ask questions in pull request discussions
- For security issues, see SECURITY.md

## License

By contributing, you agree that your contributions will be licensed under the same license as the project (see LICENSE file if present, or consult repository owner).
