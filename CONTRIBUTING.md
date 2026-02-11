# Contributing to the ROS 2 Agent Workspace

Thank you for your interest in contributing! This document provides guidelines for contributing to the workspace, developed by the Center for Coastal and Ocean Mapping / Joint Hydrographic Center (CCOM/JHC) at the University of New Hampshire (UNH).

## Code of Conduct

The Center is committed to providing a welcoming and inclusive environment. All contributors are expected to be respectful and professional in all interactions.

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
> Each AI agent framework has a self-contained instruction file with all rules inline.

### Framework Instruction Files
- **Claude Code**: [`CLAUDE.md`](CLAUDE.md) (auto-loaded every conversation)
- **GitHub Copilot**: [`.github/copilot-instructions.md`](.github/copilot-instructions.md)
- **Gemini CLI**: [`.agent/instructions/gemini-cli.instructions.md`](.agent/instructions/gemini-cli.instructions.md)
- **Other/Unknown**: [`.agent/AI_RULES.md`](.agent/AI_RULES.md) (universal fallback)

### When Updating Agent Documentation
- Update each framework's instruction file directly — rules are inlined, not shared
- Keep changes consistent across all framework files
- Include concrete code examples and build/test commands

## Development Workflow

### Planning Before Implementation

For features and non-trivial bug fixes, **always plan before implementing**:

1. **Create or use Feature Track issue** - Use the `.github/ISSUE_TEMPLATE/feature_track.md` template
2. **Fill out the Spec section**:
   - User story (As a..., I want..., so that...)
   - Acceptance criteria (what success looks like)
   - Out of scope (what you're NOT doing)
3. **Create a Plan** with phases and tasks
4. **Get user approval** before writing code
5. **Update plan as you work** - check off tasks, document decisions
6. **Verify at phase boundaries** - run tests, get user checkpoint

**Exception**: Trivial changes (typos, obvious one-liners) can skip formal planning.

See your framework's instruction file for complete details.

### Creating a Feature Branch

Follow the git hygiene rules:

- Features: `feature/TASK-<ID>-<description>` (e.g., `feature/TASK-001-multi-distro`)
- Fixes: `fix/<description>`
- Never commit directly to `main` or `jazzy`

```bash
git checkout -b feature/TASK-123-add-new-layer
```

### Making Changes

1. **Keep the workspace root clean**
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
├── .agent/              # Agent infrastructure
│   ├── scripts/        # Helper scripts for setup, build, test
│   ├── knowledge/      # ROS 2 patterns and CLI best practices
│   ├── templates/      # Issue and test templates
│   └── hooks/          # Git hooks (pre-commit)
├── configs/            # .repos files defining layers
└── layers/             # ROS 2 layers (gitignored, generated)
```

## Adding New Layers

To add a new workspace layer:

1. Create `configs/<layer-name>.repos` with repository definitions
2. Update `config/layers.txt` in the key repository if the layer should be auto-sourced
3. Document the layer in README.md
4. Test setup: `./.agent/scripts/setup.sh <layer-name>`

## Reporting Infrastructure Friction

**For AI Agents**: Help improve the workspace by reporting friction points you encounter!

If you experience issues with agent infrastructure (documentation, workflows, scripts), use the **Continuous Improvement workflow**:

```bash
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

## Feature Reversal

If a feature needs to be completely undone (not fixed, but reverted):

```bash
# Preview what would be reverted
.agent/scripts/revert_feature.sh --issue 137 --dry-run

# Actually revert all commits for the issue
.agent/scripts/revert_feature.sh --issue 137

# Or via Makefile
make revert-feature ISSUE=137
```

The script finds all commits referencing the issue number and creates revert commits in the correct order. Use this when a feature direction was wrong, not for fixing bugs in a feature.

Use the improvement issue template at `.agent/templates/improvement_issue.md` to structure your report.

## Questions or Issues?

- Check existing issues on GitHub
- Review the README.md and documentation
- Ask questions in pull request discussions
- For security issues, see SECURITY.md

## License

By contributing, you agree that your contributions will be licensed under the same license as the project (see LICENSE file if present, or consult repository owner).
