# Claude Code — Workspace Rules

@AGENTS.md

## Environment Setup

```bash
source .agent/scripts/env.sh                    # ROS 2 + checkout guardrail
source .agent/scripts/set_git_identity_env.sh "Claude Code Agent" "roland+claude-code@ccom.unh.edu"
```

After sourcing, verify `$AGENT_MODEL` matches your actual model (from your system prompt).
If stale, update the default in `.agent/scripts/framework_config.sh` and commit the one-line fix.

## Claude-Specific Notes

- **Branch naming**: `feature/issue-<N>` or `feature/ISSUE-<N>-<description>`.
- Makefile `.PHONY` targets (excluding `help`) are available as `/make_*` slash commands
  (e.g., `/make_build`, `/make_test`, `/make_status`). After adding or removing eligible
  `.PHONY` targets, run `make generate-skills` to regenerate the slash commands.

## GitHub Reference Links

- Workspace repo: `rolker/ros2_agent_workspace`. For project repos, derive
  the slug from the git remote (`git remote get-url origin`).

Examples:
- `[Issue #129: Clickable GitHub references](https://github.com/rolker/ros2_agent_workspace/issues/129)`
- `[PR #68: Example pull request](https://github.com/rolker/unh_marine_autonomy/pull/68)`
- `[e8c32bc](https://github.com/rolker/unh_marine_autonomy/commit/<full-sha>)`

## References

- [`AGENTS.md`](AGENTS.md) — Shared workspace rules (all agents)
- [`README.md` Vision section](README.md#vision) — Workspace purpose and goals
- [`ARCHITECTURE.md`](ARCHITECTURE.md) — System design and layering
- [`docs/decisions/`](docs/decisions/) — Architecture Decision Records
- [`.agent/WORKTREE_GUIDE.md`](.agent/WORKTREE_GUIDE.md) — Detailed worktree patterns
- [`.agent/AI_IDENTITY_STRATEGY.md`](.agent/AI_IDENTITY_STRATEGY.md) — Multi-framework identity
- [`.agent/WORKFORCE_PROTOCOL.md`](.agent/WORKFORCE_PROTOCOL.md) — Multi-agent coordination
- [`.agent/knowledge/`](.agent/knowledge/) — ROS 2 development patterns and CLI best practices
- [`.agent/project_knowledge/`](.agent/project_knowledge/) — Project-specific conventions (symlink, may not exist)
- [`.agent/templates/`](.agent/templates/) — Issue and test templates
