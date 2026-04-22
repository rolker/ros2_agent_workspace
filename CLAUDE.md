# Claude Code — Workspace Rules

@AGENTS.md

## Environment Setup

```bash
source .agent/scripts/setup.bash                    # ROS 2 + checkout guardrail
# Pass your actual model as the 3rd argument (from your system prompt — do NOT rely on the fallback).
source .agent/scripts/set_git_identity_env.sh "Claude Code Agent" "roland+claude-code@ccom.unh.edu" "<your model>"
```

Replace `<your model>` with your actual runtime model (e.g. `Claude Opus 4.7 (1M context)`,
`Claude Sonnet 4.6`). Verify with `echo "$AGENT_MODEL"` — it should echo exactly what you passed.
Do not edit `framework_config.sh` to match your model; the entries there are fallback-only.

## Claude-Specific Notes

- Makefile `.PHONY` targets (excluding `help`) are available as `/make_*` slash commands
  (e.g., `/make_build`, `/make_test`, `/make_dashboard`). After adding or removing eligible
  `.PHONY` targets, run `make generate-skills` to regenerate the slash commands.

## References

- [`AGENTS.md`](AGENTS.md) — Shared workspace rules (all agents)
- [`README.md` Vision section](README.md#vision) — Workspace purpose and goals
- [`ARCHITECTURE.md`](ARCHITECTURE.md) — System design and layering
- [`docs/decisions/`](docs/decisions/) — Architecture Decision Records
- [`.agent/WORKTREE_GUIDE.md`](.agent/WORKTREE_GUIDE.md) — Detailed worktree patterns
- [`.agent/AI_IDENTITY_STRATEGY.md`](.agent/AI_IDENTITY_STRATEGY.md) — Multi-framework identity
- [`.agent/WORKFORCE_PROTOCOL.md`](.agent/WORKFORCE_PROTOCOL.md) — Multi-agent coordination
- [`.agent/knowledge/`](.agent/knowledge/) — ROS 2 development patterns, CLI best practices, and skill workflows
- [`.agent/project_knowledge/`](.agent/project_knowledge/) — Symlink to manifest repo's `.agents/workspace-context/` (gitignored, created by `setup_layers.sh`; may not exist)
- [`.agent/templates/`](.agent/templates/) — Issue and test templates
