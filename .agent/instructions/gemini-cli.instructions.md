# Gemini CLI — Workspace Rules

Read and follow all rules in [`AGENTS.md`](../../AGENTS.md) at the repository root.
That file contains the shared workspace rules for all AI agents.

## Environment Setup

```bash
source .agent/scripts/env.sh                    # ROS 2 + checkout guardrail
source .agent/scripts/set_git_identity_env.sh "Gemini CLI Agent" "roland+gemini-cli@ccom.unh.edu"
```

After sourcing, verify `$AGENT_MODEL` matches your actual model (from your system prompt).
If stale, update the default in `.agent/scripts/framework_config.sh` and commit the one-line fix.

## Gemini-Specific Notes

- **Google Cloud**: If Gemini CLI has access to Google Cloud services, check your environment for available integrations.
- If `gh` CLI is installed, use it for GitHub operations.


## Workflow Skills

Reusable workflow procedures are documented in `.claude/skills/*/SKILL.md`.
These are plain markdown — not Claude Code-specific. When asked to review an
issue, plan a task, review a PR, brainstorm, or run research, read the
corresponding SKILL.md and follow its steps.

Available workflow skills: `review-issue`, `plan-task`, `review-pr`,
`brainstorm`, `research`, `audit-workspace`, `audit-project`,
`gather-project-knowledge`, `brand-guidelines`.

## References

- [`AGENTS.md`](../../AGENTS.md) — Shared workspace rules (all agents)
- [`../../ARCHITECTURE.md`](../../ARCHITECTURE.md) — System design and layering
- [`../WORKTREE_GUIDE.md`](../WORKTREE_GUIDE.md) — Detailed worktree patterns
- [`../AI_IDENTITY_STRATEGY.md`](../AI_IDENTITY_STRATEGY.md) — Multi-framework identity
- [`../WORKFORCE_PROTOCOL.md`](../WORKFORCE_PROTOCOL.md) — Multi-agent coordination
- [`../knowledge/`](../knowledge/) — ROS 2 development patterns and CLI best practices
- [`../project_knowledge/`](../project_knowledge/) — Project-specific conventions (symlink, may not exist)
- [`../templates/`](../templates/) — Issue and test templates
