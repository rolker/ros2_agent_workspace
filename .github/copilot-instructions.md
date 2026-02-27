# GitHub Copilot — Workspace Rules

Read and follow all rules in [`AGENTS.md`](../AGENTS.md) at the repository root.
That file contains the shared workspace rules for all AI agents.

## Code Review Guidelines

This repository uses a plan-first workflow. The first commit on a feature branch
is often a work plan in `.agent/work-plans/PLAN_ISSUE-<N>.md`. When reviewing:

- If the PR contains only a plan file, review the plan for clarity, completeness,
  and alignment with the principles in `docs/PRINCIPLES.md`.
- If the PR contains implementation, review code against the plan and principles.
- Reference `docs/PRINCIPLES.md` for workspace guiding principles.
- Reference `docs/decisions/` for Architecture Decision Records.
- The principle "Radical simplicity" was renamed to "Only what's needed" — use the
  current name from `docs/PRINCIPLES.md`, not historical references.

## Environment Setup

```bash
source .agent/scripts/env.sh                    # ROS 2 + checkout guardrail
source .agent/scripts/set_git_identity_env.sh "Copilot CLI Agent" "roland+copilot-cli@ccom.unh.edu"
```

After sourcing, verify `$AGENT_MODEL` matches your actual model (from your system prompt).
If stale, update the default in `.agent/scripts/framework_config.sh` and commit the one-line fix.

## Copilot-Specific Notes

- **Native GitHub access**: Use `gh` CLI for fast PR/issue operations.
- **ROS 2**: Jazzy, Kilted, and Rolling (multi-distro support).
- **Build system**: colcon. **VCS tool**: vcstool. **Python**: 3.10+.


## Workflow Skills

Reusable workflow procedures are documented at the repo root in `.claude/skills/*/SKILL.md` (see [`.claude/skills/`](../.claude/skills/)).
These are plain markdown — not Claude Code-specific. When asked to review an
issue, plan a task, review a PR, brainstorm, or run research, read the
corresponding SKILL.md and follow its steps.

Available workflow skills: `review-issue`, `plan-task`, `review-pr`,
`brainstorm`, `research`, `audit-workspace`, `audit-project`,
`gather-project-knowledge`, `brand-guidelines`.

## References

- [`AGENTS.md`](../AGENTS.md) — Shared workspace rules (all agents)
- [`ARCHITECTURE.md`](../ARCHITECTURE.md) — System design and layering
- [`.agent/WORKTREE_GUIDE.md`](../.agent/WORKTREE_GUIDE.md) — Detailed worktree patterns
- [`.agent/AI_IDENTITY_STRATEGY.md`](../.agent/AI_IDENTITY_STRATEGY.md) — Multi-framework identity
- [`.agent/WORKFORCE_PROTOCOL.md`](../.agent/WORKFORCE_PROTOCOL.md) — Multi-agent coordination
- [`.agent/knowledge/`](../.agent/knowledge/) — ROS 2 development patterns and CLI best practices
- [`.agent/project_knowledge/`](../.agent/project_knowledge/) — Project-specific conventions (symlink, may not exist)
- [`.agent/templates/`](../.agent/templates/) — Issue and test templates
