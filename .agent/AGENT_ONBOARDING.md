# Agent Onboarding (Containerized / Unknown Agents)

For agents without a framework-specific instruction file.

**If your framework has a dedicated file, use that instead:**
- Claude Code: [`CLAUDE.md`](../CLAUDE.md) (auto-loaded)
- GitHub Copilot: [`.github/copilot-instructions.md`](../.github/copilot-instructions.md)
- Gemini CLI: [`instructions/gemini-cli.instructions.md`](instructions/gemini-cli.instructions.md)

## Setup

```bash
# 1. Source ROS 2 environment
source .agent/scripts/env.sh

# 2. Configure git identity
# Host-based (shared workspace):
source .agent/scripts/set_git_identity_env.sh "<Agent Name>" "<email>"
# Container/isolated:
.agent/scripts/configure_git_identity.sh "<Agent Name>" "<email>"

# 3. Check workspace status
.agent/scripts/status_report.sh
```

See [`AI_IDENTITY_STRATEGY.md`](AI_IDENTITY_STRATEGY.md) for the full identity decision tree.

## Core Rules

See [`AGENTS.md`](../AGENTS.md) for the shared workspace rules all agents must follow.

Key points:
- Never commit to `main` — use worktrees for isolation
- Never `git checkout <branch>` — `env.sh` blocks it
- AI signature required on all GitHub Issues/PRs/Comments
- Use `--body-file` for `gh` CLI, not inline `--body`
- Build in layer directories only

## Starting Work

```bash
# Create isolated worktree for your issue
.agent/scripts/worktree_create.sh --issue <N> --type workspace
source .agent/scripts/worktree_enter.sh --issue <N>
```

## Workflow Skills

Reusable workflow procedures are documented in `.claude/skills/*/SKILL.md`.
These are plain markdown — not Claude Code-specific. When asked to review an
issue, plan a task, review a PR, brainstorm, or run research, read the
corresponding SKILL.md and follow its steps.

Available workflow skills: `review-issue`, `plan-task`, `review-pr`,
`brainstorm`, `research`, `audit-workspace`, `audit-project`,
`gather-project-knowledge`, `brand-guidelines`.

## References

- [`AGENTS.md`](../AGENTS.md) — Shared workspace rules (all agents)
- [`AI_IDENTITY_STRATEGY.md`](AI_IDENTITY_STRATEGY.md) — Identity configuration
- [`WORKFORCE_PROTOCOL.md`](WORKFORCE_PROTOCOL.md) — Multi-agent coordination
- [`WORKTREE_GUIDE.md`](WORKTREE_GUIDE.md) — Worktree patterns
- [`../ARCHITECTURE.md`](../ARCHITECTURE.md) — System design
- Project repo `.agents/README.md` — Per-repo agent guide (if present)
