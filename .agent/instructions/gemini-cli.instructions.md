# Gemini CLI — Workspace Rules

Read and follow all rules in [`AGENTS.md`](../../AGENTS.md) at the repository root.
That file contains the shared workspace rules for all AI agents.

## Environment Setup

```bash
source .agent/scripts/setup.bash                    # ROS 2 + checkout guardrail
# Pass your actual model as the 3rd argument (from your system prompt — do NOT rely on the fallback).
source .agent/scripts/set_git_identity_env.sh "Gemini CLI Agent" "roland+gemini-cli@ccom.unh.edu" "<your model>"
```

Replace `<your model>` with your actual runtime model (e.g. `Gemini 2.5 Pro`, `Gemini 3 Pro`).
Verify with `echo "$AGENT_MODEL"` — it should echo exactly what you passed. Do not edit
`framework_config.sh` to match your model; the entries there are fallback-only.

## Gemini-Specific Notes

- **Google Cloud**: If Gemini CLI has access to Google Cloud services, check your environment for available integrations.
- If `gh` CLI is installed, use it for GitHub operations.

## Pre-Push Code Review

The shared rule (see [`AGENTS.md` Post-Task Verification](../../AGENTS.md#post-task-verification))
expects authors to run `/review-code` against their diff before opening
a PR. This applies to Gemini CLI sessions too. The skill body is plain
markdown at [`.claude/skills/review-code/SKILL.md`](../../.claude/skills/review-code/SKILL.md);
follow its steps as you would any other governance skill.

**Adversarial Specialist is Claude-only** — its dispatch relies on
Claude Code's `Agent` tool to launch a fresh subagent with no shared
context. Gemini doesn't expose an equivalent. Run the other specialists
(Static Analysis, Governance, Plan Drift) and note in the report header
that Adversarial was skipped because the runtime is Gemini.


## Workflow Skills

Reusable workflow procedures are documented (from the repo root) in `.claude/skills/*/SKILL.md`.
These are plain markdown — not Claude Code-specific. When asked to review an
issue, plan a task, review a PR, brainstorm, or run research, read the
corresponding SKILL.md and follow its steps.

Available workflow skills: `review-issue`, `plan-task`, `review-plan`,
`review-code`, `brainstorm`, `research`, `audit-workspace`, `audit-project`,
`gather-project-knowledge`, `onboard-project`, `brand-guidelines`,
`triage-reviews`, `skill-importer`, `document-package`, `issue-triage`,
`test-engineering`, `inspiration-tracker`, `import-field-changes`.

## References

- [`AGENTS.md`](../../AGENTS.md) — Shared workspace rules (all agents)
- [`../../ARCHITECTURE.md`](../../ARCHITECTURE.md) — System design and layering
- [`../WORKTREE_GUIDE.md`](../WORKTREE_GUIDE.md) — Detailed worktree patterns
- [`../AI_IDENTITY_STRATEGY.md`](../AI_IDENTITY_STRATEGY.md) — Multi-framework identity
- [`../WORKFORCE_PROTOCOL.md`](../WORKFORCE_PROTOCOL.md) — Multi-agent coordination
- [`../knowledge/`](../knowledge/) — ROS 2 development patterns and CLI best practices
- [`../project_knowledge/`](../project_knowledge/) — Symlink to manifest repo's `.agents/workspace-context/` (gitignored, created by `setup_layers.sh`; may not exist)
- [`../templates/`](../templates/) — Issue and test templates
