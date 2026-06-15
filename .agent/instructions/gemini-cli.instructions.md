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

**Commit pattern**: sourcing `set_git_identity_env.sh` exports the env
vars for this shell, but Gemini CLI typically runs each bash command
in a fresh subshell — the exports don't reach a separate `git commit`
invocation later. Use per-commit `-c` overrides instead:

```bash
git -c user.name="$AGENT_NAME" \
    -c user.email="$AGENT_EMAIL" \
    commit -m "…"
```

This is enforced on `feature/issue-<N>` / `feature/ISSUE-<N>-<desc>` /
`skill/*` branches by `.agent/hooks/check-commit-identity.py` (pre-commit)
and `.agent/hooks/check_pr_authors.py` (CI). See AGENTS.md "Agent
Commit Identity" for the full rationale.

## Gemini-Specific Notes

- **Google Cloud**: If Gemini CLI has access to Google Cloud services, check your environment for available integrations.
- If `gh` CLI is installed, use it for GitHub operations.

## Pre-Push Code Review

The shared rule (see [`AGENTS.md` Post-Task Verification](../../AGENTS.md#post-task-verification))
expects authors to run `/review-code` against their diff before opening
a PR. This applies to Gemini CLI sessions too. The skill body is plain
markdown at [`.claude/skills/review-code/SKILL.md`](../../.claude/skills/review-code/SKILL.md);
follow its steps as you would any other governance skill.

**Claude Adversarial Specialist is Claude-only** — its dispatch
relies on Claude Code's `Agent` tool to launch fresh subagents with
no shared context, and it is now the **default** adversarial read (one
pass at Light; two disjoint-lens passes at Standard + Deep). Gemini
doesn't expose an equivalent, so run the other specialists (Static
Analysis, Governance, Plan Drift) and note in the report header that
**both** Claude Adversarial passes were skipped because the runtime is
Gemini — the run has no in-house adversarial coverage. Copilot
Adversarial is **opt-in** via `--copilot` (off by default to conserve
the Premium quota); when opted in with the `copilot` CLI installed and
authenticated, it gives a cross-model read that partly compensates.

**Lifecycle handoff is Claude-specific** — the workflow skills' `### Next
step` blocks dispatch the next phase via
`.agent/scripts/dispatch_subagent.sh` / Claude Code's `Agent` tool. On a
Gemini runtime there's no auto-dispatch: read the handoff block and drive
the next skill yourself; the issue's `progress.md` entry is the cross-phase
handoff vessel.


## Workflow Skills

Reusable workflow procedures are documented (from the repo root) in `.claude/skills/*/SKILL.md`.
These are plain markdown — not Claude Code-specific. When asked to review an
issue, plan a task, review a PR, brainstorm, or run research, read the
corresponding SKILL.md and follow its steps.

Available workflow skills: `review-issue`, `plan-task`, `review-plan`,
`review-code`, `brainstorm`, `research`, `audit-workspace`, `audit-project`,
`gather-project-knowledge`, `onboard-project`, `brand-guidelines`,
`triage-reviews`, `address-findings`, `skill-importer`, `document-package`, `issue-triage`,
`test-engineering`, `inspiration-tracker`, `import-field-changes`,
`start-deployment`.

## References

- [`AGENTS.md`](../../AGENTS.md) — Shared workspace rules (all agents)
- [`../../ARCHITECTURE.md`](../../ARCHITECTURE.md) — System design and layering
- [`../WORKTREE_GUIDE.md`](../WORKTREE_GUIDE.md) — Detailed worktree patterns
- [`../AI_IDENTITY_STRATEGY.md`](../AI_IDENTITY_STRATEGY.md) — Multi-framework identity
- [`../WORKFORCE_PROTOCOL.md`](../WORKFORCE_PROTOCOL.md) — Multi-agent coordination
- [`../knowledge/`](../knowledge/) — ROS 2 development patterns and CLI best practices
- [`../project_knowledge/`](../project_knowledge/) — Symlink to manifest repo's `.agents/workspace-context/` (gitignored, created by `setup_layers.sh`; may not exist)
- [`../templates/`](../templates/) — Issue and test templates
