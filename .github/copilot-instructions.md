# GitHub Copilot — Workspace Rules

Read and follow all rules in [`AGENTS.md`](../AGENTS.md) at the repository root.
That file contains the shared workspace rules for all AI agents.

## Code Review Guidelines

This repository uses a plan-first workflow. The first commit on a feature branch
is often a work plan at `.agent/work-plans/issue-<N>/plan.md` (or, for legacy
plans authored before the directory convention, `PLAN_ISSUE-<N>.md`). Project
repos follow the same convention — plans live in whichever repo owns the issue,
not only in the workspace repo. When reviewing:

- If the PR contains only a plan file, review the plan for clarity, completeness,
  and alignment with the principles in `docs/PRINCIPLES.md`.
- If the PR contains implementation, review code against the plan and principles.
- Reference `docs/PRINCIPLES.md` for workspace guiding principles.
- Reference `docs/decisions/` for Architecture Decision Records.
- The principle "Radical simplicity" was renamed to "Only what's needed" — use the
  current name from `docs/PRINCIPLES.md`, not historical references.

## Pre-Push Code Review

The shared rule (see [`AGENTS.md` Post-Task Verification](../AGENTS.md#post-task-verification))
expects authors to run `/review-code` against their diff before opening a PR.
This applies to Copilot too when used in **agent / coding-assistant mode** —
prefer a pre-push pass to catch findings locally rather than in PR
review rounds. The specialists that actually run depend on the
auto-classified tier: Light tier dispatches Static Analysis + one
Claude Adversarial pass, while Standard and Deep also dispatch
Governance, Plan Drift, and a second disjoint-lens Claude Adversarial
pass. The Copilot Adversarial Specialist is **opt-in** at every tier
via `--copilot` (off by default to conserve the Premium quota — see
[#467](https://github.com/rolker/ros2_agent_workspace/issues/467)). (The
Claude Adversarial Specialist is Claude-only — see caveat below — so a
Copilot-only pre-push pass cannot catch Claude-side adversarial
findings; the Copilot Adversarial Specialist runs natively when opted
in.) See
[`.claude/skills/review-code/SKILL.md`](../.claude/skills/review-code/SKILL.md).

**Limitation**: when Copilot runs as a **PR reviewer** (the GitHub
"review by Copilot" surface), it can't invoke local skills, so the
pre-push check is the human-or-other-agent author's responsibility.
The Copilot review surface remains complementary, not a substitute.

**Claude Adversarial Specialist is Claude-only** — it dispatches a
fresh subagent via Claude Code's `Agent` tool, which Copilot doesn't
expose. The other specialists (Static Analysis, Governance, Plan
Drift, **Copilot Adversarial**) are framework-agnostic and run
regardless of host runtime, provided the relevant CLI tools are
installed and authenticated where applicable (Copilot Adversarial is
opt-in via `--copilot` and additionally requires `copilot` CLI
authentication — opted-in but unauthenticated hosts route to the
skipped-with-notice path automatically).

**Lifecycle handoff is Claude-specific** — the workflow skills' `### Next
step` blocks dispatch the next phase via
`.agent/scripts/dispatch_subagent.sh` / Claude Code's `Agent` tool. On a
Copilot runtime there's no auto-dispatch: read the handoff block and drive
the next skill yourself; the issue's `progress.md` entry is the cross-phase
handoff vessel.

## Environment Setup

```bash
source .agent/scripts/setup.bash                    # ROS 2 + checkout guardrail
# Pass your actual model as the 3rd argument (from your system prompt — do NOT rely on the fallback).
source .agent/scripts/set_git_identity_env.sh "Copilot CLI Agent" "roland+copilot-cli@ccom.unh.edu" "<your model>"
```

Replace `<your model>` with your actual runtime model (e.g. `Claude Sonnet 4.5`, `GPT-5`).
Verify with `echo "$AGENT_MODEL"` — it should echo exactly what you passed. Do not edit
`framework_config.sh` to match your model; the entries there are fallback-only.

**Commit pattern**: sourcing `set_git_identity_env.sh` exports the env
vars for this shell, but Copilot CLI typically runs each bash command
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

## Copilot-Specific Notes

- **Native GitHub access**: Use `gh` CLI for fast PR/issue operations.
- **ROS 2**: Jazzy, Kilted, and Rolling (multi-distro support).
- **Build system**: colcon. **VCS tool**: vcstool. **Python**: 3.10+.


## Workflow Skills

Reusable workflow procedures are documented at the repo root in `.claude/skills/*/SKILL.md` (see [`.claude/skills/`](../.claude/skills/)).
These are plain markdown — not Claude Code-specific. When asked to review an
issue, plan a task, review a PR, brainstorm, or run research, read the
corresponding SKILL.md and follow its steps.

Available workflow skills: `review-issue`, `plan-task`, `review-plan`,
`review-code`, `brainstorm`, `research`, `audit-workspace`, `audit-project`,
`gather-project-knowledge`, `onboard-project`, `brand-guidelines`,
`triage-reviews`, `skill-importer`, `document-package`, `issue-triage`,
`test-engineering`, `inspiration-tracker`, `import-field-changes`,
`start-deployment`.

## References

- [`AGENTS.md`](../AGENTS.md) — Shared workspace rules (all agents)
- [`ARCHITECTURE.md`](../ARCHITECTURE.md) — System design and layering
- [`.agent/WORKTREE_GUIDE.md`](../.agent/WORKTREE_GUIDE.md) — Detailed worktree patterns
- [`.agent/AI_IDENTITY_STRATEGY.md`](../.agent/AI_IDENTITY_STRATEGY.md) — Multi-framework identity
- [`.agent/WORKFORCE_PROTOCOL.md`](../.agent/WORKFORCE_PROTOCOL.md) — Multi-agent coordination
- [`.agent/knowledge/`](../.agent/knowledge/) — ROS 2 development patterns and CLI best practices
- [`.agent/project_knowledge/`](../.agent/project_knowledge/) — Symlink to manifest repo's `.agents/workspace-context/` (gitignored, created by `setup_layers.sh`; may not exist)
- [`.agent/templates/`](../.agent/templates/) — Issue and test templates
