# Agent Onboarding (Containerized / Unknown Agents)

For agents without a framework-specific instruction file.

**If your framework has a dedicated file, use that instead:**
- Claude Code: [`CLAUDE.md`](../CLAUDE.md) (auto-loaded)
- GitHub Copilot: [`.github/copilot-instructions.md`](../.github/copilot-instructions.md)
- Gemini CLI: [`instructions/gemini-cli.instructions.md`](instructions/gemini-cli.instructions.md)

## Setup

```bash
# 1. Source ROS 2 environment
source .agent/scripts/setup.bash

# 2. Configure git identity
# Host-based (shared workspace): pass your actual model as the 3rd argument
# (from your system prompt — do NOT rely on framework_config.sh defaults).
source .agent/scripts/set_git_identity_env.sh "<Agent Name>" "<email>" "<your model>"
# Container/isolated:
.agent/scripts/configure_git_identity.sh "<Agent Name>" "<email>"

# 3. Check workspace status
.agent/scripts/dashboard.sh --quick
```

See [`AI_IDENTITY_STRATEGY.md`](AI_IDENTITY_STRATEGY.md) for the full identity decision tree.

**Commit pattern** (host-based agents): the env vars exported by
`set_git_identity_env.sh` may not survive across separate bash
invocations (most agent runtimes spawn fresh subshells per call). Use
per-commit `-c` overrides as the canonical pattern:

```bash
git -c user.name="$AGENT_NAME" \
    -c user.email="$AGENT_EMAIL" \
    commit -m "…"
```

The `-c` flags propagate to git's subprocesses (including the
pre-commit hook) via `GIT_CONFIG_PARAMETERS`, so the identity is
robust regardless of subshell semantics. Enforced on agent-convention
branches by `.agent/hooks/check-commit-identity.py` and the CI check.
See AGENTS.md "Agent Commit Identity" for full rationale.

## Core Rules

See [`AGENTS.md`](../AGENTS.md) for the shared workspace rules all agents must follow.

Key points:
- Never commit directly to the default branch (e.g. `main`, `jazzy`) on a
  GitHub-origin repo — use worktrees for isolation. Repos with non-GitHub
  origin (field mode) have their own workflow; see
  [`AGENTS.md` Field Mode](../AGENTS.md#field-mode-origin-not-on-a-github-host).
- Never `git checkout <branch>` — `setup.bash` blocks it
- AI signature required on all GitHub Issues/PRs/Comments
- Use `--body-file` for `gh` CLI, not inline `--body`
- Build in layer directories only

## Starting Work

```bash
# Create isolated worktree for your issue
.agent/scripts/worktree_create.sh --issue <N> --type workspace
source .agent/scripts/worktree_enter.sh --issue <N>
```

## Pre-Push Code Review

The shared rule (see [`AGENTS.md` Post-Task Verification](../AGENTS.md#post-task-verification))
expects authors to run `/review-code` against their diff before opening
a PR. The skill body is plain markdown at
[`.claude/skills/review-code/SKILL.md`](../.claude/skills/review-code/SKILL.md);
follow its steps regardless of framework.

**Claude Adversarial Specialist is Claude-only** — its dispatch
relies on Claude Code's `Agent` tool, and it is now the **default**
adversarial read (one pass at Light; two disjoint-lens passes at
Standard + Deep). Other frameworks can't dispatch it, so they should
run the remaining specialists (Static Analysis, Governance, Plan
Drift) and note that **both** Claude Adversarial passes were skipped
due to the runtime — i.e. the run has no in-house adversarial coverage.
Copilot Adversarial is **opt-in** via `--copilot` (off by default to
conserve the Premium quota); when opted in and the `copilot` CLI is
installed and authenticated, it provides a cross-model read that
non-Claude runtimes can use to partly compensate.

**Lifecycle handoff is Claude-specific** — the workflow skills' `### Next
step` blocks dispatch the next phase via
`.agent/scripts/dispatch_subagent.sh` / Claude Code's `Agent` tool. On a
non-Claude runtime there's no auto-dispatch: read the handoff block and
drive the next skill yourself; the issue's `progress.md` entry is the
cross-phase handoff vessel.

## Workflow Skills

Reusable workflow procedures are documented at the repo root in `.claude/skills/*/SKILL.md`.
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

- [`AGENTS.md`](../AGENTS.md) — Shared workspace rules (all agents)
- [`AI_IDENTITY_STRATEGY.md`](AI_IDENTITY_STRATEGY.md) — Identity configuration
- [`WORKFORCE_PROTOCOL.md`](WORKFORCE_PROTOCOL.md) — Multi-agent coordination
- [`WORKTREE_GUIDE.md`](WORKTREE_GUIDE.md) — Worktree patterns
- [`../ARCHITECTURE.md`](../ARCHITECTURE.md) — System design
- Project repo `.agents/README.md` — Per-repo agent guide (if present)
