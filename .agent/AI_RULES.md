# Universal AI Agent Rules

Fallback rules for agents without a framework-specific instruction file.
If your framework has a dedicated file, use that instead — it contains these
rules plus framework-specific guidance.

## Framework Entry Points

| Framework | Instruction File |
|-----------|-----------------|
| Claude Code | [`CLAUDE.md`](../CLAUDE.md) (auto-loaded) |
| GitHub Copilot | [`.github/copilot-instructions.md`](../.github/copilot-instructions.md) |
| Gemini CLI | [`.agent/instructions/gemini-cli.instructions.md`](instructions/gemini-cli.instructions.md) |
| Other / Unknown | This file |

## Rules

1. **Source environment**: `source .agent/scripts/env.sh`
2. **Configure identity**: `source .agent/scripts/set_git_identity_env.sh "<Name>" "<email>"` (host agents) or `.agent/scripts/configure_git_identity.sh` (container agents). See [`AI_IDENTITY_STRATEGY.md`](AI_IDENTITY_STRATEGY.md).
3. **Never commit to `main`** — branch is protected. Use feature branches via worktrees.
4. **Never `git checkout <branch>`** — `env.sh` blocks it. Use worktrees: `.agent/scripts/worktree_create.sh --issue <N> --type workspace [--plan-file <path>]`
5. **GIT_EDITOR=true** for rebase/amend/merge to avoid hanging on interactive editors.
6. **Issue-first** — no code without a GitHub issue. Reference with `Closes #<N>` in PRs.
   Before your first commit, verify the issue matches your task:
   `gh issue view $WORKTREE_ISSUE --json title --jq '.title'`
   If the title doesn't match, stop — you may have the wrong issue number.
7. **AI signature required** on all GitHub Issues/PRs/Comments:
   ```
   ---
   **Authored-By**: `<Your Agent Name>`
   **Model**: `<Your Actual Model Name>`
   ```
   Use `$AGENT_NAME` / `$AGENT_MODEL` env vars — never copy example names.
8. **Use `--body-file`** for `gh` CLI, not inline `--body` (newlines break).
9. **Build in layer directories only** — never `colcon build` from workspace root.
10. **Never document from assumptions** — verify every parameter, topic, service, and message type by reading source code. See [`knowledge/documentation_verification.md`](knowledge/documentation_verification.md).
11. **Link GitHub references on every mention** — in summaries/reports, include a clickable URL for each issue, PR, commit, or repository. Use markdown links where supported; otherwise include the full URL inline or on the next line. This is a link rule only, not a required summary template.

Workspace repo: `rolker/ros2_agent_workspace`. For project repos, derive
the slug from the git remote (`git remote get-url origin`).

Examples:
- `[Issue #129: Clickable GitHub references](https://github.com/rolker/ros2_agent_workspace/issues/129)`
- `PR #68: https://github.com/rolker/unh_marine_autonomy/pull/68`
- `[e8c32bc](https://github.com/rolker/unh_marine_autonomy/commit/<full-sha>)`

## Key Scripts

| Script | Purpose |
|--------|---------|
| `.agent/scripts/env.sh` | ROS 2 env + checkout guardrail |
| `.agent/scripts/set_git_identity_env.sh` | Ephemeral git identity |
| `.agent/scripts/worktree_create.sh` | Create isolated worktree (`--plan-file` to create draft PR with plan) |
| `.agent/scripts/worktree_enter.sh` | Enter worktree (must be sourced) |
| `.agent/scripts/worktree_remove.sh` | Remove worktree |
| `.agent/scripts/status_report.sh` | Workspace status (supports `--quick`, `--pr-triage`) |
| `.agent/scripts/build.sh` | Build all layers |

## References

- [`ARCHITECTURE.md`](../ARCHITECTURE.md) — System design
- [`AI_IDENTITY_STRATEGY.md`](AI_IDENTITY_STRATEGY.md) — Identity decision tree
- [`WORKFORCE_PROTOCOL.md`](WORKFORCE_PROTOCOL.md) — Multi-agent coordination
- [`WORKTREE_GUIDE.md`](WORKTREE_GUIDE.md) — Detailed worktree patterns
- [`.agent/knowledge/`](knowledge/) — ROS 2 patterns and CLI best practices
- Project repo `.agents/README.md` — Per-repo agent guide: package inventory, code layout, pitfalls (if present)
- [`.agent/project_knowledge/`](project_knowledge/) — Project-specific conventions and architecture (symlink, may not exist)
