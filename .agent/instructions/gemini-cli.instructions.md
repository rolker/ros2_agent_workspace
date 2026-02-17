# Gemini CLI — Operational Rules

This file is the single source of truth for Gemini CLI agents in this workspace.
All rules are inline — no need to read other docs before starting work.

## Environment Setup

```bash
source .agent/scripts/env.sh                    # ROS 2 + checkout guardrail
source .agent/scripts/set_git_identity_env.sh "Gemini CLI Agent" "roland+gemini-cli@ccom.unh.edu"
```

After sourcing, verify `$AGENT_MODEL` matches your actual model (from your system prompt).
If stale, update the default in `.agent/scripts/framework_config.sh` and commit the one-line fix.

## Git Rules

- **Never commit to `main`** — branch is protected; direct pushes are rejected.
- **Never `git checkout <branch>`** — `env.sh` blocks it. Use worktrees instead.
- **GIT_EDITOR=true** for rebase/amend/merge to avoid hanging on interactive editors.
- **Branch naming**: `feature/issue-<N>` or `feature/ISSUE-<N>-<description>`.
- **Atomic commits**: one logical change per commit. Don't bundle unrelated fixes.
- **All changes via Pull Requests**.

## Worktree Workflow (Required)

Every task must use an isolated worktree — never work in the main tree.

```bash
# Create + enter (add --draft-pr to create a draft PR immediately)
.agent/scripts/worktree_create.sh --issue <N> --type workspace [--draft-pr]
source .agent/scripts/worktree_enter.sh --issue <N>

# For ROS package work, use layer worktrees
.agent/scripts/worktree_create.sh --issue <N> --type layer --layer core [--draft-pr]

# List / remove
.agent/scripts/worktree_list.sh
.agent/scripts/worktree_remove.sh --issue <N>
```

## Issue-First Policy

No code without a ticket. Check for an existing GitHub issue first; if none exists,
ask the user: "Should I open an issue to track this?" Use the issue number in branches
and reference it in PRs with `Closes #<N>`.

**Exception**: trivial typo/doc fixes.

**Verify before committing**: Before your first commit, confirm the issue matches
your task: `gh issue view $WORKTREE_ISSUE --json title --jq '.title'`
If the title does not match, stop — you may be in the wrong worktree or have the
wrong issue number copied from a plan or status report.

## AI Signature (Required on all GitHub Issues/PRs/Comments)

```markdown
---
**Authored-By**: `Gemini CLI Agent`
**Model**: `<your actual model name>`
```

Use your actual runtime identity — never copy example model names from docs.
Check `$AGENT_NAME` / `$AGENT_MODEL` environment variables if unsure.

## GitHub CLI: Use `--body-file`, Not `--body`

Multiline `--body` strings break newlines. Always write to a temp file first:

```bash
BODY_FILE=$(mktemp /tmp/gh_body.XXXXXX.md)
cat << 'EOF' > "$BODY_FILE"
Your markdown content here.
EOF
gh pr create --title "Title" --body-file "$BODY_FILE"
rm "$BODY_FILE"
```

## Build & Test

```bash
make build                                       # Build all layers
make test                                        # Run all tests
make validate                                    # Validate workspace

# Single package
cd layers/main/core_ws && colcon build --packages-select <package>
colcon test --packages-select <package> && colcon test-result --verbose

make lint                                        # Lint + hooks (uses venv pre-commit)
```

**Build in layer directories only** — never `colcon build` from the workspace root.

## Documentation Accuracy

- **Never document from assumptions** — every claim about parameters, topics, services,
  message types, or API signatures must be verified by reading the actual source code.
- Before writing or updating package documentation, read `package.xml`, all source files
  that declare parameters/publishers/subscribers, and any `.msg`/`.srv`/`.action` files.
- Use the verification workflow in [`../knowledge/documentation_verification.md`](../knowledge/documentation_verification.md).
- Use the documentation template in [`../templates/package_documentation.md`](../templates/package_documentation.md).

## Workspace Cleanliness

- Keep repo root and `layers/*/src/` clean — no temp files, build artifacts, or logs.
- Use `.agent/scratchpad/` for persistent temp files (unique names via `mktemp`).
- Use `/tmp` for ephemeral files cleaned up in the same command.

## Gemini-Specific Notes

- **Google Cloud**: If Gemini CLI has access to Google Cloud services, check your environment for available integrations.
- If `gh` CLI is installed, use it for GitHub operations. Otherwise the workspace falls back to web API calls.

## Script Reference

| Script | Purpose |
|--------|---------|
| `.agent/scripts/env.sh` | Source ROS 2 env + checkout guardrail |
| `.agent/scripts/set_git_identity_env.sh` | Ephemeral git identity (session-only) |
| `.agent/scripts/worktree_create.sh` | Create isolated worktree (`--draft-pr` for immediate visibility) |
| `.agent/scripts/worktree_enter.sh` | Enter worktree (must be sourced) |
| `.agent/scripts/worktree_remove.sh` | Remove worktree |
| `.agent/scripts/worktree_list.sh` | List active worktrees |
| `.agent/scripts/agent start-task <N>` | High-level wrapper: create + enter worktree |
| `.agent/scripts/status_report.sh` | Workspace status (supports `--quick`, `--pr-triage`) |
| `.agent/scripts/build.sh` | Build all layers in order |
| `.agent/scripts/check_branch_updates.sh` | Check if branch is behind default |
| `.agent/scripts/gh_create_issue.sh` | Create issue with label validation |
| `.agent/scripts/revert_feature.sh` | Revert all commits for an issue |
| `.agent/scripts/sync_repos.py` | Sync all workspace repositories |
| `.agent/scripts/validate_workspace.py` | Validate repos match .repos config |

## Layered Architecture

```
layers/main/
├── underlay_ws/    # Additional dependencies
├── core_ws/        # UNH Marine Autonomy Framework
├── platforms_ws/   # Platform-specific code
├── sensors_ws/     # Sensor drivers
├── simulation_ws/  # Simulation tools
└── ui_ws/          # Visualization
```

`layers/` is gitignored — use `ls` to inspect it, not grep/glob.

## References (Read When Needed, Not Upfront)

- [`../../ARCHITECTURE.md`](../../ARCHITECTURE.md) — System design and layering
- [`../WORKTREE_GUIDE.md`](../WORKTREE_GUIDE.md) — Detailed worktree patterns
- [`../AI_IDENTITY_STRATEGY.md`](../AI_IDENTITY_STRATEGY.md) — Multi-framework identity
- [`../WORKFORCE_PROTOCOL.md`](../WORKFORCE_PROTOCOL.md) — Multi-agent coordination
- [`../knowledge/`](../knowledge/) — ROS 2 development patterns and CLI best practices
- [`../project_knowledge/`](../project_knowledge/) — Project-specific conventions and architecture (symlink, may not exist)
- [`../templates/`](../templates/) — Issue and test templates
