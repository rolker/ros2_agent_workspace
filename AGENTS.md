# Workspace Rules for AI Agents

Shared rules for all AI agents working in this workspace. For framework-specific
setup (environment, identity, features), see your framework's adapter file:

| Framework | Adapter File |
|-----------|-------------|
| Claude Code | [`CLAUDE.md`](CLAUDE.md) |
| GitHub Copilot | [`.github/copilot-instructions.md`](.github/copilot-instructions.md) |
| Gemini CLI | [`.agent/instructions/gemini-cli.instructions.md`](.agent/instructions/gemini-cli.instructions.md) |
| Other | [`.agent/AGENT_ONBOARDING.md`](.agent/AGENT_ONBOARDING.md) |

## Boundaries

### Always (proceed autonomously)

- Use worktrees for all feature work — never edit files in the main tree
- Run pre-commit hooks before committing
- Include AI signature on all GitHub Issues/PRs/Comments (`$AGENT_NAME` / `$AGENT_MODEL`)
- Reference issue numbers in branches and PRs (`Closes #<N>`)
- Set `GIT_EDITOR=true` for rebase/amend/merge
- Use `--body-file` for multiline `gh` CLI content (not `--body`)
- Include clickable GitHub links in summaries (use `gh` to look up URLs — never guess)
- Read `.agents/README.md` before modifying any project repo
- Verify issue matches task before first commit
- Build in layer directories only — never `colcon build` from workspace root
- Verify documentation claims against source code
- Atomic commits: one logical change per commit

### Ask First (get human approval)

- Modifying instruction files (`AGENTS.md`, `CLAUDE.md`, etc.)
- Adding or removing workspace layers
- Changing CI or branch protection configuration

### Never (hard stops)

- Commit to `main` — branch is protected; direct pushes are rejected
- `git checkout <branch>` — `env.sh` blocks it; use worktrees
- Skip hooks with `--no-verify`
- Commit secrets or credentials
- Document from assumptions — verify against source code
- Construct GitHub URLs from directory names — use `gh` CLI to look them up

## Worktree Workflow

Every task must use an isolated worktree.

**Project repos**: Directories under `layers/main/*_ws/src/` are typically independent
git repos, each containing one or more ROS 2 packages. Layer worktrees create git
worktrees *inside* these project repos — you commit and push to the project repo, not
the workspace repo. Non-git directories are symlinked instead. The `--packages` flag
takes project-repo directory names (not individual ROS package names).

```bash
# Workspace infrastructure work (docs, .agent/, configs)
.agent/scripts/worktree_create.sh --issue <N> --type workspace [--plan-file <path>]
source .agent/scripts/worktree_enter.sh --issue <N>

# ROS package work (requires --layer and --packages)
.agent/scripts/worktree_create.sh --issue <N> --type layer --layer <layer> --packages <project_repo> [--plan-file <path>]
source .agent/scripts/worktree_enter.sh --issue <N>
cd <layer>_ws/src/<project_repo>   # work here, commit/push here

# Multiple project repos in one worktree
.agent/scripts/worktree_create.sh --issue <N> --type layer --layer <layer> --packages <repo1>,<repo2>

# List / remove
.agent/scripts/worktree_list.sh
.agent/scripts/worktree_remove.sh --issue <N>
```

See [`.agent/WORKTREE_GUIDE.md`](.agent/WORKTREE_GUIDE.md) for hybrid structure details,
`--repo-slug` disambiguation, and troubleshooting.

## Issue-First Policy

No code without a ticket. Check for an existing GitHub issue first; if none exists,
ask the user: "Should I open an issue to track this?" Use the issue number in branches
and reference it in PRs with `Closes #<N>`.

Issues and PRs live in whichever repo owns the code being changed — check the project
repo, not just the workspace repo.

**Trivial fixes** (typos, minor doc corrections) don't need a dedicated issue — use the
current task's worktree or create a quick issue for a new one.

**Verify before committing**: Before your first commit, confirm the issue matches
your task: `gh issue view $WORKTREE_ISSUE --json title --jq '.title'`
If the title does not match, stop — you may be in the wrong worktree or have the
wrong issue number.

## AI Signature (Required on all GitHub Issues/PRs/Comments)

```markdown
---
**Authored-By**: `$AGENT_NAME`
**Model**: `$AGENT_MODEL`
```

Use your actual runtime identity — never copy example model names from docs.
Check `$AGENT_NAME` / `$AGENT_MODEL` environment variables.

## GitHub CLI Patterns

### Use `--body-file`, Not `--body`

Multiline `--body` strings break newlines. Always write to a temp file first:

```bash
BODY_FILE=$(mktemp /tmp/gh_body.XXXXXX.md)
cat << 'EOF' > "$BODY_FILE"
Your markdown content here.
EOF
gh pr create --title "Title" --body-file "$BODY_FILE"
rm "$BODY_FILE"
```

### Never Guess GitHub URLs

The local directory name may not match the GitHub repo name. Always use `gh` to
retrieve URLs:

```bash
gh issue view <N> --json url --jq '.url'
gh pr view <N> --json url --jq '.url'
gh repo view --json url --jq '.url'
```

### GitHub Reference Links in Summaries

When referencing any GitHub issue, PR, commit, or repository in summaries or reports,
include a clickable markdown link on every mention.

## Build & Test

```bash
make build                                       # Build all layers
make test                                        # Run all tests
make validate                                    # Validate workspace

# Single package
cd layers/main/<layer>_ws && colcon build --packages-select <package>
colcon test --packages-select <package> && colcon test-result --verbose

make lint                                        # Lint + hooks (uses venv pre-commit)
```

**Build in layer directories only** — never `colcon build` from the workspace root.

## Documentation Accuracy

- **Never document from assumptions** — every claim about parameters, topics, services,
  message types, or API signatures must be verified by reading the actual source code.
- Before writing or updating package documentation, read `package.xml`, all source files
  that declare parameters/publishers/subscribers, and any `.msg`/`.srv`/`.action` files.
- Use the verification workflow in [`.agent/knowledge/documentation_verification.md`](.agent/knowledge/documentation_verification.md).
- Use the documentation template in [`.agent/templates/package_documentation.md`](.agent/templates/package_documentation.md).

## Project-Level Guidance

When working in a project repository (`layers/main/<layer>_ws/src/<project_repo>/`),
check for `.agents/README.md` at the repo root. If present, read it before making
changes — it contains the package inventory, code layout, architecture overview, and
repo-specific pitfalls documented by previous agents.

If no `.agents/README.md` exists, note this gap. To create one, use the template at
[`.agent/templates/project_agents_guide.md`](.agent/templates/project_agents_guide.md)
and follow the [documentation verification workflow](.agent/knowledge/documentation_verification.md).
This should be a dedicated task with its own issue, not a side-effect of unrelated work.

## Workspace Cleanliness

- Keep repo root and `layers/*/src/` clean — no temp files, build artifacts, or logs.
- Use `.agent/scratchpad/` for persistent temp files (unique names via `mktemp`).
- Use `/tmp` for ephemeral files cleaned up in the same command.

## Post-Task Verification

Before marking a task complete or opening a PR:

1. Re-read issue description and work plan
2. Compare changes against requirements
3. List any gaps; complete them or explain in PR description

## Script Reference

| Script | Purpose |
|--------|---------|
| `.agent/scripts/env.sh` | Source ROS 2 env + checkout guardrail |
| `.agent/scripts/set_git_identity_env.sh` | Ephemeral git identity (session-only) |
| `.agent/scripts/worktree_create.sh` | Create isolated worktree (`--plan-file` to create draft PR with plan) |
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
| `.agent/scripts/detect_agent_identity.sh` | Auto-detect agent framework + model |

## Layered Architecture

```
layers/main/
├── underlay_ws/    # Optional: additional dependencies
├── <overlay>_ws/   # One or more overlay workspaces
│   └── src/        # Independent git repos ("project repos"), each with one or more ROS 2 packages
└── ...
```

`layers/` is gitignored — use `ls` to inspect it, not grep/glob.

## References (Read When Needed, Not Upfront)

- [`README.md` Vision section](README.md#vision) — Workspace purpose and goals
- [`ARCHITECTURE.md`](ARCHITECTURE.md) — System design and layering
- [`docs/decisions/`](docs/decisions/) — Architecture Decision Records
- [`.agent/WORKTREE_GUIDE.md`](.agent/WORKTREE_GUIDE.md) — Detailed worktree patterns
- [`.agent/AI_IDENTITY_STRATEGY.md`](.agent/AI_IDENTITY_STRATEGY.md) — Multi-framework identity
- [`.agent/WORKFORCE_PROTOCOL.md`](.agent/WORKFORCE_PROTOCOL.md) — Multi-agent coordination
- [`.agent/knowledge/`](.agent/knowledge/) — ROS 2 development patterns and CLI best practices
- [`.agent/project_knowledge/`](.agent/project_knowledge/) — Project-specific conventions (symlink, may not exist)
- [`.agent/templates/`](.agent/templates/) — Issue and test templates
