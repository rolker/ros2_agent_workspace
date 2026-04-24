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

- Use worktrees for all feature work — never edit files in the main tree.
  Exception: **field mode** (origin not github.com) — see Worktree Workflow.
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
- Branch naming: `feature/issue-<N>` or `feature/ISSUE-<N>-<description>`
- All changes via Pull Requests on GitHub-origin repos (field-mode repos
  push to the field remote without PRs — see Worktree Workflow)

### Ask First (get human approval)

- Modifying instruction files (`AGENTS.md`, `CLAUDE.md`, etc.)
- Adding or removing workspace layers
- Changing CI or branch protection configuration

### Never (hard stops)

- Commit directly to the default branch (e.g. `main`, `jazzy`) on a
  GitHub-origin repo — branch protection rejects direct pushes; open a PR
  from a worktree. Field-origin repos have their own workflow (see
  Worktree Workflow).
- `git checkout <branch>` — `setup.bash` blocks it; use worktrees
- Skip hooks with `--no-verify`
- Commit secrets or credentials
- Document from assumptions — verify against source code
- Construct GitHub URLs from directory names — use `gh` CLI to look them up
- Run bare `pip install` or use `--break-system-packages` — use `.venv` for dev tools (see ADR-0009)

## Quality Standard

This is software for autonomous robot boats operating on open water. Robustness
is not optional. The marginal cost of completeness is near zero with AI — do the
whole thing, do it right, do it with tests.

- When fixing a bug, fix it completely: add the test, handle the edge case, check
  the lifecycle transition. Never leave a "good enough" fix when the proper one is
  within reach.
- When triaging reviews, do not dismiss concerns about error handling, silent
  failures, stale data, or missing validation as "nits" unless the failure mode
  genuinely cannot occur. "Config is under our control" and "pathological input"
  are not blanket dismissals — field configs change under pressure.
- When importing field fixes, treat them as drafts: add tests, verify topic names,
  check for idempotency. The PR is the quality gate.
- Never offer to "table this for later" when the permanent solve is five minutes
  away. Never present a workaround when the real fix exists.

## Worktree Workflow

Every task on a GitHub-origin repo must use an isolated worktree. For
non-GitHub-origin repos (field mode), the worktree/PR ceremony is
relaxed — see [Field Mode](#field-mode-origin-not-githubcom) below.

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

# Alternative: set up manually using generated convenience scripts
cd <worktree_path>                 # path printed by worktree_create.sh
source setup.bash                  # set up ROS environment
./<layer>_ws/build.sh [pkg ...]    # build (uses colcon with correct flags)
./<layer>_ws/test.sh [pkg ...]     # test

# Multiple project repos in one worktree
.agent/scripts/worktree_create.sh --issue <N> --type layer --layer <layer> --packages <repo1>,<repo2>

# Sub-issue work (branches from parent's feature branch, targets PR at it)
.agent/scripts/worktree_create.sh --issue <N> --type workspace --parent-issue <parent_N>
.agent/scripts/worktree_create.sh --issue <N> --type layer --layer <layer> --packages <project_repo> --parent-issue <parent_N>

# List / remove
.agent/scripts/worktree_list.sh
.agent/scripts/worktree_remove.sh --issue <N>
```

See [`.agent/WORKTREE_GUIDE.md`](.agent/WORKTREE_GUIDE.md) for hybrid structure details,
`--repo-slug` disambiguation, and troubleshooting.

### Field Mode (origin not github.com)

When a repo's `origin` host is **not** `github.com` (e.g., gitcloud, private
Forgejo), the worktree/PR ceremony is relaxed. **Field-mode repos may**:

- Edit tracked files directly in the main/default tree
- Commit to the default branch (`main`, `jazzy`, etc.)
- Push to `origin` without opening a PR

This is the only way field hotfixes can land before the next run — there's no
GitHub, no PR review, no CI on the field remote. Mode is determined per repo
by origin URL, not by the physical machine: a dev workstation working in a
gitcloud-origin clone is in field mode for that repo.

**What field mode does NOT change** (all still required):

- Pre-commit hooks run — never `--no-verify`
- Commits use the configured agent git identity (set via
  `set_git_identity_env.sh`)
- Atomic commits (one logical change per commit)
- No committing secrets
- No force-push, no destructive ops without explicit user approval

**Detection**: the mode is inferred from the repo's origin URL. Use
[`.agent/scripts/field_mode.sh`](.agent/scripts/field_mode.sh) — it isn't
on PATH, so invoke from the workspace root:

```bash
# Form A: from workspace root, pass the target repo path
.agent/scripts/field_mode.sh --describe layers/main/platforms_ws/src/unh_echoboats_project11
# → field mode  (origin: git@gitcloud:field/unh_echoboats_project11.git)

# Form B: cd into the target repo first, reference the script via the workspace root
cd layers/main/platforms_ws/src/unh_echoboats_project11
../../../../.agent/scripts/field_mode.sh --describe

# Sourced in a script — source by explicit path (the script is not on PATH).
# is_field_mode takes an optional repo_dir arg, defaulting to $PWD.
source /path/to/workspace/.agent/scripts/field_mode.sh
if is_field_mode; then                  # checks current $PWD
    # field-mode behavior
fi
if is_field_mode "/path/to/repo"; then  # check a specific repo
    # ...
fi
```

**Reconciliation**: field commits come back to GitHub via the
`/import-field-changes` skill on a connected dev machine. Use that skill
rather than hand-rolling cherry-picks — it creates branches without
perturbing the main tree HEAD, opens draft PRs with pre-review against
the Quality Standard, and flags diverged repos for human merge.

## Issue-First Policy

No code without a ticket. Check for an existing GitHub issue first; if none exists,
ask the user: "Should I open an issue to track this?" Use the issue number in branches
and reference it in PRs with `Closes #<N>`.

Issues and PRs live in whichever repo owns the code being changed — check the project
repo, not just the workspace repo.

**Trivial fixes** (typos, minor doc corrections) don't need a dedicated issue — use the
current task's worktree or create a quick issue for a new one.

**Sub-tasks**: When creating a new issue as a sub-task of existing work, reference
the parent issue in the issue body (e.g., "Part of #NNN"). Use full
`owner/repo#NNN` syntax for cross-repo references. The reference must be in the
issue body — GitHub only auto-links body mentions in the sidebar.

`gh_create_issue.sh` auto-injects this reference when `$WORKTREE_ISSUE` is set.
When creating a worktree for a sub-issue, use `--parent-issue <N>` so the worktree
branches from the parent's feature branch and the draft PR targets it (stacked PR).

**Verify before committing**: Before your first commit, confirm the issue matches
your task: `gh issue view $WORKTREE_ISSUE --json title --jq '.title'`
If the title does not match, stop — you may be in the wrong worktree or have the
wrong issue number.

### Skill Worktree Exception

Some skills maintain living documents (research digests, project knowledge summaries)
that need worktree isolation and PR review but don't warrant a dedicated GitHub issue
for each recurring update. These skills may use `--skill <name>` instead of
`--issue <N>` across the worktree scripts:

```bash
.agent/scripts/worktree_create.sh --skill research --type workspace
source .agent/scripts/worktree_enter.sh --skill research
.agent/scripts/worktree_remove.sh --skill research
```

**Allowed skills**: `research`, `inspiration-tracker` (enforced by an allowlist in `worktree_create.sh`).

**Branch naming**: `skill/{name}-{YYYYMMDD-HHMMSS-NNNNNNNNN}` (e.g., `skill/research-20260227-143022-123456789`).

**Requirements**: Skill worktrees still require worktree isolation and PR review —
the only exception is that no GitHub issue is needed. All other rules (atomic commits,
AI signature, pre-commit hooks) still apply.

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

`make build` handles the core setup chain automatically — on a fresh clone it
runs bootstrap, manifest import, and layer setup before building. On subsequent
runs it skips already-completed steps (stamp files in `.make/`). Dev-tools
(pre-commit, venv) are installed separately via `make lint`. Use `make clean`
to reset stamps and force a full re-setup.

```bash
make build                                       # Build all layers (auto-setup on first run)
make test                                        # Run all tests (builds first if needed)
make validate                                    # Validate workspace config (CI-oriented)
make dashboard                                   # Unified workspace status
make dashboard QUICK=1                           # Quick mode (skip sync + GitHub)

# Single package
cd layers/main/<layer>_ws && colcon build --packages-select <package>
# setup.bash must be sourced in the same shell — agents run each command in a fresh subprocess
source .agent/scripts/setup.bash && cd layers/main/<layer>_ws && colcon test --packages-select <package> && colcon test-result --verbose

make lint                                        # Lint + hooks (auto-installs pre-commit)
```

**Build in layer directories only** — never `colcon build` from the workspace root.

**In worktrees**: Layer worktrees generate `build.sh` and `test.sh` for the target
(non-symlink) layer workspace. Use `./<layer>_ws/build.sh [pkg]` and
`./<layer>_ws/test.sh [pkg]` instead of raw `colcon` commands — they handle
sourcing lower layers automatically.

Set `NONINTERACTIVE=1` to suppress all interactive prompts (e.g., the first-run
bootstrap confirmation). `CI` is also recognized for CI environments.

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
3. Check consequences: do tests, docs, or dependent references need updating?
4. List any gaps; complete them or explain in PR description

## Script Reference

`scripts/` at the repo root is a symlink to `.agent/scripts/` for convenience.

Scripts marked **(source)** must be sourced (`source scripts/foo.sh`); all others
should be executed (`./scripts/foo.sh` or `scripts/foo.sh`). Execute-only scripts
include a guard that prints an error if accidentally sourced.

| Script | Purpose |
|--------|---------|
| `.agent/scripts/setup.bash` | Source ROS 2 env + checkout guardrail **(source)** |
| `.agent/scripts/set_git_identity_env.sh` | Ephemeral git identity (session-only) **(source)** |
| `.agent/scripts/worktree_create.sh` | Create isolated worktree (`--plan-file` to create draft PR with plan) |
| `.agent/scripts/worktree_enter.sh` | Enter worktree (must be sourced) **(source)** |
| `.agent/scripts/worktree_remove.sh` | Remove worktree |
| `.agent/scripts/worktree_list.sh` | List active worktrees |
| `.agent/scripts/field_mode.sh` | Detect field mode (non-GitHub origin) vs. dev mode **(source or exec)** |
| `.agent/scripts/agent start-task <N>` | High-level wrapper: create + enter worktree |
| `.agent/scripts/dashboard.sh` | Unified workspace status (supports `--quick`) |
| `.agent/scripts/build.sh` | Build all layers in order |
| `.agent/scripts/check_branch_updates.sh` | Check if branch is behind default |
| `.agent/scripts/gh_create_issue.sh` | Create issue with label validation (`GITBUG_CREATE=1` for offline) |
| `.agent/scripts/git_bug_setup.sh` | Configure git-bug identity + GitHub bridge |
| `.agent/scripts/gitbug_helpers.sh` | Shared git-bug lookup helpers **(source)** |
| `.agent/scripts/revert_feature.sh` | Revert all commits for an issue |
| `.agent/scripts/sync_repos.py` | Sync all workspace repositories (includes git-bug) |
| `.agent/scripts/add_remote.py` | Add a named remote to all repos (one-time setup) |
| `.agent/scripts/push_remote.py` | Push to a named remote across all repos |
| `.agent/scripts/pull_remote.py` | Fetch/pull from a named remote across all repos (`--json` for structured output) |
| `.agent/scripts/validate_workspace.py` | Validate repos match .repos config |
| `.agent/scripts/detect_agent_identity.sh` | Auto-detect agent framework + model |
| `.agent/scripts/fetch_pr_reviews.sh` | Fetch all PR reviews and CI status |

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
- [`.agent/project_knowledge/`](.agent/project_knowledge/) — Symlink to manifest repo's `.agents/workspace-context/` (gitignored, created by `setup_layers.sh`; may not exist)
- [`.agent/templates/`](.agent/templates/) — Issue and test templates
