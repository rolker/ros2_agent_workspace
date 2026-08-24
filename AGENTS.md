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
  Exception: **field mode** (origin not on the GitHub allowlist — see
  [`field_mode.sh`](.agent/scripts/field_mode.sh); GitHub Enterprise is
  field mode by design). See Worktree Workflow.
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
- Modify or delete raw survey data (bags, `~/data/logs` — data-of-record)
  without explicit per-action approval. Describing a plan is not consent;
  state the exact operation and stop for a yes.
- Administer remote/field hosts (salmon, gabby, boats): no package
  installs, systemd/service changes, or host-level (`/etc`) config edits —
  surface the need instead. This governs host *system administration*, not
  in-session ROS operations: deployment mode's urgency contract still
  authorizes its in-session mitigations (restart a node, tune a rate).
  SSH access for one purpose does not authorize running commands on other
  hosts; ask first.
- Start unrequested background monitoring/polling loops.
- Disable lint rules, skip tests, or suppress warnings to make a check
  pass — fix the cause; discuss before changing test or lint config.
- Put `close`/`fixes`/`resolves #N` in a PR body or commit message unless
  that PR really should close the issue — see
  [Issue-closing keywords](#issue-closing-keywords).

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
relaxed — see [Field Mode](#field-mode-origin-not-on-a-github-host) below.

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

### Field Mode (origin not on a GitHub host)

When a repo's `origin` host is **not** on the GitHub allowlist (currently
`github.com` and `ssh.github.com` — the SSH-over-443 fallback; see
[`.agent/scripts/field_mode.sh`](.agent/scripts/field_mode.sh) for the
authoritative list), the worktree/PR ceremony is relaxed. Examples of
field-mode origins: gitcloud, private Forgejo, other non-GitHub remotes.
**Field-mode repos may**:

- Edit tracked files directly in the main/default tree
- Commit to the default branch (`main`, `jazzy`, etc.)
- Push to `origin` without opening a PR

This is the only way field hotfixes can land before the next run — the
workspace's PR-and-review workflow is GitHub-based, and a field remote
isn't on GitHub. Other forges (Forgejo, GitLab) may support their own
PR/CI mechanisms, but those don't plug into this workspace's review
pipeline, so we treat them as field mode too. Mode is determined per
repo by origin URL, not by the physical machine: a dev workstation
working in a gitcloud-origin clone is in field mode for that repo.

**What field mode does NOT change** (all still required):

- Pre-commit hooks run — never `--no-verify`
- Commits use the configured agent git identity (set via
  `set_git_identity_env.sh`)
- Atomic commits (one logical change per commit)
- No committing secrets
- No force-push, no destructive ops without explicit user approval

**Hook caveat**: `no-commit-to-branch` is a common pre-commit hook that
blocks direct commits to listed branches. If a field-mode project repo
has its default branch in that list, commits will fail at pre-commit —
even though field mode otherwise permits them. Two templates in this
workspace configure the hook: the project-repo template
(`.agent/templates/pre-commit-config.yaml`, where the project substitutes
`PLACEHOLDER_DEFAULT_BRANCH`), and the workspace's own
`.pre-commit-config.yaml` (which lists `main`, `jazzy`, `rolling`).
Field-mode project repos need to remove their default branch from the
hook's list, or drop the hook. This is a project-repo config concern,
not a field-mode flag. Never bypass with `--no-verify`.

**Detection**: the mode is inferred from the repo's origin URL. Use
[`.agent/scripts/field_mode.sh`](.agent/scripts/field_mode.sh) — it isn't
on PATH, so invoke from the workspace root:

```bash
# Form A: from workspace root, pass the target repo path
.agent/scripts/field_mode.sh --describe layers/main/platforms_ws/src/unh_echoboats_project11
# → field mode  (origin: git@gitcloud:field/unh_echoboats_project11.git)

# Form B: cd into the target repo first, reference the script via the workspace root
cd layers/main/platforms_ws/src/unh_echoboats_project11
../../../../../.agent/scripts/field_mode.sh --describe

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

**Filing discipline**: don't file an issue for every backlog thought — a
dev-log backlog line is enough for on-radar items; file when work is about to
start or the item needs cross-referencing. For consolidation/cleanup work,
default to bundling related changes into one PR (atomic commits inside)
rather than fanning out many small issues. When creating an issue of a
recurring type (deployment, RCA, onboarding), read a recent closed one and
match its structure.

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

### Deployment mode

For live field deployments of an autonomous robot boat, activate **deployment
mode** in the current agent session with `/start-deployment`. The procedure
is framework-agnostic markdown (see
[`.claude/skills/start-deployment/SKILL.md`](.claude/skills/start-deployment/SKILL.md));
slash-command invocation is Claude Code native, but other runtimes can
adapt the same steps. The skill discovers the project's
`.agents/deployment.yaml` config, branches
on dev vs field side via `field_mode.sh`, and either creates a new deployment
(dev side only — field side has no GitHub access and instructs the operator
to start from a dev host first), first-activates an existing one
(worktree on dev / main-tree on field + per-host log + issue sync), or
resumes an ongoing one. Activation is per-agent-session; other sessions
on the same host are unaffected.

See [ADR-0014](docs/decisions/0014-deployment-mode.md) for the decision record
and [`.agent/knowledge/deployment_mode.md`](.agent/knowledge/deployment_mode.md)
for the operational reference (urgency contract, lifecycle phases, log-naming
convention, project-config schema). The wrap-up skill
([`.claude/skills/wrap-up-deployment/SKILL.md`](.claude/skills/wrap-up-deployment/SKILL.md))
closes the deployment issue, reconciles field code, and files RCA follow-ups.
The recovery checklist ([#496](https://github.com/rolker/ros2_agent_workspace/issues/496))
remains a follow-up.

## AI Signature (Required on all GitHub Issues/PRs/Comments)

```markdown
---
**Authored-By**: `$AGENT_NAME`
**Model**: `$AGENT_MODEL`
```

Use your actual runtime identity — never copy example model names from docs.
Check `$AGENT_NAME` / `$AGENT_MODEL` environment variables.

## Agent Commit Identity

For commits on agent-convention branches (`feature/issue-<N>`,
`feature/ISSUE-<N>-<desc>`, `skill/<name>-<timestamp>`), the canonical
pattern is per-invocation `-c` overrides:

```bash
git -c user.name="$AGENT_NAME" \
    -c user.email="$AGENT_EMAIL" \
    commit -m "…"
```

**Why per-commit `-c`, not the env-var script alone**: agent tool runtimes
(Claude Code, Copilot CLI, etc.) typically run each bash invocation in a
fresh subshell. `set_git_identity_env.sh`'s exports
(`GIT_AUTHOR_NAME`/`GIT_AUTHOR_EMAIL`) are bound to the shell that sourced
the script — they don't propagate to a separate `git commit` subshell run
later. The `-c` flags travel with the command and propagate to git's
subprocesses (including pre-commit hooks) via `GIT_CONFIG_PARAMETERS`,
so the identity is robust regardless of which subshell does the commit.

`set_git_identity_env.sh` is still useful — it exports `$AGENT_NAME`,
`$AGENT_EMAIL`, `$AGENT_MODEL`, `$AGENT_FRAMEWORK` for `gh` signatures
and as values for the `-c` flags above. Source it once at session start
and the env vars feed every commit — provided the commits run in the
same shell as the source (or one of its descendants).

**Subshell caveat**: `$AGENT_NAME` / `$AGENT_EMAIL` themselves must be
live in the shell that runs the `git -c` command. Runtimes that start
each command in a fresh subshell (Claude Code's Bash tool, Copilot CLI,
similar harnesses) lose the exports between turns — the same mechanism
that broke `GIT_AUTHOR_NAME`. In that case, either source
`set_git_identity_env.sh` in the same command as the commit, or
substitute the literal values directly into the `-c` flags:

```bash
git -c user.name="<agent name>" \
    -c user.email="<agent email>" \
    commit -m "…"
```

Substitute your own runtime identity for the placeholders — the same
values `set_git_identity_env.sh` exports as `$AGENT_NAME` and
`$AGENT_EMAIL`. Don't copy-paste another agent's identity from
examples elsewhere; commits would land under the wrong author.

**Enforcement**: `.agent/hooks/check-commit-identity.py` (pre-commit)
rejects human-pattern emails on agent-convention branches when
`$AGENT_NAME` is set. A CI check
(`.agent/hooks/check_pr_authors.py`, run from `validate.yml`) catches
the same condition on PRs and is the load-bearing defense when the
hook's env-var gate is bypassed (e.g., `$AGENT_NAME` lost across
subshells). The shared patterns and branch regex live in
`.agent/hooks/identity_patterns.py`.

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

### Issue-Closing Keywords

GitHub's parser auto-closes issues on `close`/`fixes`/`resolves #N` tokens in
PR bodies and commit messages — including **negated** mentions ("does not
close #5") and mentions describing a **sibling** PR. This has closed wrong
issues twice. Rules:

- Use the keyword only for the one issue this PR should actually close.
- For every other mention, write "Part of #N" / "addresses #N" or link the
  URL.
- Work plans pasted into PR bodies inherit this hazard — scrub keyword
  phrases from plan text before pasting.

### Docs-Only PRs

Docs-only PRs omit the Test plan section of the PR body.

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
cd layers/main/<layer>_ws && colcon build --symlink-install --packages-select <package>
# setup.bash must be sourced in the same shell — agents run each command in a fresh subprocess
source .agent/scripts/setup.bash && cd layers/main/<layer>_ws && colcon test --packages-select <package> && colcon test-result --verbose

make lint                                        # Lint + hooks (auto-installs pre-commit)
```

**Always use `--symlink-install`** for development builds — it symlinks Python
files and package markers so edits take effect without a rebuild. `make build`,
the workspace build scripts (`.agent/scripts/build.sh`), and the single-package
example above pass this flag explicitly. Worktree-generated `<layer>_ws/build.sh`
enables symlink installs by default via a generated `colcon/defaults.yaml`
(`symlink-install: true`) rather than via the CLI flag, so raw `colcon build`
from the layer workspace picks it up automatically. (Note: ament_cmake
`install(DIRECTORY ...)` directives still copy their contents, so data-file
edits still need a rebuild.)

**Build in layer directories only** — never `colcon build` from the workspace root.

**In worktrees**: Layer worktrees generate `build.sh` and `test.sh` for the target
(non-symlink) layer workspace. Use `./<layer>_ws/build.sh [pkg]` and
`./<layer>_ws/test.sh [pkg]` instead of raw `colcon` commands — they handle
sourcing lower layers automatically.

Set `NONINTERACTIVE=1` to suppress all interactive prompts (e.g., the first-run
bootstrap confirmation). `CI` is also recognized for CI environments.

### Merging

- **Merge commits, never squash** — preserve the atomic-commit history.
- Merge via `merge_pr.sh --issue <N>` (not `--pr <N>` — the PR-keyed form
  skips worktree cleanup).
- **Gate on the applicable CI verification**: for the **workspace repo**,
  never merge while hosted checks are red or pending. For **project
  repos**, a full-scope `ci-local` attestation satisfies the gate without
  waiting for hosted Actions (see Merge verification below / ADR-0018) —
  but never merge past a *red* signal from whichever verification applies.
  Either way, a poll loop that breaks on failure must not fall through to
  the merge command.
- **Green CI is not review**: never merge a PR the user hasn't
  content-reviewed — doubly so for strategic documents (roadmaps, ADRs,
  instruction files).
- Before merging, fetch and read PR review comments (human and bot) —
  `fetch_pr_reviews.sh` — and triage them; don't merge past unread
  feedback.

### Merge verification (ADR-0018)

Project-repo PRs may merge on a **full-scope local CI attestation** instead of
waiting for hosted Actions: run `.agent/scripts/ci_local.sh <project_repo_path>` on the PR
head, verify `git notes --ref=ci-local show <head-sha>` reports `ci-local: pass`
with `scope: full`, and push `refs/notes/ci-local` to origin at merge time.
Partial/dirty/`--no-attest` runs don't satisfy the gate. Hosted CI on project
repos remains a mirror/backstop — triage its post-merge failures. The
**workspace repo is exempt**: its hosted checks stay required. See
[ADR-0018](docs/decisions/0018-local-first-ci-verification.md).

## Documentation Accuracy

- **Never document from assumptions** — every claim about parameters, topics, services,
  message types, or API signatures must be verified by reading the actual source code.
- Before writing or updating package documentation, read `package.xml`, all source files
  that declare parameters/publishers/subscribers, and any `.msg`/`.srv`/`.action` files.
- Use the verification workflow in [`.agent/knowledge/documentation_verification.md`](.agent/knowledge/documentation_verification.md).
- Use the documentation template in [`.agent/templates/package_documentation.md`](.agent/templates/package_documentation.md).
- **Never hand-type timestamps or measured values** into durable artifacts
  (logs, reports, docs) — generate timestamps with
  `date '+%Y-%m-%d %H:%M %:z'`; mark human-reported times as
  `~HH:MM (operator-reported)`; look measured quantities up (URDF,
  `/tf_static`, configs) instead of estimating them.
- **Never attribute decisions or rationale to a person who didn't state
  them** — record what was actually said; a plausible justification you
  inferred is fabrication, not documentation.
- **Never reference private agent-memory filenames** from repo-tracked
  files (progress.md, plans, READMEs) — inline the relevant content
  instead; other agents and humans can't resolve those references.

## Project-Level Guidance

When working in a project repository (`layers/main/<layer>_ws/src/<project_repo>/`),
check for `.agents/README.md` at the repo root. If present, read it before making
changes — it contains the package inventory, code layout, architecture overview, and
repo-specific pitfalls documented by previous agents.

If no `.agents/README.md` exists, note this gap. To create one, use the template at
[`.agent/templates/project_agents_guide.md`](.agent/templates/project_agents_guide.md)
and follow the [documentation verification workflow](.agent/knowledge/documentation_verification.md).
This should be a dedicated task with its own issue, not a side-effect of unrelated work.

Project repos should also carry a thin root `AGENTS.md` (read by GitHub Copilot code
review) that references — never forks — these workspace rules; create it from
[`.agent/templates/project_agents_md.md`](.agent/templates/project_agents_md.md) (see [ADR-0017](docs/decisions/0017-extend-agents-md-to-project-repos.md)).

## Workspace Cleanliness

- Keep repo root and `layers/*/src/` clean — no temp files, build artifacts, or logs.
- Use `.agent/scratchpad/` for persistent temp files (unique names via `mktemp`).
- Use `/tmp` for ephemeral files cleaned up in the same command.
- **Scope filesystem searches to project paths** (the workspace, `~/data`) —
  never grep/find from `$HOME`: it descends into cloud/network FUSE mounts
  and hangs or floods results.

## Post-Task Verification

Before marking a task complete or opening a PR:

1. Re-read issue description and work plan
2. Compare changes against requirements
3. Check consequences: do tests, docs, or dependent references need updating?
4. List any gaps; complete them or explain in PR description
5. **Before opening a PR**: run `/review-code` against your diff and
   address findings before pushing. Catches static-analysis issues,
   governance gaps, plan drift, and adversarial findings while they're
   still cheap to fix locally — instead of in PR review rounds. See
   `.claude/skills/review-code/SKILL.md` (pre-push mode is the default
   when invoked with no arguments).

**Plan-first workflow**: if the PR opened with a work plan (`.agent/work-plans/issue-<N>/plan.md`,
or legacy `PLAN_ISSUE-<N>.md`),
the plan is expected to stay in sync with the committed implementation on
the branch / in the PR as implementation proceeds. Inline edits are the
default; see the "During implementation" section of
`.claude/skills/plan-task/SKILL.md` for the full rules. This keeps the
plan useful as reference material for review and future agents, and
avoids recurring plan-drift flags from Copilot.

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
| `.agent/scripts/dlog.sh` | Prompt-free, `date`-stamped deployment log appender (`dlog.sh <logfile> <message>`); allowlist once for prompt-free + accurate live-ops logging (#515/#516) |
| `.agent/scripts/progress_append.sh` | Prompt-free progress.md entry appender + scoped committer (`progress_append.sh [-C <dir>] <N> [--title <t>] < entry.md`); validates the ADR-0013 `## <Entry Type>` heading, creates frontmatter, commits only that file under the agent identity (fails loud if unset); allowlisted in tracked `.claude/settings.json` — the shared baseline; per-machine `.claude/settings.local.json` still layers on top (#594) |
| `.agent/scripts/agent start-task <N>` | High-level wrapper: create + enter worktree |
| `.agent/scripts/dispatch_subagent.sh` | Dispatch a workflow skill into a fresh-context sub-agent (in-process or container); reads the progress.md exit contract. `--check` runs the container-auth preflight; `--repo-slug <slug>` disambiguates a layer worktree on cross-repo issue-# collision (#526); `--context-file <path>` splices a host-fetched issue/PR body into the handoff so a no-GitHub-auth container phase reads it instead of `gh issue view` — composable with `--skill` (#552); container auth tokens live at `~/.config/ros2-agent/claude-oauth-token` (Claude, from `claude setup-token`) + `~/.config/ros2-agent/gh-readonly-token` (optional GH read-only) |
| `.agent/scripts/docker_run_agent.sh` | Launch the sandboxed agent container for a worktree; interactive or headless dispatch (`--prompt`/`--prompt-file`, `--model`), reads `CLAUDE_CODE_OAUTH_TOKEN`. `--build` also stages layer `package.xml` manifests into the image build context to bake their rosdep deps (#520). **The only image build path** — `--build-only` builds and exits, and `make agent-build` delegates to it, so the baked startup scripts' digest is stamped from one formula over one directory; every launch compares that digest and warns (never blocks) when the image's `agent-entrypoint.sh`/`fix-volume-ownership.sh` are stale or unmarked (#604). Builds always resolve the **main** workspace root, even when run from a worktree — so a worktree edit to those two scripts is not baked until merged (the build prints a notice when that applies) |
| `.agent/scripts/stage_rosdep_manifests.sh` | Gather layer `package.xml` manifests (recursive) into the agent-image build context for the rosdep bake (#520); called only by `docker_run_agent.sh`'s build block — the single build path, which `make agent-build` reaches via `--build-only` (#604) |
| `.agent/scripts/dashboard.sh` | Unified workspace status (supports `--quick`) |
| `.agent/scripts/build.sh` | Build all layers in order |
| `.agent/scripts/ci_local.sh` | Containerized local CI for a project repo with git-note attestation (`refs/notes/ci-local`); full-scope pass = accepted merge verification (ADR-0018). Builds `upstream.repos` source deps as an underlay with host-resolved SHAs recorded as `upstream-repo:` note lines (#577; per-repo pruning via `.agents/ci_local_upstream_extra.sh` + `.agents/ci_local_rosdep_skip_keys.txt`). `--clean-room` mirrors hosted CI exactly (preferred for `upstream.repos` repos); `-n` dry-run |
| `.agent/scripts/check_branch_updates.sh` | Check if branch is behind default |
| `.agent/scripts/gh_create_issue.sh` | Create issue with label validation (`GITBUG_CREATE=1` for offline) |
| `.agent/scripts/git_bug_setup.sh` | Configure git-bug identity + GitHub bridge |
| `.agent/scripts/gitbug_helpers.sh` | Shared git-bug lookup helpers **(source)** |
| `.agent/scripts/revert_feature.sh` | Revert all commits for an issue |
| `.agent/scripts/sync_repos.py` | Sync all workspace repositories (includes git-bug). Classifies each repo `SYNCED`/`SKIPPED`/`FAILED` and names the cause per repo in the summary. `SKIPPED` (exit 0): dirty tree, detached HEAD, or a repo from an optional layer (`configs/manifest/optional_layers.txt`) that is not on this host. **`FAILED` (the script exits 1; `make sync` reports make's own 2, since GNU make flattens any non-zero recipe status)**: failed pull/fetch, a working tree or git state that cannot be read, a required layer that is not set up, a repo missing from a *required* layer, or no repos enumerable at all (missing `configs/manifest` — e.g. run from a worktree). (The optional-layer skip applies whether or not the layer directory exists: `setup_layers.sh` exits 0 leaving a partially imported optional layer, and `validate_workspace.py` allows the same.) git-bug bridge failures only warn (ADR-0010 graceful degradation). `merge_pr.sh` catches the non-zero exit and reports "merged, but sync failed" (its own exit 3) rather than aborting bannerless (#609). Branch on the exit code only when calling the script directly, not through `make` |
| `.agent/scripts/merge_pr.sh` | Merge a PR + remove worktree + delete branches + `make sync` (worktree/issue-keyed; also `make merge-pr`). **Exit codes** (as returned by the script itself): 0 = merged, cleaned up and synced; 1 = failed **before** the merge — nothing irreversible happened, safe to retry; 2 = usage error; **3 = the merge LANDED but a post-merge step failed** — worktree removal, the `cd` back to the workspace root, or `make sync`; never retry the merge, run the one command the banner names (#609). Every post-merge failure path exits 3, which is what keeps 1's "nothing irreversible happened" true. **Through `make merge-pr` none of these survive**: GNU make reports its own 2 for any non-zero recipe status, so a post-merge failure arrives as exactly the usage-error code — call the script directly if you branch on the exit code |
| `.agent/scripts/add_remote.py` | Add a named remote to all repos (one-time setup) |
| `.agent/scripts/push_remote.py` | Push to a named remote across all repos |
| `.agent/scripts/pull_remote.py` | Fetch/pull from a named remote across all repos (`--json` for structured output) |
| `.agent/scripts/validate_workspace.py` | Validate repos match .repos config |
| `.agent/scripts/detect_agent_identity.sh` | Auto-detect agent framework + model |
| `.agent/scripts/fetch_pr_reviews.sh` | Fetch all PR reviews and CI status |
| `.agent/scripts/local_review.sh` | Cross-model review of a diff via a local Ollama model (default `qwen3.5:35b`); used by review-code specialist 5f, standalone-capable offline/field (#570) |
| `.agent/scripts/progress_read.py` | Extract `progress.md` entries by type + correlation key as JSON (consumed by `triage-reviews` integrator) |
| `.agent/scripts/test_check_commit_identity.sh` | Regression test for `check-commit-identity.py` branch+env gate |
| `.agent/scripts/test_layer_sourcing.sh` | Regression guard for runtime layer chaining (ADR-0016 / #559, #566): static no-baked-chain-source check + built-layer overlay precedence + baked-chain purity + host-side mountpoint ownership of `layers/main` (Check 4 covers neither worktree-scoped nor container-side anonymous-volume ownership — see `tests/test_entrypoint_chown_coverage.sh`, #604); run by `make test-scripts` / `make validate` |
| `.agent/hooks/identity_patterns.py` | Shared agent/human email patterns + agent-branch regex (imported by commit-identity hooks + CI script) |
| `.agent/hooks/check_pr_authors.py` | CI-callable PR-commit author validator (Mechanism C from issue #468) |
| `.agent/hooks/check_question_context.py` | Warn-only, always-on `PreToolUse` hook (matcher `AskUserQuestion`, wired in `.claude/settings.json`): nudges when no question opens with a repo-qualified `<repo>#<N>` re-orientation header (#592). Fails safe — any parse/schema error exits 0 silently; never blocks a checkpoint |

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
