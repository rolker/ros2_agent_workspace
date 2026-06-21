---
name: wrap-up-deployment
description: Close out a field deployment. Verifies dev-side access, finds the open deployment issue, collects field logs, interviews the operator for corrections, consolidates the dev log, reconciles field code via SHA-preserving merge and /import-field-changes, opens the wrap-up PR (Closes #N), merges, reconciles gitcloud, and files new RCA issues selectively.
---

# Wrap Up Deployment

## Usage

```
/wrap-up-deployment
```

No arguments. The skill discovers everything it needs from the current
working directory's project config (`<project_repo>/.agents/deployment.yaml`),
`field_mode.sh`, and the open deployment issue. Run from inside the project
repo or the workspace root. **Dev side only** — wrap-up requires `gh` access
to close the deployment issue and open PRs; the skill stops immediately if
run from a field host.

## Overview

Close out a deployment under **deployment mode**
([ADR-0014](../../../docs/decisions/0014-deployment-mode.md)) — the complement
to `/start-deployment`. Operationally: pull the field logs in, interview the
operator to catch agent misreads before they become permanent record, consolidate
the dev log, reconcile field code, open and merge the wrap-up PR (which closes
the deployment issue), and file any genuinely new RCA issues.

**Lifecycle position**: [0] start → [1] session → [2] recovery → [3] wrap-up (this skill) → [4] next

**The urgency contract relaxes here.** Wrap-up is where deep analysis is the
*expected* work — root-cause investigation, log integration, lessons-learned
synthesis. Sterile-cockpit bites hardest during live ops [1]; at [3] the boat
is out of the water and there is no real-time cost to investigation depth. The
invariants (pre-commit hooks, AI signature, atomic commits, no secrets, no
destructive ops without operator approval) do not relax.

**Bag extraction and data-analysis** (mission databases, trajectory post-processing,
sensor QA) are operator-driven steps that belong in the project's debrief checklist
([#435](https://github.com/rolker/ros2_agent_workspace/issues/435)). They are
deliberately out of scope for this skill: the commands and paths are
project-specific and cannot be abstracted generically at the workspace level
(ADR-0003). `/wrap-up-deployment` covers the dev-record and code-reconciliation
halves only.

The fuller operational reference is
[`.agent/knowledge/deployment_mode.md`](../../../.agent/knowledge/deployment_mode.md).

**ADR-0013 note**: this skill does **not** write a `progress.md` entry — it is
a deployment-lifecycle skill (writes the deployment dev log and closes the
deployment GitHub issue), not a per-issue development-pipeline step.

## Steps

### 1. Read the project config

Locate `<project_repo>/.agents/deployment.yaml` using the same discovery
logic as `/start-deployment`:

- If the current working directory is inside a project repo, the config is at
  `<repo_root>/.agents/deployment.yaml`.
- If the current working directory is the workspace root, ask the operator
  which platform's deployment is wrapping up.

If the config file is missing, stop with:

> `<project_repo>/.agents/deployment.yaml` not found. See
> `.agent/templates/deployment_config.yaml` for the template and
> `.agent/knowledge/deployment_mode.md` for the schema.

Read the config and extract: `platform`, `default_branch`, `log_dir`,
`packages`, `layer`, `issue_sync` (optional on dev side), `labels`.

**Validation**: if any required value is the literal string `TODO` (or empty /
null where required), stop with:

> `<key>` is unconfigured in `<project_repo>/.agents/deployment.yaml` —
> fill in before running `/wrap-up-deployment`.

Required keys: `platform`, `default_branch`, `log_dir`, `packages[0]`, `layer`.

### 2. Verify dev-side

Run `field_mode.sh` against the project repo's path. Discover the workspace
root by walking up from `$PWD` until a directory containing
`.agent/scripts/setup.bash` is found:

```bash
# Form A — CWD is workspace root:
.agent/scripts/field_mode.sh --describe layers/main/<layer>_ws/src/<packages[0]>

# Form B — CWD is inside the project repo:
<workspace_root>/.agent/scripts/field_mode.sh --describe
```

If `field_mode.sh` reports field mode, **stop** with:

> `/wrap-up-deployment` requires `gh` access (to close the deployment issue
> and open PRs). Run this from a dev host. On field hosts, finish any
> in-session logging and recovery ([2]), then hand off to a dev host for
> wrap-up.

Record the workspace root path; subsequent steps use it to call workspace-root
scripts from any working directory.

### 3. Find the open deployment issue

```bash
gh issue list --label deployment --state open --json number,title,createdAt,body
```

Run from inside the project repo (or with `-R <owner/repo>` if not). `gh`
auto-detects the remote from `origin`.

- **No open deployment issue**: before concluding there's nothing to wrap up,
  **scan `<log_dir>` for an issue-less field start (#533)** — a per-host log
  whose header line is the canonical `Deployment issue: pending`
  (`grep -rl '^Deployment issue: pending' <log_dir>`). If found, that
  deployment was started issue-less and never backfilled: **create + link its
  issue now** (dated from the log filename), clear the `pending` marker, then
  proceed to wrap it up. Otherwise confirm with the operator whether the
  no-issue state was intentional.
- **Exactly one**: proceed.
- **Multiple**: list them and ask the operator which deployment this wrap-up is for.

Note the issue number `<N>` and its start date (from the title:
`Deployment <YYYY-MM-DD>: <scope>`). The start date drives log-file matching
in the next step.

If `issue_sync.dev_push` is absent from the config, note this and continue —
dev-side `issue_sync` is optional; `gh` alone handles issue edits.

### 4. Collect per-host field logs

The deployment start date from the issue title is `<start-date>` (format
`YYYY-MM-DD`).

Fetch the field remote so its branches are up to date. Use `git -C` to target
the project-repo worktree (not the workspace root); the `gitcloud` remote is
configured there. The project-repo worktree path is derived from the config:
`<workspace_root>/layers/worktrees/issue-<REPO_SLUG>-<N>/<layer>_ws/src/<packages[0]>` —
where `<REPO_SLUG>` is the project-repo directory name — confirm with
`<workspace_root>/.agent/scripts/worktree_list.sh`.

```bash
git -C "<project-repo-worktree-path>" fetch gitcloud
```

(If `gitcloud` is not a configured remote for the project repo, ask the
operator for the correct field-remote name. If the fetch fails — network
unreachable, wrong remote name — note the error, continue with whatever log
files are already on disk, and revisit the fetch before step 5.)

Gather all log files for this deployment in `<log_dir>/<YYYY>/`:

```bash
ls "<project-repo-worktree-path>/<log_dir>/<YYYY>/<start-date>_*_logs.md"
```

If the glob matches no files, note "no per-host field logs found for
`<start-date>`" and continue — do not abort.

Read each file. List them for the operator: one line per file, showing the
host label extracted from the filename (`<start-date>_<label>_logs.md`).
Note any log whose label doesn't match a known host from
`deployment.yaml:hosts.field` — these may be mislabeled and will be raised
in the operator interview.

Also read the dev log (`<start-date>_dev_logs.md`) — it is the canonical sink
for wrap-up additions. Check that it exists; if not, create it with a minimal
header now (using `dlog.sh` for the timestamp):

```bash
LOGFILE="<project-repo-worktree-path>/<log_dir>/<YYYY>/<start-date>_dev_logs.md"
<workspace_root>/.agent/scripts/dlog.sh "$LOGFILE" "wrap-up started"
```

### 5. Reconcile field code — SHA-preserving merge

Merge the deployment repo's own field commits into the wrap-up branch
**before** interviewing the operator. Editing field logs in place before the
gitcloud merge risks merge conflicts on every corrected log — merging first
gives a clean, up-to-date base for the operator interview and corrections.

All git operations below target the **project-repo worktree** — the git repo
for `<packages[0]>` inside the deployment worktree, where the `gitcloud` and
`origin` remotes live:

```
<project-repo-worktree-path> =
  <workspace_root>/layers/worktrees/issue-<REPO_SLUG>-<N>/<layer>_ws/src/<packages[0]>
```

(Confirm with `<workspace_root>/.agent/scripts/worktree_list.sh` if the exact
path is uncertain.)

Re-fetch gitcloud to get its latest state before the merge — field hosts may
have pushed commits during the interval since step 4:

```bash
git -C "<project-repo-worktree-path>" fetch gitcloud
```

If the fetch fails, stop and report to the operator before merging — merging
from a stale remote risks missing field commits. Resolve the fetch error
(network, wrong remote name) before continuing.

Then merge (carry identity flags — merge commits do not trigger the pre-commit
identity hook, so the identity must travel with the command; `$AGENT_NAME` and
`$AGENT_EMAIL` come from `set_git_identity_env.sh`; substitute literal values
if the env vars are not live in this shell — AGENTS.md § Agent Commit Identity):

```bash
GIT_EDITOR=true git -C "<project-repo-worktree-path>" \
    -c user.name="$AGENT_NAME" \
    -c user.email="$AGENT_EMAIL" \
    merge gitcloud/<default_branch> \
    -m "merge: field commits from gitcloud/<default_branch> into wrap-up branch"
```

If conflicts arise that cannot be cleanly resolved, abort and surface for the
operator:

```bash
git -C "<project-repo-worktree-path>" merge --abort
```

Do not auto-resolve anything that touches control logic.

After a successful merge, the wrap-up branch is ahead of both
`origin/<default_branch>` (via the PR) and `gitcloud/<default_branch>`
(which will fast-forward after merge). Record the merge commit SHA in the
dev log via `dlog.sh`.

### 6. Operator-correction interview

**This is the highest-value step. Do not skip or compress it.**

Field agents log in good faith during live ops — but under sterile-cockpit
pressure, with limited time, they make errors: misdated entries, wrong host
labels, premature conclusions, agent overreach that the operator didn't
actually endorse. These errors propagate into the permanent dev record if not
caught here.

**Interview the operator ONE QUESTION AT A TIME** — ask a question, wait for
the answer, then ask the next. Do not present a form or a numbered list to fill
in all at once; operator attention is fragmented at wrap-up.

Minimum questions:

1. "Does the deployment start date `<start-date>` match when the boat actually
   went in the water? (Checking log filename accuracy.)"
2. For each field-log label that looks suspect (unknown host, misspelled
   hostname, etc.): "Log file `<name>` — does this match what you'd expect for
   `<platform>` on this deployment?"
3. "Were there any events or decisions in the field logs that the agent logged
   incorrectly or overstated? (Root causes, action taken, outcome?)"
4. "Anything the agent concluded that you'd walk back — either the root cause
   was wrong, or the mitigation was the agent's idea that you didn't actually
   try?"
5. "Any events or runs that didn't make it into the logs at all?"

Record the operator's answers verbatim or close to it — these become the
`## Operator Corrections` section in the next step. If the operator says
"no corrections," record that explicitly; it distinguishes "we checked" from
"we forgot to ask."

Ask follow-up questions if an answer surfaces a new issue (e.g., a mislabeled
log file implies all its timestamps may also be off). This step is conversational,
not mechanical.

### 7. Consolidate the dev log

Append the following sections to the dev log file
(`<log_dir>/<YYYY>/<start-date>_dev_logs.md`). Use
`<workspace_root>/.agent/scripts/dlog.sh` for any entries that need a
timestamp; write Markdown section headers and body prose directly.

#### 7a. Apply operator corrections to field logs in place

For each correction from the interview that pertains to a specific field log
file (wrong label, misdated entry, incorrect root cause attributed to the
agent):

- **Mislabeled log file**: use `git mv` to rename the file to the correct name
  (preserving the `<start-date>_<label>_logs.md` pattern with the corrected
  label) — a shell-glob `git add` only stages existing paths and leaves the
  old path as an unstaged deletion:

  ```bash
  git -C "<project-repo-worktree-path>" mv \
      "<log_dir>/<YYYY>/<start-date>_<old-label>_logs.md" \
      "<log_dir>/<YYYY>/<start-date>_<corrected-label>_logs.md"
  ```

- **Misdated entry**: edit the entry's timestamp in the file to the corrected
  value; add an inline note: `<!-- corrected at wrap-up: operator reported
  actual time was <HH:MM> -->`.
- **Incorrect conclusion**: add a correction block directly after the
  offending entry:

  ```
  > **Wrap-up correction**: [operator's correction verbatim or paraphrased]
  ```

Stage all corrected field log files (including any renames already done with
`git mv`):

```bash
git -C "<project-repo-worktree-path>" add -A "<log_dir>/<YYYY>/"
```

#### 7b. Append structured summary to dev log

Append these four sections to the dev log (use `dlog.sh` only for
timestamped *events*; section headers and prose go in directly):

```markdown
## Summary

<2–4 sentences: what ran, what worked, what broke, overall outcome.
Written in past tense. No invented mechanisms; only what happened.>

## Operator Corrections

<Verbatim or close-to-verbatim record of corrections from step 6.
If the operator reported no corrections, write "Operator confirmed
logs accurate; no corrections." Attribute each item: "Operator:
'…'">

## Lessons Learned

<Observations that should change future deployments: tooling gaps,
config values to update, procedures that helped or hurt.
Bullet list. No speculation.>

## RCA / Follow-up Backlog

<Items deferred from live ops or surfaced by this wrap-up that
need a GitHub issue. Format: "- [ ] <title> — <one-line root
cause or outstanding question>". Dedup against existing open issues
before this list drives issue-filing in step 10.>
```

#### 7c. Stamp the dev log link in the deployment issue

Verify the deployment issue's `## Logs` section includes a link to the dev
log:

```
- [dev log](<log_dir>/<YYYY>/<start-date>_dev_logs.md)
```

If the dev log link is missing, add it:

```bash
gh issue edit <N> --body-file <tmpfile>
```

(Read the current body first, append the link, write to a temp file, then
`--body-file`. Use `gh issue view <N> --json body --jq '.body'` to get the
current body.)

#### 7d. Commit the consolidated dev log

Stage the dev log and all corrected field logs, then commit. This commit
**must land on the branch before `gh pr create`** in step 9 — without it the
log additions and operator corrections are never part of the PR and are lost
when the worktree is removed:

```bash
git -C "<project-repo-worktree-path>" add \
    "<log_dir>/<YYYY>/<start-date>_dev_logs.md" \
    "<log_dir>/<YYYY>/<start-date>_*_logs.md"
git -C "<project-repo-worktree-path>" \
    -c user.name="$AGENT_NAME" \
    -c user.email="$AGENT_EMAIL" \
    commit -m "chore: wrap-up deployment #<N> — consolidated dev log and operator corrections"
# $AGENT_NAME/$AGENT_EMAIL come from set_git_identity_env.sh; substitute
# literal values if the env vars are not live in this shell (AGENTS.md § Agent Commit Identity).
```

### 8. `/import-field-changes` — other repos

Other repos that received field commits (nav packages, sensors, interfaces,
etc.) use the `/import-field-changes` skill — that skill opens per-repo import
PRs with pre-review against the Quality Standard. `/wrap-up-deployment` does
NOT attempt to merge those repos directly; it delegates:

> Invoke `/import-field-changes` for each repo that has field commits on
> `gitcloud` but is NOT the primary deployment repo (`packages[0]`).

**Prerequisite**: `/import-field-changes` reads `field_remote` from
`.agent/project_config.yaml` (not `.agents/deployment.yaml`) — verify that
file exists and contains the `field_remote` key before invoking the skill.
If missing, create it (run from workspace root):
`echo "field_remote: gitcloud" > .agent/project_config.yaml`
(substituting the actual remote name if different).

Ask the operator: "Which other repos received field commits during this
deployment?" Run `/import-field-changes` for each answer. Record the resulting
PR URLs in the dev log.

If no other repos had field commits, record that explicitly in the dev log:
"No other repos had field commits; `/import-field-changes` not needed."

### 9. PR → merge → gitcloud reconcile

**If re-running after interruption**, check what already completed before
repeating any irreversible action:

```bash
gh pr list --search "Wrap up deployment #<N>" --state all --json number,state,title
gh issue view <N> --json state --jq '.state'
```

- PR already merged and issue closed → skip PR create and merge; proceed to
  the gitcloud reconcile if not yet done, then step 10.
- PR open but not merged → skip `gh pr create`; proceed from the merge step.
- Issue already closed and gitcloud already reconciled → skip to step 10 if
  not yet done.

**Push the deployment branch** so `gh pr create` can find it on origin — the
feature branch is local-only until this push, and with two remotes on the
project repo `gh` cannot auto-pick:

```bash
BRANCH="$(git -C "<project-repo-worktree-path>" branch --show-current)"
# or: BRANCH="feature/issue-<N>"
git -C "<project-repo-worktree-path>" push -u origin "$BRANCH"
```

**Open the wrap-up PR** (skip if already open or merged):

```bash
BODY_FILE=$(mktemp /tmp/gh_body.XXXXXX.md)
# Write PR body to $BODY_FILE — include dev-log link, summary, operator-corrections summary
gh pr create \
    --title "Wrap up deployment #<N>: <scope>" \
    --body-file "$BODY_FILE"
rm "$BODY_FILE"
```

The PR body must include:
- `Closes #<N>` so GitHub closes the deployment issue on merge.
- Link to the deployment issue.
- One-paragraph summary from the dev log's `## Summary` section.
- Link to each per-host log file and each import-field-changes PR.
- AI signature.

**Merge** — use a **merge commit** (not squash, not rebase). `gh pr merge --merge`
with pending required CI checks fails immediately — nothing drives the merge from
that point — so poll checks first:

```bash
# 1. Wait for required CI checks to pass (re-run until all required checks show "pass")
gh pr checks <PR_number>

# 2. Once checks are green, merge
gh pr merge <PR_number> --merge
# Alternative when the repo has auto-merge enabled:
#   gh pr merge <PR_number> --merge --auto
```

Squash would discard the field-commit history folded in by step 5; rebase
would break the SHA-preserving intent. The deployment issue closes automatically
via the `Closes #<N>` reference.

**Confirm the PR merged** before starting the gitcloud reconcile:

```bash
gh pr view <PR_number> --json state --jq '.state'
# Must output "MERGED" before continuing
```

Do not force-push or skip checks.

**Reconcile gitcloud** after the PR merges to `<default_branch>`. Re-fetch
both remotes to get their latest states — gitcloud may have received commits
during the long wrap-up window:

```bash
git -C "<project-repo-worktree-path>" fetch gitcloud
git -C "<project-repo-worktree-path>" fetch origin
```

Verify `origin/<default_branch>` is ahead of or equal to
`gitcloud/<default_branch>` before pushing — the push must be a fast-forward:

```bash
git -C "<project-repo-worktree-path>" merge-base --is-ancestor \
    gitcloud/<default_branch> origin/<default_branch>
```

If this exits non-zero, `gitcloud/<default_branch>` has commits not yet in
`origin/<default_branch>`. **Do not push; do not force-push.** Surface for
manual reconcile:

> `gitcloud/<default_branch>` has diverged from `origin/<default_branch>`.
> Inspect with:
> `git -C <repo> log --oneline gitcloud/<default_branch>..origin/<default_branch>`
> and its reverse. Resolve with the operator before pushing.

Only after the ancestor check passes:

```bash
git -C "<project-repo-worktree-path>" push gitcloud origin/<default_branch>:<default_branch>
```

**Verify the push succeeded** before proceeding. If the push fails, stop here —
the worktree is the retry context. Do not remove it.

This fast-forwards `gitcloud/<default_branch>` to match `origin/<default_branch>`,
completing the SHA-preserving round-trip from step 5.

If `issue_sync.dev_push` is configured **and is not the literal string `TODO`**,
run it now to propagate the closed issue status to the field-visible issue source:

```bash
<issue_sync.dev_push>
```

On failure, print `issue_sync.failure_hint`. This is advisory — the GitHub
issue is already closed; `dev_push` just keeps the field-side view in sync.

### 10. File new RCA issues selectively

File RCA issues **before** removing the worktree — worktree removal breaks
`gh` origin auto-detection for the project repo.

From the `## RCA / Follow-up Backlog` in the dev log and any items surfaced
during wrap-up, decide which warrant a GitHub issue.

**Dedup first.** Before filing any issue, run:

```bash
gh issue list --search "<title keywords>" --state open
```

If a matching issue already exists, add a comment linking the deployment
evidence rather than opening a duplicate. Known / already-on-radar items
stay in the dev-log backlog — do not re-file them.

**File selectively.** The operator has issue overload. File issues only for:
- Bugs confirmed as root causes (not suspected).
- Tooling or config gaps that will repeat without a fix.
- Follow-up analysis that needs tracking (e.g., "analyze bag from run 3 for
  the heading oscillation observed").

Do NOT file issues for:
- Speculation or unconfirmed hypotheses.
- Items the operator explicitly said "we know about it."
- One-off field conditions unlikely to repeat.

Each filed issue gets the project's standard labels plus `rca` if appropriate.
If the `rca` label does not exist in the project repo, create it before filing:

```bash
gh label create rca --color B60205 --description "Root cause analysis" 2>/dev/null || true
```

This is a create-or-skip: the `|| true` means a pre-existing label or a
missing-label error does not abort issue filing. Include the AI signature and a
link to the deployment issue.

**Close stale deployment issues** — if there are leftover open `deployment`-labeled
issues from prior deployments that were never wrapped up, list them for the
operator and ask whether to close them with a note. Do not close them unilaterally.

**Remove the worktree** (only after RCA filing is complete and the gitcloud
push in step 9 succeeded):

```bash
<workspace_root>/.agent/scripts/worktree_remove.sh --issue <N>
```

## Guidelines

- **This is a dev-side-only skill.** The very first step after reading config is
  checking for dev side. Field-side wrap-up is not possible — no `gh` access,
  no PR, no merges.
- **The operator-correction interview is the highest-value step.** Field agent logs
  are written under sterile-cockpit pressure and may contain misattributions,
  premature conclusions, or events that didn't happen as logged. This is the only
  opportunity to fix the permanent record before it propagates.
- **Two merge mechanisms, not one.** The SHA-preserving `git merge` (step 5) is
  for the primary deployment repo. `/import-field-changes` (step 8) is for
  everything else. Do not conflate them — cherry-pick or squash on the primary
  repo would break the gitcloud fast-forward.
- **Merge commit, not squash.** The PR merge MUST be `--merge`. Squash discards
  the field-commit SHAs folded in by step 5; the resulting non-fast-forward would
  be refused by gitcloud when pushing (step 9's ancestor check would also fail).
  The merge approach keeps gitcloud reconcilable by fast-forward.
- **`issue_sync` is optional on dev side.** If `dev_push` is absent from the
  config, skip silently — `gh` handles everything. Only field-side wrap-up
  (which is disallowed) would need the full `issue_sync` suite.
- **Bag extraction is out of scope.** Data analysis, bag database ingestion, and
  trajectory post-processing are operator-driven, project-specific steps that
  belong in the project's debrief checklist (#435). Document where the bags are;
  don't attempt extraction from the SKILL.
- **`dlog.sh` for timestamped entries, prose for sections.** Timestamped log
  entries go through `dlog.sh` to guarantee accurate timestamps (never type a
  time). Section headers and prose in the dev log write directly.
- **One platform at a time.** The skill assumes a single active deployment per
  project repo — same as `/start-deployment`.
- **ADR-0013**: this skill writes no `progress.md` entry. It is a
  deployment-lifecycle skill (writes the deployment dev log and closes the
  deployment GitHub issue), not a development-pipeline step.

## References

- [ADR-0014](../../../docs/decisions/0014-deployment-mode.md) — decision record
- [ADR-0011](../../../docs/decisions/0011-field-mode-for-non-github-origins.md) — field mode (`field_mode.sh` is the side-detection authority)
- [ADR-0003](../../../docs/decisions/0003-workspace-infrastructure-is-project-agnostic.md) — three-tier content split (this skill is generic; all project specifics in `.agents/deployment.yaml`)
- [`.claude/skills/start-deployment/SKILL.md`](../start-deployment/SKILL.md) — sibling skill (phase [0])
- [`.agent/knowledge/deployment_mode.md`](../../../.agent/knowledge/deployment_mode.md) — operational reference (fuller treatment of contract, lifecycle, schema)
- [`.agent/templates/deployment_config.yaml`](../../../.agent/templates/deployment_config.yaml) — per-project config template
- [`.claude/skills/import-field-changes/SKILL.md`](../import-field-changes/SKILL.md) (skill: `/import-field-changes`) — per-repo field import PRs (step 8)
- [#435](https://github.com/rolker/ros2_agent_workspace/issues/435) — debrief / bag analysis checklist (out of scope here; tracked there)
- [#495](https://github.com/rolker/ros2_agent_workspace/issues/495) — umbrella for the full deployment lifecycle
- [#496](https://github.com/rolker/ros2_agent_workspace/issues/496) — recovery checklist (phase [2]; follow-up)
- [#530](https://github.com/rolker/ros2_agent_workspace/issues/530) — v1 implementation issue
