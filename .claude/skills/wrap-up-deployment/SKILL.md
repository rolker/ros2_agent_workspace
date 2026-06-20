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

- **No open deployment issue**: nothing to wrap up; confirm with the operator
  whether this was intentional.
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

Fetch the field remote so its branches are up to date:

```bash
git fetch gitcloud
```

(If `gitcloud` is not a configured remote for the project repo, ask the
operator for the correct field-remote name.)

Gather all log files for this deployment in `<log_dir>/<YYYY>/`:

```bash
ls <log_dir>/<YYYY>/<start-date>_*_logs.md
```

Read each file. List them for the operator: one line per file, showing the
host label extracted from the filename (`<start-date>_<label>_logs.md`).
Note any log whose label doesn't match a known host from
`deployment.yaml:hosts.field` — these may be mislabeled and will be raised
in the operator interview.

Also read the dev log (`<start-date>_dev_logs.md`) — it is the canonical sink
for wrap-up additions. Check that it exists; if not, create it with a minimal
header now (using `dlog.sh` for the timestamp):

```bash
LOGFILE="<log_dir>/<YYYY>/<start-date>_dev_logs.md"
<workspace_root>/.agent/scripts/dlog.sh "$LOGFILE" "wrap-up started"
```

### 5. Operator-correction interview

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

### 6. Consolidate the dev log

Append the following sections to the dev log file
(`<log_dir>/<YYYY>/<start-date>_dev_logs.md`). Use
`<workspace_root>/.agent/scripts/dlog.sh` for any entries that need a
timestamp; write Markdown section headers and body prose directly.

#### 6a. Apply operator corrections to field logs in place

For each correction from the interview that pertains to a specific field log
file (wrong label, misdated entry, incorrect root cause attributed to the
agent):

- **Mislabeled log file**: rename the file to the correct name (preserving the
  `<start-date>_<label>_logs.md` pattern with the corrected label).
- **Misdated entry**: edit the entry's timestamp in the file to the corrected
  value; add an inline note: `<!-- corrected at wrap-up: operator reported
  actual time was <HH:MM> -->`.
- **Incorrect conclusion**: add a correction block directly after the
  offending entry:

  ```
  > **Wrap-up correction**: [operator's correction verbatim or paraphrased]
  ```

Stage all corrected field log files for commit (`git add`); do NOT commit yet
(step 8 is where the commit lands).

#### 6b. Append structured summary to dev log

Append these four sections to the dev log (use `dlog.sh` only for
timestamped *events*; section headers and prose go in directly):

```markdown
## Summary

<2–4 sentences: what ran, what worked, what broke, overall outcome.
Written in past tense. No invented mechanisms; only what happened.>

## Operator Corrections

<Verbatim or close-to-verbatim record of corrections from step 5.
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
before this list drives issue-filing in step 9.>
```

#### 6c. Stamp the dev log link in the deployment issue

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

### 7. Reconcile field code

Two distinct mechanisms. Do both.

#### 7a. SHA-preserving merge — deployment repo's own field commits

The deployment repo (primary project repo in `packages[0]`) accumulates field
commits on `gitcloud/<default_branch>` during live ops. Bring these into the
deployment branch with a **merge commit** — NOT cherry-pick, NOT squash — to
preserve SHA lineage and make reconciliation to gitcloud a fast-forward after
the wrap-up PR merges:

```bash
git merge gitcloud/<default_branch>
```

Run from inside the deployment worktree (`feature/issue-<N>` on the deployment
repo). Resolve any conflicts with the operator; do not auto-resolve anything
that touches control logic.

After this merge, the wrap-up branch is ahead of both `origin/<default_branch>`
(via the PR) and `gitcloud/<default_branch>` (which will fast-forward after
merge). Record the merge commit SHA in the dev log via `dlog.sh`.

#### 7b. `/import-field-changes` — other repos

Other repos that received field commits (nav packages, sensors, interfaces,
etc.) use the `/import-field-changes` skill — that skill opens per-repo import
PRs with pre-review against the Quality Standard. `/wrap-up-deployment` does
NOT attempt to merge those repos directly; it delegates:

> Invoke `/import-field-changes` for each repo that has field commits on
> `gitcloud` but is NOT the primary deployment repo (`packages[0]`).

Ask the operator: "Which other repos received field commits during this
deployment?" Run `/import-field-changes` for each answer. Record the resulting
PR URLs in the dev log.

If no other repos had field commits, record that explicitly in the dev log:
"No other repos had field commits; `/import-field-changes` not needed."

### 8. PR → merge → gitcloud reconcile → remove worktree

**Open the wrap-up PR**:

```bash
BODY_FILE=$(mktemp /tmp/gh_body.XXXXXX.md)
# Write PR body to $BODY_FILE — include dev-log link, summary, operator-corrections summary
gh pr create \
    --title "Wrap up deployment #<N>: <scope>" \
    --body-file "$BODY_FILE" \
    --draft
rm "$BODY_FILE"
```

The PR body should include:
- Link to the deployment issue (will be closed by the merge).
- One-paragraph summary from the dev log's `## Summary` section.
- Link to each per-host log file and each import-field-changes PR.
- AI signature.

Add `Closes #<N>` in the PR body so GitHub closes the deployment issue on merge.

**Merge** — use a **merge commit** (not squash, not rebase):

```bash
gh pr merge <PR_number> --merge
```

Squash would discard the field-commit history folded in by step 7a; rebase
would break the SHA-preserving intent. The deployment issue closes automatically
via the `Closes #<N>` reference.

**Reconcile gitcloud** after the PR merges to `<default_branch>`:

```bash
git fetch origin
git push gitcloud origin/<default_branch>:<default_branch>
```

This fast-forwards `gitcloud/<default_branch>` to match `origin/<default_branch>`,
completing the SHA-preserving round-trip from step 7a.

If `issue_sync.dev_push` is configured, run it now to propagate the closed
issue status to the field-visible issue source:

```bash
<issue_sync.dev_push>
```

On failure, print `issue_sync.failure_hint`. This is advisory — the GitHub
issue is already closed; `dev_push` just keeps the field-side view in sync.

**Remove the worktree**:

```bash
<workspace_root>/.agent/scripts/worktree_remove.sh --issue <N>
```

### 9. File new RCA issues selectively

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
Include the AI signature and a link to the deployment issue.

**Close stale deployment issues** — if there are leftover open `deployment`-labeled
issues from prior deployments that were never wrapped up, list them for the
operator and ask whether to close them with a note. Do not close them unilaterally.

## Guidelines

- **This is a dev-side-only skill.** The very first step after reading config is
  checking for dev side. Field-side wrap-up is not possible — no `gh` access,
  no PR, no merges.
- **The operator-correction interview is the highest-value step.** Field agent logs
  are written under sterile-cockpit pressure and may contain misattributions,
  premature conclusions, or events that didn't happen as logged. This is the only
  opportunity to fix the permanent record before it propagates.
- **Two merge mechanisms, not one.** The SHA-preserving `git merge` (step 7a) is
  for the primary deployment repo. `/import-field-changes` (step 7b) is for
  everything else. Do not conflate them — cherry-pick or squash on the primary
  repo would break the gitcloud fast-forward.
- **Merge commit, not squash.** The PR merge MUST be `--merge`. Squash discards
  the field-commit SHAs folded in by step 7a, making gitcloud reconciliation a
  force-push rather than a fast-forward.
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
- [`.agent/scripts/import-field-changes`](../../../.agent/scripts/) (skill: `/import-field-changes`) — per-repo field import PRs (step 7b)
- [#435](https://github.com/rolker/ros2_agent_workspace/issues/435) — debrief / bag analysis checklist (out of scope here; tracked there)
- [#495](https://github.com/rolker/ros2_agent_workspace/issues/495) — umbrella for the full deployment lifecycle
- [#496](https://github.com/rolker/ros2_agent_workspace/issues/496) — recovery checklist (phase [2]; follow-up)
- [#530](https://github.com/rolker/ros2_agent_workspace/issues/530) — v1 implementation issue
