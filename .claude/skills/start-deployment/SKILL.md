---
name: start-deployment
description: Activate deployment mode for a live field deployment. Discovers the project's deployment config, detects dev/field side via field_mode.sh, and either creates a new deployment, first-activates an existing one (worktree/main-tree + per-host log + issue-sync push), or resumes an ongoing one. Loads the urgency contract (sterile-cockpit / mitigate-before-diagnose / time-box) into the current agent session.
---

# Start Deployment

## Usage

```
/start-deployment
```

No arguments. The skill discovers everything it needs from the current
working directory's project config plus `field_mode.sh` and the project
repo's filesystem. On **dev side**, it additionally uses `gh` (running
inside the project repo, so origin auto-detection picks the right
remote). On **field side**, there is no `gh` access — all issue
discovery and reads go through the configured `issue_sync` commands
instead.

## Overview

Activate **deployment mode** ([ADR-0014](../../../docs/decisions/0014-deployment-mode.md))
in the current agent session for a live field deployment of an autonomous
robot boat. Operationally: collapse the manual deployment-start steps
(open issue, create worktree, seed log, push to field share) into one
command, and put the urgency contract in this session's context so the
agent stays fast and operator-responsive during live ops.

**Lifecycle position**: [0] start (this skill) → [1] session → [2] recovery → [3] wrap-up → [4] next

**Activation is per-agent-session.** Other agent sessions on the same
host are NOT affected. No filesystem marker, no host-wide hook. On-disk
evidence of an ongoing deployment is the open `deployment`-labeled issue
plus the worktree (dev) or host log file (field) that already exist
naturally — see the three-state detection in step 4.

The fuller operational reference is
[`.agent/knowledge/deployment_mode.md`](../../../.agent/knowledge/deployment_mode.md).
If this SKILL.md and the knowledge doc drift on the urgency-contract
wording, **this file wins** — it's what's in context when the skill runs.

## The Urgency Contract

You are now operating under the deployment-mode urgency contract. These
rules apply for the rest of this agent session (until the operator
explicitly invokes the wrap-up flow or activates a different mode).

1. **You are an assistant, not an investigator.** During live ops the
   operator is running the boat; your job is to make the operator
   faster, not to pursue an investigation independently.
2. **Mitigate before diagnose.** Stabilize the symptom now — turn down a
   rate, swap a topic, restart a node — and defer root-cause analysis to
   wrap-up. A live deployment is not a debugging session.
3. **Time-box / confidence handoff.** If a path is taking more than ~1–2
   minutes or your confidence is low, stop, summarize what you've
   checked and what you'd try next, and hand back to the operator.
   Don't spiral.
4. **Append, don't commit.** Batch commits at natural breakpoints
   (between runs, at recovery). Each commit is a permission prompt and
   a context switch; live ops absorb neither well.
5. **Overlog rather than underlog.** Appending to a log file is
   near-zero cost. Commits are what cause the prompts. Capture
   observations liberally; sort them out at wrap-up.
6. **Avoid permission prompts.** Use `git -C <path>`, absolute paths,
   `gh -R`, and the file tools over heredocs / find / cat. The
   allowlist defeats itself when commands are wrapped in `cd && …`.
   **Timestamps: invoke `date '+%Y-%m-%d %H:%M %:z'` for every
   timestamped entry — never type a time that "looks right," even when
   typing feels faster under pressure.** A typed time is a durable lie
   the wrap-up integration and downstream analysis will trust. For an
   event the operator reports after the fact, call `date` to record
   *when you're logging it* and write the operator's time separately as
   `~HH:MM (operator-reported)`. Use `date '+…'`, never `date | sed`;
   reach for the `dlog` helper below so the timestamp is never optional.
7. **Sterile cockpit.** During live on-water ops, do *only*
   operation-essential work — no doc polish, refactors, or "while we're
   here" cleanups. Write them down for wrap-up.
8. **Tides / weather / currents are dev-only.** Field hosts skip
   pre-flight data entirely. Dev sessions ingest them (after asking the
   operator) and log them in the dev-side log's per-day `## Pre-flight`
   section.
9. **Suggestions are conversation, not log entries.** "What if we tried
   X?" stays in chat unless the operator decides to act on it. The log
   records what *happened*, not what was proposed.
10. **Operator notes are quotes plus factual context only.** No invented
    mechanisms. If the operator says "felt sluggish," log the quote and
    the nearest objective measurement; don't extrapolate to "thrust
    loss."
11. **No self-imposed hard gates.** Don't write "⛔ Mandatory" /
    "sim-verify required before…" into issues or PRs during live ops.
    Risk-acceptance gates are the operator's call.

The contract is **phase-aware**. Sterile-cockpit bites hardest during
live ops [1] and **relaxes at wrap-up [3]**, where deep analysis is the
expected work.

**Invariants the contract does NOT relax**: safety-critical correctness
(boat control, collision avoidance), pre-commit hooks, AI signature,
atomic commits, no secrets, no destructive ops without operator
approval. *Defer is not skip* — deferred RCA / tests / docs go on the
wrap-up checklist, follow-up issues, or the roadmap.

## Steps

### 1. Read the project config

The skill runs against a project repo. Locate `<project_repo>/.agents/deployment.yaml`:

- If the current working directory is inside a project repo, the config
  is at `<repo_root>/.agents/deployment.yaml`.
- If the current working directory is the workspace root, ask the
  operator which platform's deployment is starting and look up the
  project repo from the user response (only one platform is typically
  active at a time).

If the config file is missing, stop with:

> `<project_repo>/.agents/deployment.yaml` not found. See
> `.agent/templates/deployment_config.yaml` for the template, and
> `.agent/knowledge/deployment_mode.md` for the schema.

Read the config and extract: `platform`, `default_branch`, `layer`,
`packages`, `labels`, `log_dir`, `issue_sync` (optional), `tides` /
`currents` / `weather` (optional).

**Validation**: if any required value is the literal string `TODO`
(or empty / null where required), stop with:

> `<key>` is unconfigured in `<project_repo>/.agents/deployment.yaml` —
> fill in before running `/start-deployment`. See
> `.agent/templates/deployment_config.yaml` for the schema.

Required keys: `platform`, `default_branch`, `layer`, `packages[0]`,
`log_dir`. (`issue_sync` and the pre-flight sections are optional;
absent values disable their respective steps.) This is a hard stop —
half-filled configs cause silent failures mid-procedure (the skill
would otherwise build paths like `layers/main/TODO_ws/src/TODO/`).

### 2. Detect side (dev vs field)

Run `field_mode.sh` against the project repo's path. The script lives at
the workspace root and is not on `PATH` — invoke by explicit path.

**Discovering the workspace root**: walk up from `$PWD` until a directory
is found that contains `.agent/scripts/setup.bash` (the canonical marker
for the workspace root). For the standard layer structure, a project
repo at `layers/main/<layer>_ws/src/<repo>/` is exactly 5 levels below
the workspace root (`../../../../../`), but don't hardcode — projects
may reorganize.

Then pick whichever form matches the current working directory:

```bash
# Form A — when CWD is the workspace root, pass the repo path:
.agent/scripts/field_mode.sh --describe layers/main/<layer>_ws/src/<packages[0]>

# Form B — when CWD is already inside the project repo, reference the
# script via the discovered workspace root and pass no path (defaults
# to $PWD):
<workspace_root>/.agent/scripts/field_mode.sh --describe
```

The repo identity comes from the origin URL either way; only the
relative path plumbing differs.

- **Dev side**: origin on the GitHub allowlist → worktree + draft PR flow.
- **Field side**: origin not on the GitHub allowlist → direct-commit to
  default branch per [ADR-0011](../../../docs/decisions/0011-field-mode-for-non-github-origins.md).

Record the side; it branches several steps below.

### 3. Determine the per-host log label

- **Dev side**: label is the literal string `dev` (regardless of
  hostname), so wrap-up tooling can reliably find the dev log.
- **Field side**: label is `$(hostname -s)` (short hostname).

Log filename is `<log_dir>/<YYYY>/<YYYY-MM-DD>_<label>_logs.md`. The
date is the deployment **start** date, NOT today's date if resuming a
multi-day deployment — step 4 picks the right date from the existing
deployment issue.

### 4. Three-state detection

Probe for an open `deployment`-labeled issue in the project repo, then
for local artifacts that indicate first-activation vs resume.

**Issue lookup** depends on side. **All dev-side `gh` calls in this and
subsequent steps run from inside the project repo** (main tree before
step 4b's worktree creation, worktree after); `gh` auto-detects the
correct remote from `origin`, so no `-R <owner/repo>` flag is needed.

- **Dev side**: `gh issue list --label deployment --state open --json number,title,createdAt,body`. `issue_sync` is fully
  optional here — `gh` alone handles discovery, listing, and editing.
- **Field side**: run the configured `issue_sync.field_pull` first
  (refreshes the local view), then `issue_sync.field_list_open` (lists
  open deployment issues), and `issue_sync.field_show` (used later in
  step 4b to read each issue's body). These three commands are
  **hard-required on field side** — without them the skill has no way
  to discover or read deployment issues (there is no `gh` fallback).
  If any of `issue_sync.field_pull` / `field_list_open` / `field_show`
  is missing from `.agents/deployment.yaml`, stop with:
  > Field-side `/start-deployment` requires `issue_sync.field_pull`,
  > `field_list_open`, and `field_show` in `.agents/deployment.yaml`
  > (no `gh` access on field hosts). Configure them — see
  > `.agent/templates/deployment_config.yaml` for examples — then
  > re-invoke.

Then branch on state:

| State | Evidence | Action (this step) |
|---|---|---|
| **Create new** | No open deployment issue | Step 4a |
| **Activate first time** | Issue exists, no local worktree (dev) / no host log (field) | Step 4b |
| **Resume ongoing** | Issue exists + worktree+log (dev) / log file (field) | Step 4c |

If multiple open deployment issues are found, list them and ask the
operator which one this session is joining.

#### 4a. Create new

**Field side**: cannot create the issue (no GitHub access). Stop with:

> No open deployment issue found via `issue_sync`. Start the deployment
> from a dev host first (where `gh` can create the issue and configure
> labels), then run `issue_sync.dev_push` so this field host sees it on
> the next `issue_sync.field_pull`.

**Dev side**: ask the operator for the scope of this deployment (one or
two sentences; this becomes the issue title's `<scope>` portion and the
body's first paragraph). Confirm before creating.

Open the deployment issue with `gh issue create` (run from inside the
project repo's main tree — `gh` auto-detects the remote):

- **Title**: `Deployment <YYYY-MM-DD>: <scope>` (today's date).
- **Labels**: from `labels` in the project config.
- **Body**: minimal template — `## Purpose` (scope sentence), `## Hosts
  in use` (placeholder), `## Logs` (empty section the skill will stamp
  links into), AI signature.

Then proceed to step 4b's first-activation flow against the just-created
issue.

#### 4b. Activate first time

This step has two branches — dev-side edits the canonical issue,
field-side reads it and warns on drift. The locked design's intended
order is **dev-first-then-field**: the operator runs `/start-deployment`
on dev first (where rename / body edits happen and `dev_push`
propagates them), then on each field host as those come online.

**On both sides — read the issue body** to drive the state checks below.

- Dev: `gh issue view <N> --json title,body,createdAt`
- Field: `issue_sync.field_show` with `{id}` substituted for `<N>`.

**Create the worktree / prepare main-tree** — branch on side. This is
done *before* the title/body checks so the per-host log (initialized
next) exists by the time any warnings need to land in it.

- **Dev side**: create the worktree, using the workspace-root prefix
  discovered in step 2 (the skill may be running from inside the
  project repo, where `.agent/scripts/` does not exist):
  ```bash
  <workspace_root>/.agent/scripts/worktree_create.sh \
      --issue <N> \
      --type layer \
      --layer <layer> \
      --packages <packages,comma-separated>
  ```
  Then `source <workspace_root>/.agent/scripts/worktree_enter.sh --issue <N>`.
- **Field side**: ensure the project repo is on its default branch
  (`<default_branch>` from the config) and the tree is clean. Do NOT
  create a worktree (per [ADR-0011](../../../docs/decisions/0011-field-mode-for-non-github-origins.md),
  field-mode repos edit and commit on the default branch).

**Initialize the per-host log file**:

```bash
<project_repo>/<log_dir>/<YYYY>/<YYYY-MM-DD>_<label>_logs.md
```

(Create the `<YYYY>` parent dir if missing.) Seed with a minimal header:

```markdown
# <YYYY-MM-DD> — <label> log (<platform> deployment #<N>)

Deployment issue: <issue URL>
Host: <hostname>
Side: <dev|field>
Started: <YYYY-MM-DD HH:MM ±HH:MM>
```

Any title / body / log-stamp warnings emitted below append to this
file. It is the canonical sink for field-side drift warnings; dev side
also appends here for parity.

**Append entries with the committed `dlog` helper** —
`<workspace_root>/.agent/scripts/dlog.sh`. It bakes the timestamp from
`date` at write time (never typed, never approval-delayed) and appends
one entry, so a measured timestamp is the path of least resistance
(rule 6 — never type times by hand):

```bash
LOGFILE="<project_repo>/<log_dir>/<YYYY>/<YYYY-MM-DD>_<label>_logs.md"
<workspace_root>/.agent/scripts/dlog.sh "$LOGFILE" "charger disconnected; controls check good; going to launch"
```

`$LOGFILE` does not survive across the agent's separate Bash invocations
(fresh subshell each call), so pass the **literal log-file path** to
`dlog.sh` every time — the path is stable for the deployment.

**Allowlist it once** so live-ops logging is prompt-free: add
`Bash(<workspace_root>/.agent/scripts/dlog.sh:*)` to
`.claude/settings.local.json` (one generic entry, reused across every
deployment — *not* a per-deployment script). **Do not substitute the Edit
tool for logging:** it avoids the prompt but cannot run `date`, so it
forces a *typed* timestamp — the inaccurate, approval-skewed "durable
lie" times #515 exists to prevent. `dlog.sh` is the only method that is
both prompt-free *and* accurately stamped.

For an event the operator reports after the fact — whose time you did
**not** measure — let `dlog.sh` stamp when you're recording it and put the
operator's time in the text, marked approximate:
`dlog.sh "$LOGFILE" "in water (operator-reported ~12:35)"`. Never
back-fill the stamp to the operator's time; the stamp is when *you* logged.

**Verify the issue title** (form: `Deployment <YYYY-MM-DD>: <scope>`):

- **Dev side**: if not already in that form, ask the operator to
  confirm the deployment-start date (default: the issue's `createdAt`
  date in the operator's TZ), then update the title via
  `gh issue edit <N> --title "..."`.
- **Field side**: if not already in that form, append a warning to the
  per-host log file (initialized above): *"Deployment issue title not
  in canonical form; run `/start-deployment` from dev first to rename
  + push."* Do not edit on field side — `issue_sync` has no edit verb,
  and edits would diverge from dev's view.

**Ensure essentials on the body** (only the `## Logs` section is
managed; everything else is operator-authored and untouched):

- **Dev side**: append a `## Logs` section if absent.
- **Field side**: read-only — if `## Logs` is absent, append the same
  "rename + push from dev" warning to the per-host log file; do not
  edit the issue.
- If the body is suspiciously empty on either side (no operator-authored
  sections), warn but do NOT impose structure.

**Stamp the log file link in the issue body's `## Logs` section** —
branch on side:

- **Dev side**: edit the issue body via `gh issue edit <N> --body-file <tmpfile>`
  to add a line under `## Logs`:
  `- [<label> log](<log_dir>/<YYYY>/<YYYY-MM-DD>_<label>_logs.md)`.
- **Field side**: do not edit. Log the new log file's path to the host
  log itself with a note: *"Stamp this link under the deployment
  issue's `## Logs` section from dev next time `/start-deployment`
  runs there."* Dev's next invocation reads the field log directory
  and can add missing entries.

**Run `issue_sync.dev_push` (dev side only, if configured)** to
propagate the title / body changes to the field-visible source. Skip
silently on field side (the field host is the consumer of the share,
not a producer). On failure, print `issue_sync.failure_hint`.

Proceed to step 5 (dev pre-flight) if dev side; otherwise the skill is
done.

#### 4c. Resume ongoing

Re-attach this session to the existing deployment. No artifacts created.

- Print the deployment issue's URL and title (dev: `gh issue view <N> --json title,url`;
  field: `issue_sync.field_show` with `{id}` substituted for `<N>`).
- Print the current host's log file path.
- Read the last ~20 entries of the current host's log and summarize them
  in chat so the operator can confirm context.
- The urgency contract above is already loaded. Confirm to the operator
  that this session is now in deployment mode.

Skip step 5 (no new pre-flight on resume).

### 5. Dev pre-flight (dev side only, first-activation only)

Ask the operator: *"Pulling pre-flight tides / currents / weather for
today's session — proceed, or skip?"*

- If the operator wants to enter them manually, skip the fetches; they
  log it themselves.
- If the operator defers ("you do it"), fetch using the defaults in the
  config's `tides` / `currents` / `weather` sections.

Log results in the dev log under a per-day `## Pre-flight` section
(today's date as the day header inside the file):

- **Tide heights in metres.**
- **Tide times with explicit TZ in both local and UTC**, e.g.
  `L 04:23 EDT (08:23 UTC)`.
- **Currents and winds in conventional marine units** (knots, nm).
- **Temperatures as the source gives them** — don't convert.
- **Preserve source-station identifiers** (NOAA / NDBC) so the data is
  reproducible.

The pre-flight section goes in the **dev log**, never the issue body.

## Guidelines

- **The urgency contract is the load-bearing part of this skill.** The
  artifact creation matters, but the contract is what changes how the
  rest of the session behaves. Re-read it if you find yourself
  spiraling.
- **One platform at a time.** The skill assumes a single active
  deployment per project repo. If a project ever needs concurrent
  multi-deployment, that's a redesign, not a parameter.
- **Don't impose structure on the operator's issue body.** Ensure the
  `## Logs` section exists; leave everything else alone.
- **Field-side has no GitHub.** Every GitHub call (`gh issue …`,
  `gh pr …`) is gated on dev-side detection. Field-side uses
  `issue_sync` commands instead.
- **Multi-day deployments keep the original date.** The filename's date
  is the deployment **start**, not the date of any given session. Per-day
  sections inside the log use `## <YYYY-MM-DD>` headers.
- **Wrap-up is a separate skill.** Don't try to close the deployment
  issue from here. `/wrap-up-deployment` (tracked under [#495](https://github.com/rolker/ros2_agent_workspace/issues/495))
  owns the wrap-up procedure.

## References

- [ADR-0014](../../../docs/decisions/0014-deployment-mode.md) — decision record
- [ADR-0011](../../../docs/decisions/0011-field-mode-for-non-github-origins.md) — sibling mode (`field_mode.sh` is the side-detection authority)
- [ADR-0003](../../../docs/decisions/0003-workspace-infrastructure-is-project-agnostic.md) — three-tier content split (this skill is generic; all project specifics in `.agents/deployment.yaml`)
- [`.agent/knowledge/deployment_mode.md`](../../../.agent/knowledge/deployment_mode.md) — operational reference (fuller treatment of contract, lifecycle, schema)
- [`.agent/templates/deployment_config.yaml`](../../../.agent/templates/deployment_config.yaml) — per-project config template
- [#495](https://github.com/rolker/ros2_agent_workspace/issues/495) — umbrella for the full deployment lifecycle (wrap-up / next-deployment / recovery skills tracked there)
- [#501](https://github.com/rolker/ros2_agent_workspace/issues/501) — v1 implementation
