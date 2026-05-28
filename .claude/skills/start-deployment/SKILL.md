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
working directory's project config plus `gh` / `field_mode.sh` / the
project repo's filesystem.

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
   Timestamps: `date '+%Y-%m-%d %H:%M %:z'`, never `date | sed`.
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

### 2. Detect side (dev vs field)

Run `field_mode.sh` against the project repo's path:

```bash
.agent/scripts/field_mode.sh --describe layers/main/<layer>_ws/src/<packages[0]>
```

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

**Issue lookup** depends on side:

- **Dev side**: `gh -R <owner/repo> issue list --label deployment --state open --json number,title,createdAt,body`
- **Field side**: run the configured `issue_sync.field_pull` first,
  then `issue_sync.field_list_open`. If `issue_sync` is absent, skip
  the sync step and warn:
  > No `issue_sync` configured in `.agents/deployment.yaml`. The skill
  > can't refresh the field share's view of issues. If this host
  > already has a recent copy, the local view will be used; otherwise,
  > ensure all hosts see the deployment issue by other means.

Then branch on state:

| State | Evidence | Action (this step) |
|---|---|---|
| **Create new** | No open deployment issue | Step 4a |
| **Activate first time** | Issue exists, no local worktree (dev) / no host log (field) | Step 4b |
| **Resume ongoing** | Issue exists + worktree+log (dev) / log file (field) | Step 4c |

If multiple open deployment issues are found, list them and ask the
operator which one this session is joining.

#### 4a. Create new

Ask the operator for the scope of this deployment (one or two sentences;
this becomes the issue title's `<scope>` portion and the body's first
paragraph). Confirm before creating.

Open the deployment issue with `gh -R <owner/repo> issue create`:

- **Title**: `Deployment <YYYY-MM-DD>: <scope>` (today's date).
- **Labels**: from `labels` in the project config.
- **Body**: minimal template — `## Purpose` (scope sentence), `## Hosts
  in use` (placeholder), `## Logs` (empty section the skill will stamp
  links into), AI signature.

Then proceed to step 4b's first-activation flow against the just-created
issue.

#### 4b. Activate first time

**Verify the issue title**: if it's not already in
`Deployment <YYYY-MM-DD>: <scope>` format, ask the operator to confirm
the deployment-start date (default: the issue's `createdAt` date in the
operator's TZ), then update the title via `gh issue edit`.

**Ensure essentials on the body** — only:

- Append a `## Logs` section if absent (the skill will stamp per-host
  log file links into this section as they appear).
- If the body is suspiciously empty (no operator-authored sections),
  warn but do NOT impose structure. Operator-authored sections
  (Purpose, Must-verify, Operators, Hosts in use, Notes) are left
  untouched.

**Create the worktree (dev side only)**:

```bash
.agent/scripts/worktree_create.sh \
    --issue <N> \
    --type layer \
    --layer <layer> \
    --packages <packages,comma-separated>
```

Then `source .agent/scripts/worktree_enter.sh --issue <N>`.

Field side: ensure the project repo is on its default branch
(`<default_branch>` from the config) and the tree is clean. Do NOT
create a worktree.

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

Stamp a link to this log file in the issue body's `## Logs` section
(`<log_dir>/<YYYY>/<YYYY-MM-DD>_<label>_logs.md` relative to the project
repo).

**Run `issue_sync.dev_push` (dev side, if configured)** to propagate
the issue title / body / label changes to the field-visible source.
On failure, print `issue_sync.failure_hint`.

Proceed to step 5 (dev pre-flight) if dev side; otherwise the skill is
done.

#### 4c. Resume ongoing

Re-attach this session to the existing deployment. No artifacts created.

- Print the deployment issue's URL and title.
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
