# Deployment Mode — Operational Reference

Canonical reference for *deployment mode*: how to activate it, what the urgency
contract means in practice, the lifecycle phases, log-naming conventions, what
to write down and what to skip, and the per-project config schema that drives
the [`start-deployment`](../../.claude/skills/start-deployment/SKILL.md) skill.

See [ADR-0014](../../docs/decisions/0014-deployment-mode.md) for the decision
record. SKILL.md embeds the urgency contract verbatim; if this doc and SKILL.md
ever drift on the contract wording, **SKILL.md wins** (it's what's in context
when the skill runs).

## What it is

Deployment mode is a *behavioral* operating mode the agent enters for a live
field deployment of an autonomous robot boat. The decision record is in
[ADR-0014](../../docs/decisions/0014-deployment-mode.md); this doc is the
operational how-to.

Two halves, both load-bearing:

1. **An urgency contract** — sterile-cockpit-style behavioral adjustments that
   keep the agent fast and operator-responsive during live ops.
2. **Lifecycle tooling** — the skills and templates that run inside the mode
   (`/start-deployment` is the first; `/wrap-up-deployment` and
   `/next-deployment` are tracked as follow-ups under the umbrella
   [#495](https://github.com/rolker/ros2_agent_workspace/issues/495)).

It's a sibling to **field mode** ([ADR-0011](../../docs/decisions/0011-field-mode-for-non-github-origins.md)):
field mode is detected by origin URL and scopes *where commits go*; deployment
mode is operator-set per agent session and scopes *how the agent behaves under
live time pressure*. The two axes are orthogonal; they overlap on field hosts
during a deployment.

## Activation

**Per-agent-session.** Invoking the `/start-deployment` skill puts the current
agent session in deployment mode. Other agent sessions on the same host doing
unrelated work are **not** affected — the operator has to invoke
`/start-deployment` in each session that should run under the contract.

There is **no filesystem marker** and **no host-wide hook**. On-disk evidence
of an ongoing deployment is what already exists naturally: an open
`deployment`-labeled issue, plus a worktree (dev side) or a host log file
(field side). The skill's three-state detection (see below) uses those
artifacts to decide whether to create a new deployment, first-activate an
existing one, or resume an ongoing one.

## The urgency contract

Eleven rules. They are embedded verbatim in
[`SKILL.md`](../../.claude/skills/start-deployment/SKILL.md) so that invoking
the skill puts the contract in context. The fuller treatment below explains
*why* and *when* each one bites.

1. **You are an assistant, not an investigator.** During live ops the operator
   is running the boat; the agent's job is to make the operator faster, not to
   pursue an investigation independently.
2. **Mitigate before diagnose.** Stabilize the symptom now — turn down a rate,
   swap a topic, restart a node — and defer root-cause analysis to wrap-up. A
   live deployment is not a debugging session.
3. **Time-box / confidence handoff.** If a path is taking more than ~1–2
   minutes or your confidence is low, **stop**, summarize what you've checked
   and what you'd try next, and hand back to the operator. Don't spiral.
4. **Append, don't commit.** Batch commits at natural breakpoints (between
   runs, at recovery). Each commit is a permission prompt and a context
   switch; live ops absorb neither well.
5. **Overlog rather than underlog.** Appending to a log file is near-zero cost.
   Commits are what cause the prompts. Capture observations liberally; sort
   them out at wrap-up.
6. **Avoid permission prompts.** Use `git -C <path>`, absolute paths, `gh -R`,
   and the file tools over heredocs / find / cat. The allowlist defeats
   itself when commands are wrapped in `cd && …`. **Generate every timestamp
   by invoking `date '+%Y-%m-%d %H:%M %:z'` — never type a plausible-looking
   time, even when typing feels faster under pressure.** A typed time is a
   durable lie the wrap-up integration and downstream analysis will trust;
   for an operator-reported event, `date`-stamp when you log it and mark
   their time `~HH:MM (operator-reported)`. (And `date '+…'`, not
   `date | sed`.) Use the committed `.agent/scripts/dlog.sh <logfile>
   <message>` helper — it bakes `date` into each entry and, allowlisted
   once (`Bash(.../dlog.sh:*)`), makes prompt-free logging the easy path.
   The Edit tool is **not** a substitute: it dodges the prompt but cannot
   run `date`, forcing a typed (inaccurate) time — the very failure this
   rule prevents (see #515 / #516).
7. **Sterile cockpit.** During live on-water ops, do *only* operation-essential
   work — no doc polish, refactors, or "while we're here" cleanups. Write
   them down for wrap-up.
8. **Tides / weather / currents are dev-only.** Field hosts skip pre-flight
   data entirely. The dev session ingests them (after asking the operator) and
   logs them in the dev-side log's per-day `## Pre-flight` section.
9. **Suggestions are conversation, not log entries.** "What if we tried X?"
   stays in chat unless the operator decides to act on it. The log records
   what *happened*, not what was proposed.
10. **Operator notes are quotes plus factual context only.** No invented
    mechanisms. If the operator says "felt sluggish," log the quote and the
    nearest objective measurement; don't extrapolate to "thrust loss."
11. **No self-imposed hard gates.** Don't write "⛔ Mandatory" / "sim-verify
    required before…" into issues or PRs during live ops. Risk-acceptance
    gates are the operator's call.

The contract is **phase-aware** — sterile-cockpit bites hardest during live
ops (phase [1] below) and **relaxes in wrap-up (phase [3])**, where deep
analysis is the *expected* work.

### Invariants (what the contract does NOT relax)

- **Safety-critical correctness** (boat control, collision avoidance) is
  never traded for speed.
- Pre-commit hooks, AI signature, atomic commits, no secrets, no destructive
  ops without explicit operator approval — same as field mode.
- **Defer is not skip.** Deferred RCA / tests / docs are *tracked* — as
  follow-up issues, roadmap items, or wrap-up checkboxes — not dropped. The
  Quality Standard's "never table when the permanent solve is near" still
  holds; deployment mode only defers *within a live session*, and wrap-up is
  where completeness is restored.
- **Human control.** The operator is continuously in the loop, which is
  precisely why deployment mode can be more responsive — real-time oversight
  is higher, not lower.

## Lifecycle phases

```
[0] Start of day — pre-deployment
    Activate /start-deployment; deployment issue templated;
    worktree (dev) or main-tree (field); dev log seeded with
    pre-flight tides / weather / currents.

[1] Deployment session — field execution
    Sterile cockpit bites. Per-host logs append-mostly. New
    bugs / observations filed as follow-up issues. In-deployment
    fixes commit to gitcloud during the session.

[2] Recovery — boat-side, pre-shutdown
    Safety-critical checklist: field agent logs pushed to
    gitcloud; bag data offloaded; screenshots / supplemental
    offloaded; stack cleanly shut down.

[3] Wrap-up — dev-side
    Contract relaxes. Import field changes; review-loop import
    PRs; consolidate logs; extract bag DB; run standard
    analyses; file analysis sub-issues; merge wrap-up PR
    (closes deployment issue).

[4] Next deployment-issue drafted from analysis outputs.
```

The phases map onto the SRE Prepare → Detect → Respond → Recover → Learn loop.
Phases [0]–[2] are under the urgency contract; [3] relaxes it; [4] is
deployment-mode-off (planning, not live).

## Log-naming convention

Per-host, per-deployment files in the project repo's logs directory
(`docs/logs/<YYYY>/` for `unh_echoboats_project11`; other projects configure
`log_dir`):

```
<YYYY-MM-DD>_<label>_logs.md
```

- **`<YYYY-MM-DD>`** is the deployment **start date**, not the date the file
  is being appended to. Multi-day deployments keep the same date on the
  filename; per-day sections inside the file use `## <YYYY-MM-DD>` headers.
- **`<label>`** is:
  - `dev` for dev-side sessions (regardless of host), so that wrap-up agents
    can reliably find the dev log.
  - `hostname -s` (short hostname) for field-side sessions — `gabby`,
    `salmon`, `mercat`, etc.
- The host is determined by `field_mode.sh` against the project repo's
  origin, not by where the physical hardware lives. A dev workstation
  working in a gitcloud-origin clone is in field mode for that repo, and
  uses its short hostname accordingly.

## What to write

The principle is **overlog, but write what happens, not what was proposed**.

Append to the dev log (during phase [3], or dev-side pre-flight in phase [0]):

- Per-day `## Pre-flight` section with tides, weather, currents (see format
  below).
- Operator decisions and their rationale, in quote form where possible.
- What was tried, what worked, what didn't.
- Bag paths, run identifiers, screenshot paths.
- Cross-references to follow-up issues filed during the session.

Append to the field log (during phase [1]):

- Run identifiers and rough timestamps.
- Symptoms observed, mitigations applied, results.
- Hardware events (power cycles, sensor swaps, link drops).
- Quotes from operator chatter that influenced decisions.

What stays in chat (does **not** go in either log):

- Speculation about causes that wasn't acted on.
- "We should consider X for next time" — that's a follow-up issue, not a log
  entry, if it's worth keeping.
- Long debugging traces that resolved in-session — summarize the outcome.

### Tides / weather / currents format

Dev-only. Asked first, fetched only on operator deferral. Logged in the dev
log's `## Pre-flight` section for that day:

- **Tide heights in metres** (matching local hydrographic convention).
- **Tide times with explicit TZ in both local and UTC**, e.g.
  `L 04:23 EDT (08:23 UTC)`. Never assume the reader's TZ matches the
  operator's.
- **Currents and winds in conventional marine units** (knots; nautical miles).
- **Temperatures as the source gives them.** Don't convert.
- **Source-station identifiers preserved** — NOAA station IDs, NDBC buoy
  numbers — so the data is reproducible.

Default station(s) live in the project's `.agents/deployment.yaml` under
`tides:` / `currents:` / `weather:` so the skill can fetch without asking the
operator every time.

## Three-state detection

The skill detects which of three states the current invocation is in, based on
on-disk artifacts that already exist:

| State | Evidence | Skill action |
|---|---|---|
| **Create new** | No open `deployment`-labeled issue (per `issue_sync`) | Offer to create; operator approves scope; templated issue body. |
| **Activate first time** | Open issue exists, but no local worktree (dev) / no host log (field) | Rename issue title to `Deployment YYYY-MM-DD: <scope>` if not already in that format; ensure `## Logs` section present in body; create worktree (dev) or prepare main-tree (field); initialize host log; run `issue_sync.dev_push` so field hosts see the changes. |
| **Resume ongoing** | Open issue + worktree+log (dev) / log file (field) | Re-attach: load the urgency contract into context, summarize state from the deployment issue body and the last ~20 entries of the current host's log. No artifacts created. |

"Ensure essentials" on the deployment issue body is limited to:

- Rename the title (only if it's not already in `Deployment YYYY-MM-DD: <scope>`
  form).
- Append a `## Logs` section if absent (the skill stamps per-host log file
  links here as they appear).
- Operator-authored sections (Purpose, Must-verify, Operators, Hosts in use,
  Notes) are left untouched.
- No automatic wrap-up checklist (the future `/wrap-up-deployment` skill owns
  that procedure).
- If the body is suspiciously empty, warn — don't impose structure.

## Project-config schema (`.agents/deployment.yaml`)

Each adopting project ships a small per-project config that the skill reads at
invocation time. Template lives at
[`.agent/templates/deployment_config.yaml`](../templates/deployment_config.yaml);
copy it to `<project_repo>/.agents/deployment.yaml` and fill in.

```yaml
platform: <short name>            # e.g. "BizzyBoat"
default_branch: <branch>          # e.g. "jazzy" — repo's default branch
layer: <layer>                    # e.g. "platforms" — workspace layer
packages: [<repo1>, …]            # project repos to include in worktrees
labels: [deployment, …]           # labels applied to the deployment issue
log_dir: <path>                   # e.g. "docs/logs" — relative to project repo

hosts:
  dev: <list of dev hostnames>    # informational
  field: <list of field hostnames>  # informational; field_mode.sh is authoritative

# Pluggable issue-sync mechanism (no git-bug-shaped default — see ADR-0003).
# - Dev side: issue_sync is fully optional (gh handles discovery, listing,
#   editing). If absent, the skill skips dev_push and continues.
# - Field side: field_pull, field_list_open, and field_show are
#   HARD-REQUIRED — without them the skill has no way to discover or
#   read deployment issues on field hosts (no gh fallback). If any of
#   the three is missing on field side, the skill stops with an explicit
#   error directing the operator to configure them.
issue_sync:
  field_pull: <command>           # field: refresh issues from share
  field_list_open: <command>      # field: list open deployment issues
  field_show: <command>           # field: show issue body — uses {id}
  dev_push: <command>             # dev: propagate changes to field-visible source
  failure_hint: <message>         # actionable error printed on failure

# Default data sources for pre-flight (all optional; dev-only).
tides:
  station: <NOAA station ID>      # heights always reported in metres
currents:
  station: <NOAA station ID>      # e.g. "ACT0731" for Clark Island
weather:
  source: <source>                # e.g. NWS office identifier
```

## Side detection (dev vs field)

`field_mode.sh` against the **project repo's origin URL** drives the dev /
field branch in the skill:

- **Dev-mode** project repo (origin on the GitHub allowlist) → worktree +
  draft PR per the existing convention.
- **Field-mode** project repo (origin not on the GitHub allowlist) → direct
  commit to the default branch per
  [ADR-0011](../../docs/decisions/0011-field-mode-for-non-github-origins.md),
  no PR.

If field hosts ever get GitHub access *and* the project repo's origin moves
to GitHub, the skill auto-switches without any code change.

## What's NOT in deployment mode v1

Tracked separately under [#495](https://github.com/rolker/ros2_agent_workspace/issues/495):

- `/wrap-up-deployment` — phase [3] orchestration (review-loop, log
  integration, analysis kickoff).
- `/next-deployment` — phase [4] drafting of the next deployment issue from
  analysis outputs.
- Framework-level hook for auto-injecting the urgency contract into every
  turn (rejected for v1 — would over-broadcast across unrelated sessions).
- Recovery checklist skill ([#496](https://github.com/rolker/ros2_agent_workspace/issues/496))
  — phase [2] safety-critical checks.
- A general "agent operating modes" framework ADR — deferred per ADR-0014
  until ≥3 mode instances are formalized.

## References

- [ADR-0014](../../docs/decisions/0014-deployment-mode.md) — decision record
- [ADR-0011](../../docs/decisions/0011-field-mode-for-non-github-origins.md) — sibling mode (where commits go)
- [ADR-0003](../../docs/decisions/0003-workspace-infrastructure-is-project-agnostic.md) — three-tier content split
- [`.claude/skills/start-deployment/SKILL.md`](../../.claude/skills/start-deployment/SKILL.md) — the skill
- [`.agent/templates/deployment_config.yaml`](../templates/deployment_config.yaml) — sample project config
- Workspace research digest entry "Operational-Assistant Behavior Under Live Time Pressure" — grounding for the urgency contract
- [#495](https://github.com/rolker/ros2_agent_workspace/issues/495) — umbrella for the full lifecycle
- [#501](https://github.com/rolker/ros2_agent_workspace/issues/501) — v1 skill issue
