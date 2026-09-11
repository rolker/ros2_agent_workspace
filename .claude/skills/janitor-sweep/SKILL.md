---
name: janitor-sweep
description: Run the workspace's staleness and drift detectors in one pass and write the result to a single local report file. Report-only — opens no PRs, files no issues, and publishes nothing.
---

# Janitor Sweep

## Usage

```
/janitor-sweep [--repos <a,b,c>]
```

- `--repos` overrides the repo rotation for a hand-run (comma-separated repo
  names, as they appear in the workspace `.repos` manifests). It replaces the
  chunk selection **only** — every other rule in step 2 still applies: the
  no-manifest FAILED guard, the non-GitHub exclusion, and the onboarding probe.
  A repo named on `--repos` that those filters exclude is reported as excluded,
  not audited anyway — to audit an excluded repo deliberately, run
  `/audit-project <repo>` directly rather than through the sweep.

## Overview

**Lifecycle position**: Utility/periodic — the "garbage collection" pass over
the workspace's own staleness detectors. Not tied to the per-issue lifecycle.

Four staleness/drift detectors already exist, and all four report only into the
conversation that ran them: `audit-workspace`, `audit-project`, `issue-triage`,
and the freshness header of `.agent/knowledge/research_digest.md`. This skill
chains them into **one sweep** and writes that sweep to **one local report
file**.

It is **report-only**, and in this slice it is also **publish-nothing**: no
PRs, no per-finding issues, no rolling GitHub issue, no comments. The report is
a local file and the conversation that ran the sweep. The operator triages it
into work. See **Deferred: publishing and the trigger**.

It also **must not assume `layers/` exists** — `layers/` is gitignored and
absent in every worktree, fresh clone and container. Repo lookups go through
`.agent/scripts/resolve_repo_checkout.sh`, which falls back to a shallow clone.

## The status contract

Every check ends in exactly one of four states, and the state is always
reported:

| Status | Meaning |
|---|---|
| `OK` | The check ran to completion and found nothing |
| `FINDINGS` | The check ran to completion and found something |
| `SKIPPED(<reason>)` | The check did not run, for a named and expected reason |
| `FAILED(<reason>)` | The check could not run, or errored partway |

**A check that could not run is never rendered as a pass**, and the words
"clean" / "no findings" may appear in the report only when all four checks
completed (`OK` or `FINDINGS`). This is the #609 false-green rule applied to a
report: an empty result and an unobtainable result are different answers.

## Steps

### 1. Resolve the workspace root and the report directory

Everything the sweep reads or writes is anchored at the **main workspace
root**, never the current worktree. This is load-bearing, not tidiness: the
sweep's primary environment is a fully set-up host, and it is usually invoked
from a worktree — where `layers/` and `configs/manifest` do not exist. A
worktree-relative path would enumerate zero repos on a host that has 35.

```bash
ROOT=$(git rev-parse --path-format=absolute --git-common-dir 2>/dev/null) \
    && ROOT=$(dirname "$ROOT") || ROOT=$(pwd)
REPORT_DIR="$ROOT/.agent/scratchpad/janitor"
mkdir -p "$REPORT_DIR"
```

`.gitignore` already covers `.agent/scratchpad/*`, so nothing here is ever
committed. Use `"$ROOT/..."` for **every** script and file the sweep touches —
`list_overlay_repos.py`, `research_digest.md`, `docs/`, the report directory.

The three environments this must work in, and how each is verified before a
change to this step ships:

| Environment | Expected | How to verify |
|---|---|---|
| Fully set-up host, run from the main checkout | `$ROOT` is the main root; repos enumerate | `python3 "$ROOT/.agent/scripts/list_overlay_repos.py" --format names \| wc -l` is non-zero |
| Fully set-up host, run from a worktree | `$ROOT` is still the **main** root | the same command, run from the worktree, returns the same count — covered mechanically for the resolver by `test_resolve_repo_checkout.sh`'s worktree case |
| No `layers/` at all (fresh clone, container) | repos still enumerate from `configs/manifest`; `audit-project` clones what it needs | `ls "$ROOT/layers"` is absent while the count above is non-zero; `resolve_repo_checkout.sh <repo>` prints mode `clone` |

If `configs/manifest` is itself absent — an un-bootstrapped clone — step 2's
first rule fires and the sweep is `FAILED`, not empty.

### 2. Build the repo rotation

```bash
python3 "$ROOT/.agent/scripts/list_overlay_repos.py"
```

Then, in order:

1. **If the list is empty, stop and report
   `FAILED(no repo manifest configured — run 'make setup-all')`.** This is a
   deliberate, named failure. `configs/manifest` is gitignored and absent in
   every fresh clone, and `list_overlay_repos.py` prints an empty list at
   **exit 0** in that state — rendering it as "no repos to audit" is exactly
   the false green this contract forbids. A non-zero exit from that script is
   `FAILED(manifest unreadable: <stderr>)` — also not an empty list.
2. Exclude repos whose origin is not on the GitHub allowlist. Do not hand-roll
   the host list: `.agent/scripts/field_mode.sh` is the authoritative source
   (AGENTS.md § Field Mode) and admits `ssh.github.com` — the SSH-over-443
   fallback — alongside `github.com`. A gitcloud/Forgejo origin is listed as
   **excluded: non-GitHub origin, not reachable from a generic runner** —
   never silently dropped.
3. Probe the survivors for onboarding. The slug comes from the manifest URL
   (`<owner>/<repo>`), never from a hardcoded workspace name — this skill
   contains no repo slug of its own, so a fork of this workspace sweeps its own
   repos (ADR-0003).

   ```bash
   gh api "repos/$SLUG" >/dev/null            # 1. is the repo visible at all?
   gh api "repos/$SLUG/contents/AGENTS.md"    # 2. only then, is it onboarded?
   ```

   Both probes are needed, because a 404 on the second one alone means two
   different things — "this repo has no root `AGENTS.md`" and "this token
   cannot see this repo" — and publishing the second as the first states a
   check that never ran as a finding about the repo. So:
   - repo probe fails (404, auth, rate limit, network) → `FAILED(repo probe:
     <reason>)` for that repo. Not an exclusion.
   - repo visible, `AGENTS.md` 404 → **excluded: no root `AGENTS.md`**.
   - `AGENTS.md` probe fails for any other reason → `FAILED(onboarding probe:
     <reason>)`.

   What this gates on is **presence**, not currency: ADR-0017's currency signal
   is the `## Quality Standard` marker *inside* the file, which `audit-project`
   step 2 checks once the repo is in the rotation. A present-but-stale
   `AGENTS.md` is a finding for the audit to make, not a reason to skip the
   repo.
4. If repos were enumerated but every one was excluded, that is
   `SKIPPED(no eligible repos: <reasons>)` — distinct from rule 1's FAILED.
5. Sort the survivors by name, chunk by 3, and select chunk
   `ISO-week mod chunk-count`.

The report always lists the **full** candidate set with each repo's in/out
status and reason, plus the chunk index and the ISO week used.

### 3. Run the four checks

Run each in turn and record its status per the contract above.

| # | Check | How | Layer-dependent? |
|---|---|---|---|
| 1 | Workspace governance | `/audit-workspace`, full | No |
| 2 | Project governance | `/audit-project <repo>` for each repo in the rotation chunk | No — `audit-project` resolves a clone when there is no layer checkout, and reports its two layer-dependent items as SKIPPED |
| 3 | Issue staleness | `/issue-triage --stale-days 90` | No |
| 4 | Research-digest freshness | Below | No |

Each check's FAILED evidence is named, because two of these are sub-skills that
report into the conversation rather than returning an exit code — "it seemed to
run" is not a status.

- **Check 1 — `audit-workspace`** is `FAILED` when an input it needs cannot be
  read (`$ROOT/docs/PRINCIPLES.md`, `$ROOT/docs/decisions/`, `$ROOT/AGENTS.md`,
  `$ROOT/.agent/templates/`), or when its run ends without producing all seven
  checklist sections. A section it could not complete is `SKIPPED(<reason>)`
  inside the audit and makes the check `FINDINGS` at worst, never `OK`. Probe
  the inputs before reporting the check's status; do not infer it from the
  narrative.
- **Check 2 — `audit-project`** rolls up its per-repo runs: `FAILED` if any
  repo failed to resolve (any non-zero exit from `resolve_repo_checkout.sh`) or
  failed to audit, otherwise `FINDINGS` if any repo produced findings,
  otherwise `OK`. Per-repo statuses are listed individually regardless.
- **Check 3 — `issue-triage`** is `FAILED` when it scanned no repos (its step 1
  now carries the same empty-manifest guard as rule 1 above — a scan of zero
  repos reports no stale issues, which is not the same as there being none),
  when `gh` is unauthenticated, or when any per-repo `gh issue list` errors.
  A repo whose issue list could not be fetched is named, and the check is
  `FAILED` — not an `OK` over the repos that did answer.
- **Check 4** — below.

For check 4, read `$ROOT/.agent/knowledge/research_digest.md` and apply the
thresholds the file declares in its own header comment: the file-level
`<!-- Last updated: YYYY-MM-DD -->` stamp is a finding when older than
**30 days** ("consider running `/research --refresh`"), and any per-entry
`**Updated**: YYYY-MM-DD` stamp older than **90 days** is a finding ("should be
flagged for review"). A missing or unparseable header is
`FAILED(digest header unreadable)`, not OK.

### 4. Write the report

Write the full report to a **timestamped** file:

```bash
REPORT="$REPORT_DIR/$(date '+%Y%m%dT%H%M%S')-sweep.md"
```

Generate the timestamp with `date`; never hand-type one (AGENTS.md
§ Documentation Accuracy). A date-only name would let a second run the same day
overwrite the first run's report — the runs are the record, so they must
accumulate.

This write depends on neither network nor auth, so the sweep always produces
its record. It is the sweep's durable output.

**Keep the report free of host identity and absolute local paths.** Name files
relative to the workspace root (`.agent/knowledge/research_digest.md`, not
`/home/…/.agent/knowledge/research_digest.md`), and do not record the hostname.
Nothing publishes this report today, but the deferred publish decision lands on
a repo that may be public, and a report written this way needs no scrubbing
when it gets there. Excluded repo *names* are fine — they are already in the
tracked manifests.

Report format:

```markdown
## Janitor Sweep — <YYYY-MM-DD HH:MM ±HH:MM>

**Checks**: X of 4 completed, Y skipped, Z failed
**Repo rotation**: chunk <i> of <n>, ISO week <YYYY-Www>

### Check Status

| Check | Status | Detail |
|---|---|---|
| Workspace governance (`audit-workspace`) | OK / FINDINGS / SKIPPED(...) / FAILED(...) | ... |
| Project governance (`audit-project`) | ... | <n> repos audited |
| Issue staleness (`issue-triage`) | ... | <n> repos scanned |
| Research-digest freshness | ... | last updated <date>, <n> days |

### Findings

#### Workspace governance
- ...

#### Project governance — <repo> (mode: layer/clone)
- ...

#### Issue staleness
- ...

#### Research-digest freshness
- ...

### Repos not audited this run

| Repo | Reason |
|---|---|
| <repo> | excluded: non-GitHub origin |
| <repo> | excluded: no root AGENTS.md |
| <repo> | not in this week's chunk |
```

Fill the **Checks** line from the status table, not from impression. If any
check is `SKIPPED` or `FAILED`, the report may not describe the workspace as
clean.

### 5. Report to the operator

Summarise in the conversation and name the report file by its path **relative
to the workspace root** (`.agent/scratchpad/janitor/<file>`). Lead with the
coverage line — `X of 4 completed` — so a partial sweep cannot read as a clean
one. Nothing is published; say so plainly rather than leaving it ambiguous.

## Known limitations

Both are deliberate, and both are for the deferred decision below to close —
they are listed here so that decision meets them rather than rediscovering
them.

- **The report is host-local and ephemeral.** `$REPORT_DIR` lives under
  `.agent/scratchpad/`, which is gitignored. Anchoring it at the main workspace
  root (step 1) means every *worktree on a host* shares one report directory,
  but a sweep run on another host, or in an ephemeral container, leaves a
  record nobody else can read — and a container's copy dies with the container.
  Until publishing is decided, a sweep is only as durable as the host it ran
  on.
- **The rotation has no coverage guarantee while the trigger is deferred.**
  `ISO-week mod chunk-count` cycles every repo in `ceil(N/3)` weeks *only under
  a real weekly trigger*. Two hand-runs in the same week re-audit the same
  chunk, and a week with no run is never made up. No cursor state is kept for
  this on purpose — statelessness is what lets the sweep run identically from a
  worktree, a container, or a fresh clone. The report names the chunk index and
  ISO week so a reader can see which slice was covered.

## Deferred: publishing and the trigger

This slice writes a local report and nothing else. **Publishing and the trigger
are one decision, deferred together** (operator decision, 2026-09-11 on issue
#569): where a sweep's findings live durably depends on what runs the sweep,
and deciding either half alone would fix the other by accident.

Whatever is chosen, a future GitHub publish step is a GitHub **write**, which
is the crux:

- [ADR-0015](../../../docs/decisions/0015-dispatch-handoff-context-contract.md)
  — a dispatched container has no GitHub write auth; the container produces and
  the host publishes. A container-based trigger inherits that split.
- [ADR-0019](../../../docs/decisions/0019-what-contains-a-dispatched-agent.md)
  — what containment does and does not buy, before assuming a sandboxed runner
  is equivalent.

Cite these rather than re-deriving them. The decision also inherits both
**Known limitations** above, and must settle what a report may say on a public
repo before anything is posted there — which is why step 4 already keeps the
host and absolute paths out of it.

## Why no `progress.md` entry

`progress.md` is keyed by issue (ADR-0013), and this skill is not issue-scoped
— like its three periodic siblings (`audit-workspace`, `audit-project`,
`issue-triage`), none of which write one. Its durable record is the local
report file written in step 4.

## Guidelines

- **Report, don't fix** — this skill identifies staleness. Fixing it is
  separate work with its own issues, decided by the operator. Do not open PRs
  and do not file per-finding issues; the operator has asked explicitly for one
  report over issue spam.
- **Publish nothing** — not this slice. No issue, no comment, no PR.
- **Never report a check you did not run** — `SKIPPED` and `FAILED` carry a
  reason, and both are visible in the status table and the headline count.
- **Be specific** — "`research_digest.md` last updated 2026-05-02, 132 days, 6
  entries past the 90-day threshold" is actionable; "digest may be stale" is
  not.
- **Don't nitpick** — inherit each chained skill's own judgement about what is
  worth flagging; the sweep aggregates, it does not re-grade.
