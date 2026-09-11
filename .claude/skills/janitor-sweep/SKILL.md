---
name: janitor-sweep
description: Run the workspace's staleness and drift detectors in one pass and publish the result to a single rolling GitHub report issue. Report-only — opens no PRs and files no per-finding issues.
---

# Janitor Sweep

## Usage

```
/janitor-sweep [--repos <a,b,c>] [--dry-run]
```

- `--repos` overrides the repo rotation for a hand-run (comma-separated repo
  names, as they appear in the workspace `.repos` manifests).
- `--dry-run` writes the local report but publishes nothing.

## Overview

**Lifecycle position**: Utility/periodic — the "garbage collection" pass over
the workspace's own staleness detectors. Not tied to the per-issue lifecycle.

Four staleness/drift detectors already exist, and all four report only into the
conversation that ran them: `audit-workspace`, `audit-project`, `issue-triage`,
and the freshness header of `.agent/knowledge/research_digest.md`. This skill
chains them into **one sweep** and gives that sweep a **durable home**: a single
rolling GitHub issue whose body is the latest snapshot and whose comments are
the history.

It is **report-only**. It opens no PRs, files no per-finding issues, and fixes
nothing. The operator triages the report into work. The one write it makes is to
the rolling report issue.

It also **must not assume `layers/` exists** — `layers/` is gitignored and
absent in every worktree, fresh clone and container. Repo lookups go through
`.agent/scripts/resolve_repo_checkout.sh`, which falls back to a shallow clone.

**Trigger is out of scope.** This skill is hand-run for now; see
**Deferred: the trigger** below.

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

The report and the clone cache are anchored at the **main workspace root**, not
the current worktree, so every worktree on a host shares one of each:

```bash
ROOT=$(git rev-parse --path-format=absolute --git-common-dir 2>/dev/null) \
    && ROOT=$(dirname "$ROOT") || ROOT=$(pwd)
REPORT_DIR="$ROOT/.agent/scratchpad/janitor"
mkdir -p "$REPORT_DIR"
```

`.gitignore` already covers `.agent/scratchpad/*`, so nothing here is ever
committed.

### 2. Post any report an earlier run left unpublished

Before doing any work of its own, look in `$REPORT_DIR` for a report from an
earlier run that was written but never published (step 7 leaves an
`.unpublished` marker beside it). Publish those first, oldest first, then
remove their markers. An unattended failed post must not vanish just because
nobody read that session.

See **Known limitations** — this backlog is host-local.

### 3. Build the repo rotation

```bash
python3 .agent/scripts/list_overlay_repos.py
```

Then, in order:

1. **If the list is empty, stop and report
   `FAILED(no repo manifest configured — run 'make setup-all')`.** This is a
   deliberate, named failure. `configs/manifest` is gitignored and absent in
   every worktree and fresh clone, and `list_overlay_repos.py` prints an empty
   list at **exit 0** in that state — rendering it as "no repos to audit" is
   exactly the false green this contract forbids.
2. Exclude repos whose URL host is not `github.com`. A gitcloud/Forgejo origin
   is listed as **excluded: non-GitHub origin, not reachable from a generic
   runner** — never silently dropped.
3. Probe the survivors for onboarding with
   `gh api repos/<owner>/<repo>/contents/AGENTS.md`. A repo with no root
   `AGENTS.md` has not adopted ADR-0017's marker and is listed as
   **excluded: not onboarded**. A probe that errors for any other reason
   (auth, rate limit, network) is **not** an exclusion — it is
   `FAILED(repo probe: <reason>)` for that repo.
4. If repos were enumerated but every one was excluded, that is
   `SKIPPED(no eligible repos: <reasons>)` — distinct from step 1's FAILED.
5. Sort the survivors by name, chunk by 3, and select chunk
   `ISO-week mod chunk-count`. `--repos` replaces this selection entirely.

The report always lists the **full** candidate set with each repo's in/out
status and reason, plus the chunk index and the ISO week used.

### 4. Run the four checks

Run each in turn and record its status per the contract above.

| # | Check | How | Layer-dependent? |
|---|---|---|---|
| 1 | Workspace governance | `/audit-workspace`, full | No |
| 2 | Project governance | `/audit-project <repo>` for each repo in the rotation chunk | No — `audit-project` resolves a clone when there is no layer checkout, and reports its two layer-dependent items as SKIPPED |
| 3 | Issue staleness | `/issue-triage --stale-days 90` | No |
| 4 | Research-digest freshness | Below | No |

For check 4, read `.agent/knowledge/research_digest.md` and apply the
thresholds the file declares in its own header comment: the file-level
`<!-- Last updated: YYYY-MM-DD -->` stamp is a finding when older than
**30 days** ("consider running `/research --refresh`"), and any per-entry
`**Updated**: YYYY-MM-DD` stamp older than **90 days** is a finding ("should be
flagged for review"). A missing or unparseable header is
`FAILED(digest header unreadable)`, not OK.

Check 2's status is the roll-up of its per-repo runs: `FAILED` if any repo
failed to resolve or audit, otherwise `FINDINGS` if any repo produced findings,
otherwise `OK`. Per-repo statuses are listed individually regardless.

### 5. Write the report locally — this happens first, always

Write the full report to `$REPORT_DIR/<YYYY-MM-DD>-sweep.md` **before** any
GitHub call. This write depends on neither network nor auth, so the sweep
always produces its record even when publishing is impossible. This mirrors
`review-issue`'s canonical-local-write-then-post pattern.

Report format:

```markdown
## Janitor Sweep — YYYY-MM-DD

**Checks**: X of 4 completed, Y skipped, Z failed
**Repo rotation**: chunk <i> of <n>, ISO week <YYYY-Www>
**Host**: <hostname>

### Check Status

| Check | Status | Detail |
|---|---|---|
| Workspace governance (`audit-workspace`) | OK / FINDINGS / SKIPPED(...) / FAILED(...) | ... |
| Project governance (`audit-project`) | ... | <n> repos audited |
| Issue staleness (`issue-triage`) | ... | ... |
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
| <repo> | excluded: not onboarded (no root AGENTS.md) |
| <repo> | not in this week's chunk |
```

Fill the **Checks** line from the status table, not from impression. If any
check is `SKIPPED` or `FAILED`, the report may not describe the workspace as
clean.

### 6. Find the rolling issue

Exactly one open issue on `rolker/ros2_agent_workspace` titled
**`Janitor sweep report (rolling)`** is the report's home. Match the title
**exactly** — a substring search will collect look-alikes:

```bash
gh issue list --repo rolker/ros2_agent_workspace --state open \
    --search 'Janitor sweep report (rolling) in:title' \
    --json number,title \
    --jq '[.[] | select(.title == "Janitor sweep report (rolling)")]'
```

| Matches | Action |
|---|---|
| 1 | Use it (step 7) |
| 0 | Create it, once (step 7a) |
| 2 or more | `FAILED(ambiguous rolling issue: #<a>, #<b>)` — **never guess**; the report stays local and the operator resolves the duplication |

### 7. Publish

**7a. Create, only when there were zero matches.** Use plain `gh issue create` —
**not** `.agent/scripts/gh_create_issue.sh`, which auto-injects
`Part of #<WORKTREE_ISSUE>` when that variable is set (gh_create_issue.sh:82-95)
and would permanently mis-parent the rolling issue to whatever issue the
operator happened to be working on. No label: none of the workspace's labels
fit, and `gh issue create` requires none.

```bash
gh issue create --repo rolker/ros2_agent_workspace \
    --title "Janitor sweep report (rolling)" \
    --body-file "$REPORT_DIR/<YYYY-MM-DD>-sweep.md"
```

**7b. Update in place, then comment.** Set the issue *body* to the current
snapshot and post the same report as a *comment*, so "one rolling issue" and an
audit trail both hold:

```bash
gh issue edit <N> --repo rolker/ros2_agent_workspace --body-file <report>
gh issue comment <N> --repo rolker/ros2_agent_workspace --body-file <report>
```

Use `--body-file`, never `--body` (AGENTS.md § GitHub CLI Patterns), and append
the AGENTS.md AI signature to the created issue and to every comment:

```markdown
---
**Authored-By**: `$AGENT_NAME`
**Model**: `$AGENT_MODEL`
```

**7c. When publishing fails.** Leave the local report in place, touch an
`.unpublished` marker beside it, and make the run's headline:

```
POST FAILED — report written to <absolute path> on <hostname>, not published
```

Do not report the sweep as complete. The next run on this host picks the report
up at step 2.

`--dry-run` stops after step 5 and says so explicitly — a dry run is reported
as `not published (--dry-run)`, never as published.

## Known limitations

Both are deliberate, and both are for the trigger decision to close — they are
listed here so that decision meets them rather than rediscovering them.

- **The failed-post backlog is host-local.** `$REPORT_DIR` lives under
  `.agent/scratchpad/`, which is gitignored. Anchoring it at the main workspace
  root (step 1) means every *worktree on a host* shares one backlog, but a
  sweep run on a different host, or in an ephemeral container, cannot see
  another host's unpublished report. The mitigation is that an unpublished run
  is loud in its own right — the `POST FAILED` headline names the absolute path
  and the host — and that whichever trigger is chosen must carry its own
  failure signal. A trigger pinned to one host makes this backlog sufficient;
  an ephemeral one does not.
- **The rotation has no coverage guarantee while the trigger is deferred.**
  `ISO-week mod chunk-count` cycles every repo in `ceil(N/3)` weeks *only under
  a real weekly trigger*. Two hand-runs in the same week re-audit the same
  chunk, and a week with no run is never made up. No cursor state is kept for
  this on purpose — statelessness is what lets the sweep run identically from a
  worktree, a container, or a fresh clone. The report names the chunk index and
  ISO week so a reader can see which slice was covered.

## Deferred: the trigger

Scheduling is explicitly out of scope for this slice (operator scope decision,
2026-09-11 on issue #569). When it is decided, note that **updating the rolling
issue is a GitHub write**, which is the crux:

- [ADR-0015](../../../docs/decisions/0015-dispatch-handoff-context-contract.md)
  — a dispatched container has no GitHub write auth; the container produces and
  the host publishes. A container-based trigger inherits that split.
- [ADR-0019](../../../docs/decisions/0019-what-contains-a-dispatched-agent.md)
  — what containment does and does not buy, before assuming a sandboxed runner
  is equivalent.

Cite these rather than re-deriving them. The two **Known limitations** above
are also that decision's to close.

## Why no `progress.md` entry

`progress.md` is keyed by issue (ADR-0013), and this skill is not issue-scoped
— like its three periodic siblings (`audit-workspace`, `audit-project`,
`issue-triage`), none of which write one. Its durable record is the rolling
report issue, plus the local canonical file that is written first. That is why
step 7c exists: without a named `POST FAILED` state, an unpublished sweep would
be indistinguishable from "nothing to report".

## Guidelines

- **Report, don't fix** — this skill identifies staleness. Fixing it is
  separate work with its own issues, decided by the operator. Do not open PRs
  and do not file per-finding issues; the operator has asked explicitly for one
  report over issue spam.
- **One rolling issue, never a second** — if the exact-title search returns two
  or more, that is a failure to report, not a choice to make.
- **Never report a check you did not run** — `SKIPPED` and `FAILED` carry a
  reason, and both are visible in the status table and the headline count.
- **Be specific** — "`research_digest.md` last updated 2026-05-02, 132 days, 6
  entries past the 90-day threshold" is actionable; "digest may be stale" is
  not.
- **Don't nitpick** — inherit each chained skill's own judgement about what is
  worth flagging; the sweep aggregates, it does not re-grade.
