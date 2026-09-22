---
name: janitor-sweep
description: Run the workspace's staleness and drift detectors in one pass, split into a workspace scope and a project scope. Local-first - every run writes a local report; committing docs/health.md via PR is opt-in behind --publish. Neither scope files per-finding issues.
---

# Janitor Sweep

## Usage

```
/janitor-sweep [--repos <a,b,c>] [--publish]
```

- `--repos` overrides the repo rotation for a hand-run (comma-separated repo
  names, as they appear in the workspace `.repos` manifests). It replaces the
  chunk selection **only** — every other rule in step 2 still applies: the
  no-manifest FAILED guard, the non-GitHub exclusion, and the onboarding probe.
  A repo named on `--repos` that those filters exclude is reported as excluded,
  not audited anyway — to audit an excluded repo deliberately, run
  `/audit-project <repo>` directly rather than through the sweep.
- `--publish` commits `docs/health.md` at the workspace repo root and opens a
  PR for the workspace scope (§ Publish the workspace scope). **Off by
  default** — the sweep is local-first: every run writes its local report
  under `.agent/scratchpad/janitor/` regardless, and the flag gates only the
  commit-and-PR flow and the workspace scope's diff source (§ Run-over-run
  diff). Read `$PUBLISH` below as `1` when the flag was passed and empty
  otherwise; it is the one switch this whole document keys on.

  The flag exists rather than the step being deleted, for two stated uses: a
  hand run that wants to refresh the committed baseline, and the parked cloud
  trigger ([#636](https://github.com/rolker/ros2_agent_workspace/issues/636)),
  which is the step's eventual caller. Parked is not obsolete.

## Overview

**Lifecycle position**: Utility/periodic — the "garbage collection" pass over
the workspace's own staleness detectors. Not tied to the per-issue lifecycle.

Four staleness/drift detectors already exist, and all four report only into the
conversation that ran them: `audit-workspace`, `audit-project`, `issue-triage`,
and the freshness header of `.agent/knowledge/research_digest.md`. This skill
chains them into **one sweep** and splits the result into two scopes:

- **Workspace scope** — check 1 (`audit-workspace`) + check 4 (research-digest
  freshness). Both grade the workspace repo itself.
- **Project scope** — check 2 (`audit-project`, per repo in the rotation
  chunk) + check 3 (`issue-triage`, which already scans only overlay/project
  repos, never the workspace repo).

**The sweep is local-first.** Every run's durable output is the local
report(s) under `.agent/scratchpad/janitor/` — written whether or not the
run publishes, whether or not it has network or GitHub auth. **Committing
`docs/health.md` at the repo root and opening a PR is opt-in, behind
`--publish`** (§ Publish the workspace scope); the committed file on `main`
is a baseline a non-publishing run neither rewrites nor contradicts.
A run may also write an **optional second local report for the project this
clone is configured for** — `<ts>-<pid>-<repo>-health.md`, beside the workspace
one — when that project's root repo keeps a root `ROADMAP.md`: health
follows the roadmap, and that is the whole opt-in (§ Resolve the configured
project root; § Render, redact, and write the report). A checkout with no
project configured, and a project with no root roadmap, each simply get what
they get today.

**Project scope is never published by this skill** — its findings land only
in the local report(s), pending the operator's decision on the project-repo
health-rollup shape (per-repo PR / one document per project /
health-follows-roadmap — not yet made). Neither scope files per-finding
issues or opens issues on its own initiative; the operator triages findings
into work. See **Deferred: the project-scope rollup and the trigger**.

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

**The report write is itself a state, and it is the fifth one.** The four
states above grade the four *checks*; the sweep's durable output is the report
file, and that write can fail — a read-only or full filesystem, a
`$REPORT_DIR` that cannot be created, a scratchpad on a container volume that
is not mounted. A sweep whose report did not land is
`FAILED(report write: <reason>)`, announced in the conversation as the
headline, with the findings printed inline so the run is not lost with the
file. The consequences map requires a durable-output skill to name the state
in which that write failed
([`principles_review_guide.md`](../../../.agent/knowledge/principles_review_guide.md)),
and this is that state. It is never "the sweep ran clean".

**The workspace-scope commit-and-PR (§ Publish the workspace scope) is a
sixth, separate state** — on runs that ask for it. It happens only under
`--publish`; a default run records
`## Publish outcome: not requested (--publish off)` and has no sixth state to
report, which is itself the state and is never rendered as a success or a
failure. Where the step does run, it sits on top of the five above: the local
report can land even when the `docs/health.md` publish fails (a worktree that could not be
created, a push that was rejected, a PR that could not be opened), and the
reverse also holds (a report-write failure per the fifth state above is
already terminal for the run and the publish step is never reached). The two
never stand in for each other — a failed publish is stated by name, next to
the local report's own status, never folded into it.

**Step 1a's project-root resolution is not a check and has no state of its
own here** — it decides whether the optional per-project report can be
written at all, and its four outcomes (§ 1a) are reported on that report's
own line in step 8, never folded into the four check states. One of them is
a failure: `FAILED(project root probe: <reason>)`, the checkout was there
but could not be probed. It is stated exactly as plainly as the other three,
and it is distinct from the seventh state below — the probe failing means no
report could be *built*; the seventh means one was built and could not be
*written*.

**The optional per-project report's write is a seventh state** (§ 6): the
same "the report write is itself a state" rule as the fifth, named
separately — `FAILED(project health write: <reason>)` — so it can never be
confused with the primary report's write failure or with a publish failure.
Unlike the fifth, it is **not terminal for the run**: by the time it is
attempted the primary report has already been written successfully, so it is
reported alongside the run's other outcomes (step 8), never in place of
them. The same relationship the sixth state has to the fifth.

## Steps

### 1. Resolve the workspace root and the report directory

Everything the sweep reads or writes is anchored at the **main workspace
root**, never the current worktree. This is load-bearing, not tidiness: the
sweep's primary environment is a fully set-up host, and it is usually invoked
from a worktree — where `layers/` and `configs/manifest` do not exist. A
worktree-relative path would enumerate zero repos on a host that has 35.

```bash
# The workspace root is the nearest ancestor that HOLDS the workspace
# (`.agent/scripts/` + `configs/`) — not whatever repo you happen to be
# standing in. `workspace_root.sh` has the last word: it validates
# $WORKSPACE_ROOT when set, and hops a worktree to the MAIN checkout, where
# `layers/` and the `configs/manifest` symlink actually live. The walk is
# inline because a script cannot be called from a directory not yet found —
# and it STARTS at $WORKSPACE_ROOT when that is set, which is what makes "set
# WORKSPACE_ROOT" a real remedy: the variable is read only *inside* the
# script, so a walk that always began at $(pwd) could never reach a copy of
# the script to read it, and the remedy printed below would be inert on
# exactly the path that prints it. `[ -f ]` + `bash` rather than `[ -x ]`:
# exec bits are lost on a noexec mount, an unpacked archive or a CIFS share,
# and a missing +x is not a reason to walk past the workspace. $WORKSPACE_ROOT
# is normalised to an ABSOLUTE path before the walk: `dirname .` is `.`, so a
# relative value would make the loop below spin forever. A value that cannot
# be entered at all falls back to $(pwd) so the walk still terminates — the
# failure is then reported below, naming the $WORKSPACE_ROOT that was set.
d=$(cd "${WORKSPACE_ROOT:-$(pwd)}" 2>/dev/null && pwd) || d=$(pwd); ROOT=""
while [ "$d" != "/" ]; do
    if [ -f "$d/.agent/scripts/workspace_root.sh" ]; then
        ROOT=$(bash "$d/.agent/scripts/workspace_root.sh") || ROOT=""
        break
    fi
    d=$(dirname "$d")
done
if [ -z "$ROOT" ]; then
    echo "FAILED(workspace root: none above ${WORKSPACE_ROOT:-$(pwd)} — run from the workspace, or set WORKSPACE_ROOT to one)"
    exit 1
fi

# redact.sh and $REDACT_PATH_PREFIXES are set up here, before the report
# directory is even created, because EVERY failure from this point on —
# starting with the `mkdir` two lines down — is a reason string that can name
# an absolute path, and this skill's own contract forbids host identity in the
# report. Loading it late (as a prior version of this step did, just before
# sourcing manifest_fallback.sh) left the `mkdir` failure printing $REPORT_DIR
# verbatim. `declare -F` guards the load the same way resolve_repo_checkout.sh
# and manifest_fallback.sh guard their own: a `source` can succeed as a
# no-op on a truncated file, leaving the functions undefined.
REDACT_PATH_PREFIXES=("$ROOT=<workspace>")
if [ -n "${HOME:-}" ] && [ "$HOME" != "/" ] && [ "$HOME" != "$ROOT" ]; then
    REDACT_PATH_PREFIXES+=("$HOME=~")
fi
# shellcheck source=/dev/null
if ! source "$ROOT/.agent/scripts/redact.sh" \
   || ! declare -F redact_url >/dev/null 2>&1 \
   || ! declare -F redact_text >/dev/null 2>&1; then
    echo "FAILED(redact.sh unusable — missing or will not load; cannot safely print any further failure reason)"
    exit 1
fi

REPORT_DIR="$ROOT/.agent/scratchpad/janitor"
if ! mkdir -p "$REPORT_DIR"; then
    echo "FAILED(report directory: cannot create $(redact_text "$REPORT_DIR"))"
    exit 1
fi

# The one switch the rest of this document keys on (§ Usage). Assigned once,
# here, and never inferred later from whether a PR exists or a worktree was
# created: `1` when --publish was passed on the invocation, empty otherwise.
# Set before any branch reads it, so it is never unbound under `set -u`.
PUBLISH=""
case " $* " in *" --publish "*) PUBLISH=1 ;; esac
```

This is the same snippet `audit-project` step 1 uses. It resolves the workspace
from a *layer* worktree too, where the earlier `git --git-common-dir` form
answered with the project repo's own root — a root with no `layers/`, no
`configs/`, and no scripts to call.

`.gitignore` already covers `.agent/scratchpad/*`, so nothing here is ever
committed. Use `"$ROOT/..."` for **every** script and file the sweep touches —
`list_overlay_repos.py`, `research_digest.md`, `docs/`, the report directory.

**Where there is no `configs/manifest`** — a fresh clone or a container, where
`layers/` does not exist and `configs/manifest` is a symlink *into* it — the
manifests have to be cloned before any repo can be enumerated. That is
`manifest_fallback.sh`'s job, and it is the first thing "the sweep clones what
it needs" covers:

```bash
# `manifest_fallback.sh` routes every diagnostic through `redact.sh`, which
# rewrites absolute paths only when the CALLER says which prefixes to strip.
# `$REDACT_PATH_PREFIXES` and `redact.sh` itself were already set up in step 1
# (before the report directory was even created), so nothing further is needed
# here — `manifest_fallback.sh` sources `redact.sh` itself only when the
# functions are not already declared, so this does not re-source it.
#
# The `source` itself can fail — exit 5, "the redact.sh it routes diagnostics
# through is missing or will not load". Unchecked, that surfaces later as
# `manifest_config_dir: command not found`, which is not a code any arm below
# claims. Check it here, where the reason is still on stderr.
if ! source "$ROOT/.agent/scripts/manifest_fallback.sh"; then
    echo "FAILED(manifest fallback unusable — the reason is on stderr)"
    exit 1
fi
EXTRA_CONFIG=""
LIST_ARGS=()
if EXTRA_CONFIG=$(manifest_config_dir "$ROOT"); then
    # empty when the workspace has its own manifest (the normal case)
    if [ -n "$EXTRA_CONFIG" ]; then
        LIST_ARGS+=(--config-dir "$EXTRA_CONFIG")
    fi
    python3 "$ROOT/.agent/scripts/list_overlay_repos.py" --format json "${LIST_ARGS[@]}"
else
    # Every arm is terminal: the sweep is FAILED and nothing is enumerated.
    # The status is captured before anything else runs — read inside an arm it
    # is no longer reliably the one `case` branched on.
    rc=$?
    case "$rc" in
        3) echo "FAILED(no repo manifest configured — run 'make setup-all')" ;;
        5) echo "FAILED(manifest repo unreachable — the reason is on stderr)" ;;
        6) echo "FAILED(manifest refresh — the reason is on stderr)" ;;
        *) echo "FAILED(manifest fallback: unexpected exit $rc)" ;;
    esac
fi
```

`$EXTRA_CONFIG` and `$LIST_ARGS` are what the rest of the sweep enumerates
with — **every** later `list_overlay_repos.py` call passes `"${LIST_ARGS[@]}"`,
and the optional-layer lookup in step 2 passes `$EXTRA_CONFIG`. Both are set
(empty) before the `if`, so neither is unbound under `set -u`, and neither
failure arm falls through to an enumeration.

A manifest clone that was **attempted and failed** is
`FAILED(manifest repo unreachable: <reason>)` — never rule 1's "no repo
manifest configured — run `make setup-all`", which would send the operator to a
command that cannot fix an unreachable remote.

A **cached** manifest clone that could not be refreshed (rc 6) is
`FAILED(manifest refresh: <reason>)`, and it is terminal in the same way. The
cached copy is still on disk and still readable, which is exactly why it needs
its own state: enumerating from it would produce a full four-check run over a
repo list — which repos exist, at which versions — that this host could not
verify, and the report would then say `4 of 4 completed`. An unverifiable
result is not an empty one and not a clean one; per the status contract it is
`FAILED`, with the reason named.

The three environments this must work in, and how each is verified before a
change to this step ships:

| Environment | Expected | How to verify |
|---|---|---|
| Fully set-up host, run from the main checkout | `$ROOT` is the main root; repos enumerate | `python3 "$ROOT/.agent/scripts/list_overlay_repos.py" --format names \| wc -l` is non-zero |
| Fully set-up host, run from a worktree | `$ROOT` is still the **main** root | the same command, run from the worktree, returns the same count — covered mechanically for the resolver by `test_resolve_repo_checkout.sh`'s worktree case |
| No `layers/` at all (fresh clone, container) | **no manifest is on disk** (`configs/manifest` is a symlink into `layers/`), so the manifest repo is shallow-cloned from the tracked `configs/project_bootstrap.url` pointer and the repos enumerate from that clone; `audit-project` then clones each repo it audits | `ls "$ROOT/layers"` is absent; `manifest_config_dir "$ROOT"` prints a path under `.agent/scratchpad/manifest-repo/`, and `list_overlay_repos.py --config-dir "$(manifest_config_dir "$ROOT")" --format names \| wc -l` is non-zero; `resolve_repo_checkout.sh <repo>` prints mode `clone` — covered mechanically by `test_resolve_repo_checkout.sh`'s manifest-fallback cases |

If there is neither a `configs/manifest` nor a usable bootstrap pointer — an
un-bootstrapped clone with nothing to derive from — step 2's first rule fires
and the sweep is `FAILED`, not empty.

**Retention.** Both outputs are caches with no automatic cleanup, and this is a
report-only skill that will run repeatedly:

- `$REPORT_DIR` — keep the **last 20** of **each** report shape; delete older
  ones at the end of a run. The reports are small, but nothing else will ever
  prune them, and a shape that matches no glob here accumulates forever:

  ```bash
  ls -1t "$REPORT_DIR"/*-sweep.md  | tail -n +21 | xargs -r rm -f
  ls -1t "$REPORT_DIR"/*-health.md | tail -n +21 | xargs -r rm -f
  ```

  Two globs, two counts, because the two shapes are written at different
  rates: a `<ts>-<pid>-<repo>-health.md` (step 6's optional per-project report) is
  written only on the runs where the project root repo fell in the rotation
  chunk, so pruning both under one combined count would evict per-project
  history spanning far more than 20 sweeps. Separate counts are also what
  keeps § 5's `prior report predates retention` wording true for each shape:
  "no prior report" and "pruned away" stay distinguishable instead of one
  silently becoming the other.
- `.agent/scratchpad/janitor-repos/` and `.agent/scratchpad/manifest-repo/` —
  disposable shallow clones that self-heal: the next run re-creates what it
  needs, so the whole-directory `rm -rf` is the reclaim to reach for. It is
  safe **when no sweep or audit is running on this host** — not at any time.
  `resolve_repo_checkout.sh` states the contract these clones are held under
  (see its "Concurrency" header): the per-repo `flock` serialises the
  clone/refresh only and is released when the script exits, so the caller reads
  the checkout **unlocked**, and concurrent audits of the same repo on one host
  are not safe. Deleting a cache out from under a running audit is the same
  hazard from the other side: the audit reads a vanished or half-recreated tree
  and the sweep reports a phantom finding (`MISSING: no root AGENTS.md`) that
  is indistinguishable from a real one. Say the condition — not a bare "safe to
  delete" — in the report footer, so an operator short of disk knows both what
  to delete and when (this host has hit 100% before, and routinely runs several
  agent sessions at once). Closing the window properly would take a lock the
  caller holds for the audit's duration; that is a change to
  `resolve_repo_checkout.sh`'s caller contract, deliberately out of scope for a
  report-only skill.

  **These, not the reports, are what fills a disk.** The reports are a few KB
  each and capped at 20; the caches are one shallow working tree per repo the
  sweep has *ever* resolved — including repos since dropped from the manifest,
  which no later run will ever touch again. They are deliberately left
  uncapped: pruning by age would delete a clone a concurrent audit is reading,
  and the whole-directory `rm -rf` above is the cheap reclaim — under the same
  quiescence condition, since it deletes strictly more. So the footer names the
  two directories **with their current sizes**
  (`du -sh "$ROOT/.agent/scratchpad/janitor-repos" "$ROOT/.agent/scratchpad/manifest-repo"`),
  rather than leaving the operator to discover the number when the disk is
  already full.

### 1a. Resolve the configured project root

The sweep writes an **optional second local report** for the project this
clone is configured for, when that project keeps a root `ROADMAP.md` (§ 6;
health follows the roadmap, per the design draft's expected-location table
and the two-root rule). This step decides whether there is such a project at
all. It is lettered `1a` rather than renumbered into the sequence so the many
existing cross-references to steps 5–8 stay correct; the skill already uses
lettered sub-steps (7a–7h).

**Identity from the tracked pointer; checkout from this host only; nothing is
cloned.** The project root is the manifest repo that
`configs/project_bootstrap.url` names — the repo a manifest entry points at,
per the two-root rule. Its *identity* is a string parse of that tracked
pointer (`manifest_bootstrap_identity`, no clone, no network). Its *checkout*
is only ever one that already exists under `$ROOT/layers/main/*/src/<repo>`.
Where `layers/` is absent, or the pointer's repo is not in it, **there is no
project on this checkout** and the per-project report is skipped — not
failed, and emphatically not cloned. That is
[#652](https://github.com/rolker/ros2_agent_workspace/issues/652)'s
decision 4: the project is per-clone configuration, not a property of the
GitHub repo, so a cloud runner or a fresh container is configured for no
project, and cloning manifests to find one would manufacture a project the
checkout does not have.

```bash
PROJECT_ROOT_STATUS=""
PROJECT_REPO_NAME=""
PROJECT_ROOT_PATH=""
PROJECT_REPO_AUDITED_THIS_RUN=""     # assigned in step 3, check 2

# manifest_fallback.sh was sourced in step 1 and its failure is terminal
# there, so manifest_bootstrap_identity is defined by the time this runs.
# Its only failure is 3 — no pointer, or a url form it cannot parse safely —
# and it prints nothing on stdout in that state.
if ! PROJECT_IDENTITY=$(manifest_bootstrap_identity "$ROOT" 2>/dev/null); then
    PROJECT_ROOT_STATUS="SKIPPED(no project configured on this checkout)"
else
    PROJECT_REPO_NAME=$(cut -f2 <<< "$PROJECT_IDENTITY")
    if [ -n "$PROJECT_REPO_NAME" ]; then
        for candidate in "$ROOT"/layers/main/*/src/"$PROJECT_REPO_NAME"; do
            [ -d "$candidate" ] || continue
            PROJECT_ROOT_PATH="$candidate"
            break
        done
    fi
    if [ -z "$PROJECT_ROOT_PATH" ]; then
        # No layers/ at all, or the pointer's repo is not checked out in it.
        PROJECT_ROOT_STATUS="SKIPPED(no project configured on this checkout)"
    elif ! PROBE_OUT=$(bash "$ROOT/.agent/scripts/planning_doc_probe.sh" "$PROJECT_ROOT_PATH" 2>/dev/null); then
        PROJECT_ROOT_STATUS="FAILED(project root probe: planning_doc_probe.sh could not read $PROJECT_REPO_NAME)"
    else
        ROADMAP_ROW=$(awk -F'\t' '$1=="roadmap"' <<< "$PROBE_OUT")
        case "$(cut -f2 <<< "$ROADMAP_ROW")" in
            present) PROJECT_ROOT_STATUS="configured: $PROJECT_REPO_NAME" ;;
            absent)  PROJECT_ROOT_STATUS="no-roadmap: $PROJECT_REPO_NAME" ;;
            *)       PROJECT_ROOT_STATUS="FAILED(project root probe: no roadmap row for $PROJECT_REPO_NAME)" ;;
        esac
    fi
fi
```

The `[ -n "$PROJECT_REPO_NAME" ]` guard is not belt-and-braces: an empty
repo field would leave the glob as `layers/main/*/src/`, which matches every
layer's `src` directory, and the first one would be adopted as "the project
root" and probed. `manifest_bootstrap_identity` cannot emit an empty repo
today (its safety regex requires at least one character), so this guard
never fires — which is exactly why it is cheap, and why the failure it
prevents would be silent and confident if the contract ever loosened.

**First glob wins, and that is unrecorded.** If two layers happen to hold a
checkout of the same repo name, this loop takes the first in glob order
(alphabetical by layer) and says nothing about the other. That is
deliberate — a duplicate repo name across layers is a workspace
misconfiguration `validate_workspace.py` owns, not something the per-project
report should adjudicate — but it does mean the report can name a checkout
the operator was not thinking of. Nothing downstream depends on which one
was picked beyond the roadmap probe.

**No project name appears anywhere in this skill** (ADR-0003):
`$PROJECT_REPO_NAME` comes from the tracked pointer, exactly as
`manifest_config_dir` already derives it for its own clone, so a fork of this
workspace resolves its own project.

The `ROADMAP.md` test goes through `planning_doc_probe.sh`'s **`roadmap`
row**, not a hand-rolled `[ -f ROADMAP.md ]` — the probe is the one reader of
the expected-location table, and `audit-project` already probes the same four
kinds through it. A probe that ran but emitted no `roadmap` row is a contract
violation, so it is `FAILED`; it is never quietly read as "no roadmap".

**Four terminal states, every one recorded** (step 8 states whichever
applies; none is silently swallowed):

| `$PROJECT_ROOT_STATUS` | Means |
|---|---|
| `SKIPPED(no project configured on this checkout)` | No pointer, a url form the derivation cannot parse, **or** no layer checkout of that repo on this host. One state, because the consequence is identical: no project this run can report on |
| `FAILED(project root probe: <reason>)` | The checkout is there but could not be probed |
| `no-roadmap: <repo>` | Probed fine, `ROADMAP.md` absent. Decision 3's "a project without a root roadmap gets what it gets today": its repos' findings still appear in the workspace run's `## Projects` section, and there is no second report. Absence of a planning document is never a finding |
| `configured: <repo>` | The per-project report proceeds — subject to the rotation condition below |

There is no `mode:` qualifier on these: the checkout is always a layer
checkout, because that is the only kind this step accepts.

**Rotation interaction.** The project root repo is audited by the ordinary
rotation like any other repo — only when it falls in this run's ISO-week
chunk (step 2). The per-project report is a *re-scoping* of findings the
`## Projects` section already computed for that repo (§ 6), so on a run where
the repo was not audited there is nothing to re-scope, and the report is not
written. Step 6 records **which** of the three reasons applied:
`SKIPPED(project repo excluded by rotation rule <n>: <reason>)` when step 2
excluded the repo from the rotation altogether (non-GitHub origin, no root
`AGENTS.md`, an inaccessible optional layer) — a state no later week will
change, so it names the remedy rather than a wait;
`SKIPPED(project repo not in this run's chunk)` when the repo is eligible but
this week's chunk did not select it — which also covers a project root listed
in no manifest at all, which the rotation therefore never sees — and
`SKIPPED(check 2 did not audit <repo>)` when it *was* selected but check 2
was itself `SKIPPED` or `FAILED` before auditing it. The third is a check
that did not run, not a quiet week, and saying the second there would read as
reassurance. Collapsing the first into the second would be worse still: it
would report a fixable misconfiguration as a scheduling accident. The prior per-project report's
findings carry forward unstated — a reader is pointed at that report's own
timestamp rather than at a diff synthesized against absent data.

### 2. Build the repo rotation

The rotation is built from the manifest **urls**, before anything is cloned —
so every rule below has to be answerable from a url plus a `gh` probe.

```bash
# --format json: the rotation needs each repo's url (for the origin check and
# the slug) and its source_file, not just the name.
# "${LIST_ARGS[@]}" is step 1's — empty on a host with its own configs/manifest,
# and `--config-dir <cloned manifest>` where there is none. Dropping it here
# enumerates ZERO repos on a no-`layers/` host, and rule 1 below then fires the
# one remedy lines above forbid.
python3 "$ROOT/.agent/scripts/list_overlay_repos.py" --format json "${LIST_ARGS[@]}"
```

Then, in order:

1. **If the list is empty, stop and report
   `FAILED(no repo manifest configured — run 'make setup-all')`.** This is a
   deliberate, named failure. `configs/manifest` is a symlink into the
   gitignored layer tree and absent in every fresh clone, and
   `list_overlay_repos.py` prints an empty list at **exit 0** in that state —
   rendering it as "no repos to audit" is exactly the false green this contract
   forbids. A non-zero exit from that script is
   `FAILED(manifest unreadable: <stderr>)` — also not an empty list. Where
   there is no `configs/manifest`, step 1's fallback has already added the
   cloned manifest's config dir, so an empty list here means nothing is
   configured at all.
2. Exclude repos whose origin is not on the GitHub allowlist, by URL:

   ```bash
   source "$ROOT/.agent/scripts/field_mode.sh"
   [ -n "$REPO_URL" ] || { echo "FAILED(manifest entry has no url)"; continue; }
   is_field_url "$REPO_URL" && echo "excluded: non-GitHub origin"
   ```

   Check the url is **non-empty** first: `is_field_url` classifies an empty or
   host-less url as dev mode by design (`field_mode.sh`'s header assigns that
   check to the caller), so an entry with no `url:` would otherwise be swept
   into the rotation and fail later, unexplained.

   Do not hand-roll the host list: `field_mode.sh` is the authoritative source
   (AGENTS.md § Field Mode, ADR-0011) and admits `ssh.github.com` — the
   SSH-over-443 fallback — alongside `github.com`. Use `is_field_url`, **not**
   `is_field_mode`: the latter reads a *checkout's* origin, and at this point
   in the sweep nothing is checked out — with no checkout it returns 1 ("dev
   mode"), which would silently *include* every gitcloud repo, the exact false
   green this rule exists to prevent. A gitcloud/Forgejo origin is listed as
   **excluded: non-GitHub origin, not reachable from a generic runner** —
   never silently dropped.
3. Probe the survivors for onboarding. The slug comes from the manifest URL
   (`<owner>/<repo>`), never from a hardcoded workspace name — this skill
   contains no repo slug of its own, so a fork of this workspace sweeps its own
   repos (ADR-0003).

   ```bash
   gh api "repos/$SLUG" >/dev/null                       # 1. visible at all?
   gh api "repos/$SLUG/contents/AGENTS.md" >/dev/null    # 2. then: onboarded?
   ```

   Both are probes, not reads — only their exit status is used, so both send
   their output to `/dev/null` (the second otherwise dumps base64 file JSON
   into the transcript on every onboarded repo).

   Both probes are needed, because a 404 on the second one alone means two
   different things — "this repo has no root `AGENTS.md`" and "this token
   cannot see this repo" — and publishing the second as the first states a
   check that never ran as a finding about the repo. So:
   - repo probe fails (404, auth, rate limit, network) → `FAILED(repo probe:
     <reason>)` for that repo. Not an exclusion — **except** for a repo from an
     **optional layer**, below.
   - repo visible, `AGENTS.md` 404 → **excluded: no root `AGENTS.md`**.
   - `AGENTS.md` probe fails for any other reason → `FAILED(onboarding probe:
     <reason>)`.

   **Optional layers are a supported host configuration, not a failure.**
   `configs/manifest/optional_layers.txt` lists layers a host is allowed not to
   have — typically private repos this host cannot see (`site` here).
   `setup_layers.sh` exits 0 without them and `validate_workspace.py` allows
   the same, so a repo-probe 404/auth failure for a repo whose `source_file` is
   that layer's `.repos` is **excluded: optional layer `<name>`, not accessible
   from this host** — not `FAILED`. Without this the same repo goes red on
   every sweep, forever, for a condition nobody intends to fix:

   Pass step 1's `$EXTRA_CONFIG` as well: `optional_layers.txt` lives at
   `configs/manifest/optional_layers.txt`, behind the same symlink as the
   `.repos` files, so on a host with no `layers/` the only copy is the one
   inside the cloned manifest. Without it this returns an **empty set** there
   and every inaccessible optional-layer repo is `FAILED` on every sweep —
   exactly what this rule exists to prevent.

   ```bash
   python3 -c 'import sys; sys.path.insert(0, sys.argv[1] + "/.agent/scripts/lib"); \
       from workspace import get_optional_layers; \
       extra = [d for d in sys.argv[2:] if d]; \
       print(" ".join(sorted(get_optional_layers(sys.argv[1], extra_config_dirs=extra))))' \
       "$ROOT" "${EXTRA_CONFIG:-}"
   # a repo from site.repos is in layer "site"
   ```

   A repo from an optional layer whose probes **succeed** stays in the rotation
   — the exclusion covers the inaccessible case only.

   What this gates on is **presence**, not currency: ADR-0017's currency signal
   is the `## Quality Standard` marker *inside* the file, which `audit-project`
   step 2 checks once the repo is in the rotation. A present-but-stale
   `AGENTS.md` is a finding for the audit to make, not a reason to skip the
   repo.
4. An empty survivor set has **two different causes, and they are not the same
   status**. If repos were enumerated and every one was *excluded* (a named,
   expected reason — non-GitHub origin, no root `AGENTS.md`, inaccessible
   optional layer), that is `SKIPPED(no eligible repos: <reasons>)`. If any
   candidate FAILED its probe, the rotation is **not** trustworthy and the
   project-governance check is `FAILED` — see check 2 below. In particular, an
   unauthenticated or rate-limited `gh` fails *every* probe, which would
   otherwise leave an empty chunk that reads as "0 repos audited, OK". Probe
   `gh auth status` once, before the per-repo probes, so the report can name
   the one cause instead of repeating it per repo.
5. Sort the survivors by name, chunk by 3, and select chunk
   `ISO-week mod chunk-count`. `date +%V` is **zero-padded**, and bash reads a
   leading zero as octal — `$(( 08 % 3 ))` is an error, twice a year — so force
   base 10:

   ```bash
   WEEK=$(date +%V)                      # e.g. "08"
   CHUNK_COUNT=$(( (N + 2) / 3 ))        # N = surviving candidates
   [ "$CHUNK_COUNT" -gt 0 ] || { echo "SKIPPED/FAILED per rule 4"; return; }
   CHUNK_INDEX=$(( 10#$WEEK % CHUNK_COUNT ))
   ```

   Once the chunk is selected, record whether step 1a's project root repo is
   in it — **here, where chunk membership is decided**, and independently of
   whether check 2 later manages to audit anything:

   ```bash
   # $CHUNK_REPOS: the selected chunk, one repo name per line.
   PROJECT_REPO_IN_CHUNK=""
   if [ -n "$PROJECT_REPO_NAME" ] && grep -qxF "$PROJECT_REPO_NAME" <<< "$CHUNK_REPOS"; then
       PROJECT_REPO_IN_CHUNK=1
   fi
   ```

   `$CHUNK_REPOS` is built from the **survivors**, so a project root that
   rules 2 or 3 excluded never reaches it — and "not in this run's chunk"
   would then point the operator at waiting for a later week when the remedy
   is a fix (onboard the repo, or accept that a non-GitHub origin is out of
   reach). So each exclusion also records itself when the repo being excluded
   is the project root, at the point the exclusion is decided:

   ```bash
   # In rules 2 and 3, beside each `excluded: …` the rule already emits:
   if [ "$REPO_NAME" = "${PROJECT_REPO_NAME:-}" ]; then
       PROJECT_REPO_EXCLUSION="excluded by rotation rule 2: non-GitHub origin"
   fi
   ```

   The reason text is the rule's own — `non-GitHub origin`, `no root
   AGENTS.md`, `optional layer <name>, not accessible from this host` — so the
   per-project line and the rotation's own listing read the same way.

   Step 6 needs three facts separately: *excluded, and why*
   (`$PROJECT_REPO_EXCLUSION`), *in the chunk* (`$PROJECT_REPO_IN_CHUNK`), and
   *actually audited* (`$PROJECT_REPO_AUDITED_THIS_RUN`, set in check 2). A
   repo that was in the chunk but whose audit never ran — because check 2
   itself was `SKIPPED` or `FAILED` — must not be reported as "not in this
   run's chunk", which is a different and reassuring statement; and a repo the
   rotation excluded outright must not be reported as either, which would hide
   a remediable state behind a wait.

The report always lists the **full** candidate set with each repo's in/out
status and reason, plus the chunk index and the ISO week used.

### 3. Run the four checks

Run each in turn and record its status per the contract above.

| # | Check | How | Layer-dependent? |
|---|---|---|---|
| 1 | Workspace governance | `cd "$ROOT"` first, then `/audit-workspace`, full | No |
| 2 | Project governance | `/audit-project <repo>` for each repo in the rotation chunk | No — `audit-project` resolves a clone when there is no layer checkout, and reports its two layer-dependent items as SKIPPED |
| 3 | Issue staleness | `/issue-triage --stale-days 90` | No |
| 4 | Research-digest freshness | Below | No |

Each check's FAILED evidence is named, because two of these are sub-skills that
report into the conversation rather than returning an exit code — "it seemed to
run" is not a status.

- **Check 1 — `audit-workspace`** is the one check that does not take a root:
  it addresses every input by bare relative path and so audits **the current
  directory**. `cd "$ROOT"` before invoking it, and stay there for its run.
  Without that, a sweep launched from a worktree — the skill's own normal case
  — grades that branch's governance docs for check 1 and the main root for
  checks 2-4, then reports the mixture as one workspace state. All four checks
  must grade the same tree, and `$ROOT` is that tree.

  It is `FAILED` when an input it needs cannot be
  read (`$ROOT/docs/PRINCIPLES.md`, `$ROOT/docs/decisions/`, `$ROOT/AGENTS.md`,
  `$ROOT/.agent/templates/`), or when its run ends without producing all seven
  checklist sections. A section it could not complete is `SKIPPED(<reason>)`
  inside the audit and makes the check `FINDINGS` at best, never `OK`. Probe
  the inputs before reporting the check's status; do not infer it from the
  narrative.

  **Relay its coverage, do not re-collect it.** `audit-workspace`'s report
  carries a `### Coverage` table, between its `### Summary` and `### Findings`
  sections — one row per checklist section, with the section's
  name, its `<kind>` token, `X of Y examined`
  (naming the items when X < Y) for the two sections that may sample and
  `all N` — or `M of N — <item>: <reason>` when a few items could not be
  read — for the five that never do
  ([#651](https://github.com/rolker/ros2_agent_workspace/issues/651)).
  Carry that table, folded into one line, into the check's `Detail` cell
  (§ Render the report) and hold it for the run-over-run diff (§ 5), which
  keys on it. A section with no coverage row counts as `0 of Y` for the
  diff — its findings still render, but nothing from a prior run can be
  `Resolved` against it — and the check is `FINDINGS` at best, with the
  missing row named in `Detail`. Coverage rows are the audit's claim about
  itself, not this skill's: relay what it reported.
- **Check 2 — `audit-project`** rolls up its per-repo runs, and the rollup
  covers the rotation as well as the audits: `FAILED` if **any candidate failed
  its rule-3 probe**, if any repo in the chunk failed to resolve (any non-zero
  exit from `resolve_repo_checkout.sh`), or if any repo failed to audit;
  otherwise `FINDINGS` if any repo produced findings; otherwise `OK`. The
  embedded Planning Documents table (see below) is descriptive only and never
  counts toward "produced findings" here — the same non-scoring rule
  `audit-project` itself states in its own § 7, tied back explicitly because
  this rollup is where a stray finding would actually change a repo's
  reported status. Per-repo statuses are listed individually regardless.

  **An empty chunk is never `OK`.** Audit-count zero has three causes and the
  check must say which: every candidate *excluded* → `SKIPPED(no eligible
  repos: <reasons>)`; any candidate *failed* → `FAILED(repo probe: <reason>)`,
  naming the shared cause once where there is one (`gh` unauthenticated,
  rate-limited, offline); a chunk that legitimately holds repos which all
  audited clean → `OK`, with a non-zero count. `Project governance | OK | 0
  repos audited` is a report this skill must never produce.

  **Note when the project root repo is the one being audited.** Step 1a
  resolved which repo that is; the per-project report (§ 6) is written only
  when this run actually audited it, so the flag is set here, in the one
  place that knows:

  ```bash
  # inside check 2's per-repo loop, after that repo's audit-project run
  [ -n "$PROJECT_REPO_NAME" ] && [ "$REPO" = "$PROJECT_REPO_NAME" ] \
      && PROJECT_REPO_AUDITED_THIS_RUN=1
  ```

  Set it whatever the repo's own audit status was: the per-project report
  re-scopes this run's findings for that repo, and a repo that audited
  `FAILED` has a finding-set worth reporting (its failure) exactly as one
  that audited clean does.

  **The loop must not be a pipeline.** `... | while read REPO; do ...; done`
  runs its body in a subshell, so this assignment — and any other state the
  loop accumulates for later steps — dies with it, and step 6 then reports
  `SKIPPED(check 2 did not audit <repo>)` for a repo it just audited. Drive
  the loop from a `for` over the chunk, or from a here-string
  (`while read REPO; do ...; done <<< "$CHUNK_REPOS"`), both of which keep
  the body in the current shell.

  **Relay each repo's coverage too.** `audit-project`'s per-repo report
  carries its own seven-row `### Coverage` table, near its top; carry it, folded into one clause
  per repo, into the check's `Detail` cell alongside the checkout mode
  (§ Render the report) and hold it for the run-over-run diff (§ 5), whose
  project-scope gate keys on it exactly as the workspace-scope gate keys on
  check 1's. A repo with no coverage table counts as unexamined for the diff:
  its findings still render, but nothing from a prior run can be `Resolved`
  against it.
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

- **Provisional decisions with no review scheduled** (project scope,
  report-only — riding check 2's per-repo rotation, not a fifth check of its
  own). For each repo in the chunk, read `<log_dir>/**/*.md` from that repo's
  `.agents/deployment.yaml` (`log_dir:` key,
  [`.agent/knowledge/deployment_mode.md`](../../../.agent/knowledge/deployment_mode.md)
  § Project-config schema) for a `## Decisions made this deployment` heading
  (the section `/wrap-up-deployment` is meant to write, per
  [#642](https://github.com/rolker/ros2_agent_workspace/issues/642) — **not
  shipped yet**, so no such heading exists anywhere in the repo today). This
  scan is tolerant by construction, since #642's actual output shape has never
  been observed:

  - No `.agents/deployment.yaml` in the repo → skip the repo silently, counted
    as "no deployment config" — never a failure, and never rendered as a
    finding.
  - `.agents/deployment.yaml` present but no `## Decisions made this
    deployment` heading found in any file under `<log_dir>` → nothing to
    report for that repo. This is a normal empty result, `OK`, never
    `FAILED` — absence is never a finding, the same rule the design draft
    states for every planning-document probe.
  - A heading found → parse entries loosely (decision text, date made, where
    recorded, review owed). An entry is **unscheduled** when its "review
    owed" text names no date and no forcing-function phrase (e.g. "at the
    next design-mode pass", "before the 2027 season", "when #391's vertical
    slice lands") — age it from its recorded date.

  Across **every** scanned repo, if no heading was ever found, the row renders
  "no provisional decisions found this run" — empty, `OK`. Where headings
  exist, unscheduled entries feed the **contradictions-in-the-record** tier
  (§ Classify findings into tiers) as a distinct labeled sub-list
  (`Provisional decisions with no review scheduled`), not merged into that
  tier's other findings — a later #642-driven refinement can retarget the
  tier without re-deriving the scan. This row is project scope: it is not
  part of the committed `docs/health.md` (the workspace repo has no project
  dev logs of its own).

### 4. Classify findings into tiers

Every finding from checks 1-4 (and the provisional-decisions scan above) is
classified into exactly **one** of five tiers, in this severity order:

| Tier | Covers |
|---|---|
| 1. Work that can be lost | Uncommitted/unpushed work flagged by `audit-workspace`/`audit-project`; stale worktrees |
| 2. Unowned safety bugs | `audit-project`'s Quality Standard gaps that name a safety-relevant failure mode (error handling, silent failure, missing validation) with **no tracking issue** |
| 3. Rules that have bitten with no enforcement | A documented rule with no CI/hook backing it — ADR-0005's own test; `audit-workspace`'s enforcement-gap section |
| 4. Contradictions in the record | `audit-workspace`/`audit-project` findings that two tracked documents disagree; the provisional-decisions-with-no-review-scheduled sub-list above |
| 5. Drift (catch-all/default) | Everything else — stale docs, digest freshness, stale issues. Any finding not matched by a more specific rule above defaults here, so the mapping never fails closed |

The mapping is **mechanical, not judgment-per-finding**: apply the table
above to each finding's *kind*, not to its content. This keeps tiering
reproducible run over run — the same finding kind always lands in the same
tier, which is what makes the run-over-run diff below meaningful (a finding
that moved tiers between runs would otherwise look like two different
findings).

Render findings grouped by tier within each scope section (§ Write the
report), replacing the flat per-check findings list a prior revision of this
skill used. A tier with no findings in a given run is omitted from that
scope's section, not rendered empty.

### 5. Run-over-run diff

For **each scope**, diff this run's findings against the previous run's, by a
stable key: `tier + check + one-line description`. Every tier in every scope
carries this diff state — there is no tier that goes undiffed by default.
The **rendering shape** differs by scope (both described fully in § Write
the report, so the two steps must be read together, not just this one): the
Workspace scope renders four subsections per tier — `New`, `Resolved`,
`Unchanged`, and the coverage-gated `Not re-examined` below — because it
diffs against one committed `docs/health.md`.
Project scope renders the same four states as a
`[New/Resolved/Unchanged/Not re-examined]` tag inline on each per-repo
finding, because it diffs many repos against
independent, best-effort local-report history, where per-tier subsections
spanning dozens of repos would be unreadable. The **states** are the same in
both scopes — the coverage gate below is the rule in both — and only the
rendering differs. The one stated exception is
the Projects section's "Provisional decisions" sub-list (§ Write the
report): it is not diffed, because the row's own wording already states
whether a decision has a review scheduled — a New/Resolved/Unchanged tag on
top would be redundant, not because diffing was skipped.

- **Workspace scope** — the previous run's state comes from **whichever
  source this run is diffing against, and the report says which** (§ Render
  the report's `**Diffed against**` line). The two branches:

  **When `--publish` was passed**: the last **committed** `docs/health.md`.
  Read it from the main checkout, `$ROOT` — the skill worktree does not exist
  yet (it is created in § Publish the workspace scope, 7a, after this step
  and the next) and is not needed for the read, since it branches from the
  same commit:

  ```bash
  if PREV_HEALTH=$(git -C "$ROOT" show HEAD:docs/health.md 2>/dev/null); then
      PREV_SOURCE="committed docs/health.md @ $(git -C "$ROOT" rev-parse --short HEAD)"
  else
      PREV_HEALTH=""
      PREV_SOURCE="none — first committed run, no prior health document"
  fi
  ```

  The source is named **only when the read succeeded**. `git show` failing
  (no `docs/health.md` in history — the normal first `--publish` run) leaves
  nothing to diff against, and a `**Diffed against**` line naming a commit
  in that state would attribute an empty prior set to a document that was
  never read.

  **When `--publish` was not passed (the default)**: the most recent prior
  **local** workspace-scope report — the `## Workspace` section of the
  newest `.agent/scratchpad/janitor/<ts>-<pid>-sweep.md` — read from the
  retention-pruned set (last 20 runs, step 1). This is the identical
  best-effort rule project scope has always used, applied to the other
  scope; the local report's `## Workspace` section is structurally the same
  bytes `docs/health.md` carries (§ 6 renders `$WORKSPACE_SECTION` once and
  composes both artifacts around it), so the prior-finding definition below
  reads correctly off either source:

  ```bash
  PREV_REPORT=$(ls -1t "$REPORT_DIR"/*-sweep.md 2>/dev/null | head -n 1)
  if [ -n "$PREV_REPORT" ]; then
      PREV_HEALTH=$(awk '/^## Workspace$/{f=1;next} /^## Projects$/{f=0} f' "$PREV_REPORT")
      PREV_SOURCE="local report $(basename "$PREV_REPORT")"
  else
      PREV_HEALTH=""
      PREV_SOURCE="none — first local run, no prior report"
  fi
  ```

  **Known limitation — two sweeps on one host can diff against each other.**
  `ls -1t … | head -n 1` picks the newest file in `$REPORT_DIR`, and on this
  branch that directory is the *live* one this run will also write into. Two
  sweeps running on the same host close enough together (same clone, or any
  two worktrees — `$REPORT_DIR` is anchored at the main root, step 1) can each
  pick up the other's report, including one still being written, and call it
  "previous". The committed-`docs/health.md` branch above cannot do this: it
  reads an immutable commit. This is named rather than fixed — no lock, no pid
  filtering — because the consequence is bounded: a wrong or partial prior set
  mis-labels findings as `New`/`Resolved` for one run, the report says which
  file it diffed against (`$PREV_SOURCE` carries the basename, pid included),
  and the next run is correct. A sweep is a periodic single-runner job, so
  making concurrent local runs exact is deliberately not attempted here; if it
  ever matters, it is a change to this rule, not a lock bolted onto it.

  `$PREV_SOURCE` always holds one of the template's own strings (§ Render the
  report's `**Diffed against**` line) — never the empty string, which would
  render as a blank line where a reader expects to be told what `Resolved`
  meant this run.

  There is exactly **one** "nothing to diff against" state on this branch,
  and it is said plainly rather than rendered as "no changes": **`no prior
  local report`** — the directory holds none at all, the first local run on
  this host. The Project-scope bullet below distinguishes a second state,
  `prior report predates retention`, and that one is **not reachable here**:
  workspace scope diffs against whichever report is newest, so retention
  pruning older runs is invisible to it — there is no particular prior run it
  could find missing. Project scope can tell the two apart because it looks
  for a report covering *a given repo*, which retention can prune out from
  under it while other reports remain.

  `$PREV_HEALTH` and `$PREV_SOURCE` are assigned exactly once, here, on
  whichever branch ran; nothing later re-reads or re-derives either. Note
  that a non-publishing run **never reads or rewrites the committed
  `docs/health.md`** — the baseline on `main` stays as whichever `--publish`
  run last wrote it (decision 2).

  **What in that file counts as "the previous run's findings"**: every
  finding listed under **any** of a tier's four subsections — `New`,
  `Resolved`, `Unchanged` **and `Not re-examined`** — minus the `Resolved`
  ones, which by definition were gone as of that run. So the prior-finding
  set is the union of the previous file's `New`, `Unchanged` and
  `Not re-examined` entries, across all tiers. Including `Not re-examined`
  is the load-bearing part: an entry carried forward verbatim because its
  section was not looked at is still an open finding, and dropping it from
  the prior set would make it render as `New` the next time the section *is*
  examined — resetting its age on every partial run and hiding exactly the
  long-lived gaps this state exists to keep visible. A `Not re-examined`
  entry that is still absent, from a section still not covered, stays
  `Not re-examined` for as many runs as that lasts.

  **A miss is `Resolved` only when this run looked** ([#651](https://github.com/rolker/ros2_agent_workspace/issues/651)).
  A prior finding absent from this run's findings used to render as
  `Resolved` unconditionally, so a shallow `audit-workspace` run — 2 of 10
  principles spot-checked, "no gaps found" — would have read as the prior
  run's four enforcement gaps being fixed. Before classifying an absent
  prior finding, read the coverage row (check 1's `### Coverage` table,
  relayed in § 3) of the audit **section the finding came from** (tier 3
  findings come from section 1, principles; ADR drift and Consequences-map
  findings from sections 2 and 5; tier 1 stale worktrees from section 7;
  script-table and template findings from sections 3 and 4;
  instruction-file/adapter-consistency findings from section 6 — all seven
  sections are named here, so no section falls through to the
  can't-determine bullet by omission):
  - Section fully covered this run (`all N`, or `X of Y` with `X == Y`) and
    the finding is absent → **`Resolved`**.
  - Section partially covered (`X of Y` with `X < Y`, whether a sample from
    sections 1–2 or a never-sampling section's `M of N — <item>: <reason>`
    shortfall) and the item the finding is about (a named principle, an ADR
    number, a named script or template) **was covered** — it is among the
    items the sampled section names as examined, or it is not among the
    items the shortfall names as unread — and the finding is absent →
    **`Resolved`**. Partial coverage does not block resolution of the thing
    actually re-examined.
  - Section partially covered and the item was **not** covered, or the
    section reported `0 of Y` / `SKIPPED`, or has no coverage row →
    **`Not re-examined`**. The prior finding is carried forward verbatim
    under that subsection, with a `(since <YYYY-MM-DD>)` stamp; nothing about
    it is known to have changed.
  - **A prior finding from check 4** (research-digest freshness) has no
    `audit-workspace` section and no coverage row in check 1's table — it is
    workspace scope and tier 5, but it comes from a different check, whose
    coverage is not sampled at all: check 4 reads the one digest file, whole,
    every run. So its gate is the check's own status, not a section row: an
    absent prior check-4 finding is **`Resolved`** when check 4 completed
    this run (`OK` or `FINDINGS`), and **`Not re-examined`** when check 4 is
    `SKIPPED` or `FAILED` — the digest was not read, so nothing about a prior
    digest finding is known to have changed. Do not route check-4 findings
    through the section bullets above or the can't-determine bullet below;
    neither applies to them.
  - A prior finding whose section cannot be determined from its text (the
    stable key carries the check but the section is inferred from the
    finding's kind) → `Not re-examined` when *any* of check 1's sections
    was less than fully covered this run, `Resolved` only when all seven
    were fully covered. Fail toward "not known to be fixed".

  The diff key itself (`tier + check + one-line description`) is unchanged;
  what changes is what a *miss* on that key resolves to when coverage is
  partial. `Unchanged` and `New` are unaffected: a finding present in both
  runs, or only in this one, is what it is regardless of coverage.

  **Every `Not re-examined` entry carries a `(since <YYYY-MM-DD>)` stamp**
  ([#651](https://github.com/rolker/ros2_agent_workspace/issues/651)) — the
  date of the health document that **first** carried this finding forward,
  not this run's date. When a finding becomes `Not re-examined` for the first
  time, stamp it with this run's date; on every later run that carries it
  forward again, copy the stamp from the previous file verbatim. The date is
  what makes age visible: sections 1–2 sample at the agent's discretion with
  no rotation mechanism, so an entry can park under this subsection
  indefinitely, and without a date one missed once and one missed twenty
  times render identically. A finding that leaves the subsection — because
  its section was covered and it was found again (`Unchanged`) or was gone
  (`Resolved`) — drops the stamp; if it later returns to `Not re-examined` it
  is stamped afresh. This is the whole mechanism: a date, not a rotation
  schedule or an escalation rule.

  **The gate is the rule in both scopes**, not a workspace-section
  peculiarity: a prior finding renders `Resolved` only when the section it
  came from was fully covered this run. Project scope applies the same gate
  against `audit-project`'s own `### Coverage` table — see the Project scope
  bullet below. What differs between the scopes is only the rendering shape
  (four subsections here, a fourth inline tag value there), never the rule.

  **Coverage is reportable, not self-verifying** — and this skill is where
  that matters most, because this is the step that turns a self-reported
  number into a *classification*. `audit-workspace` states the limit for
  itself (its own § Coverage: nothing stops a run from writing
  `principles: 10 of 10` without reading all ten); the consequence lands
  here. An inflated coverage row silently converts a real, unexamined gap
  into `Resolved` — the exact failure this state exists to prevent, arriving
  by a different door. Relay the number as given (it is the audit's claim,
  not this skill's), and treat a coverage row that is implausible against the
  audit's own narrative — `all N` from a section whose prose describes no
  work, a full pass from a run that reported nothing examined — as a
  `FINDINGS` condition on that check, named in `Detail`, not as a licence to
  resolve prior findings against it.

  **First run — nothing to diff against**, on either branch: under
  `--publish`, no prior `docs/health.md` in history (the normal case for a
  first publish, and for any repo where the file was never committed or was
  removed from the base branch) — `git show` fails; on the default path, no
  prior local report in the retention set. Either way `PREV_HEALTH` is empty
  and this is **not an error**. Every finding renders under `New` in every
  tier, with an explicit note in the report — "first committed run — no
  prior health document" under `--publish`, "first local run — no prior
  report" on the default path — rather than an empty `Resolved`/`Unchanged`
  subsection that would read as "nothing changed" on a run that had nothing
  to compare.
- **Project scope** — no committed history exists (project scope is still
  report-only, § Overview), so the diff is **best-effort** against the most
  recent prior **local** report under `.agent/scratchpad/janitor/` for the
  same repo, when one exists (the retention-pruned set from step 1, so a
  report older than the last 20 runs is no longer available for this — say so
  rather than silently diffing against nothing). When no prior local report
  covers this repo — first sweep ever, or the repo was not in a prior run's
  chunk — say so plainly (`no prior report for this repo` / `prior report
  covered a different chunk`) rather than rendering an empty diff as "no
  changes."

  **A miss is `Resolved` only when this run looked** — here too
  ([#651](https://github.com/rolker/ros2_agent_workspace/issues/651)). Read
  the gate off `audit-project`'s `### Coverage` table for the repo in hand
  (its own § Report Format), section by section, exactly as the workspace
  scope reads check 1's. `audit-project` never *samples* — every section
  enumerates a set it discovered — but that does not make its coverage always
  `N of N`. Its own lines
  admit `0 of 1 — <reason>` (an agent guide that exists but could not be
  read — § 3's partial form), `N-1 of N — <item>: <reason>` (an item that could not be read),
  `2 of 3 — layer: SKIPPED (no layer checkout)` (clone mode), `0 of 4 —
  SKIPPED(<reason>)` (the planning-document probe), and `N of N packages —
  run: M of N` (a partial `colcon test` pass). A repo audited in `layer` mode
  last run and in `clone` mode this run drops the layer check outright —
  resolving its prior finding would be precisely the "not looked at reads as
  fixed" failure this issue closes, in the scope where it is likeliest to
  happen, since checkout mode changes with the host and not with the code.

  **Section 3's two word-form tokens are coverage values like any other**
  ([#651](https://github.com/rolker/ros2_agent_workspace/issues/651)).
  `audit-project` § 3 reports `agent guide: checked` / `absent` /
  `0 of 1 — <reason>` rather than a count, because the set it enumerates is a
  single file:
  - **`checked`** is § 3's **full-coverage** form — the guide exists and all
    four of its questions were answered. It satisfies the full-coverage
    bullet below exactly as `all N` does. Read only against the numeric
    forms it matches none of them, and a § 3-sourced prior finding could
    then never resolve.
  - **`0 of 1 — <reason>`** is § 3's **partial** form — the guide exists but
    could not be read. It is the partial bullet's case, and it is the form
    that makes an incomplete § 3 pass visible.
  - **`absent`** is neither: no item exists to carry a finding about. A prior
    finding about the *content* of a guide that is now absent is
    `[Resolved]` only when this run reports § 2's "`.agents/README.md`
    missing" finding — the audit looked, and the guide's absence is what it
    found, so the content finding is moot. Without that § 2 finding present
    this run, nothing was established either way and the prior finding is
    `[Not re-examined]`.
  **Which `audit-project` section a tier's findings come from**, so no tier
  reaches the gate without a named source the way the workspace scope's
  all-seven mapping avoids
  ([#651](https://github.com/rolker/ros2_agent_workspace/issues/651)):
  - **Tier 1 (work that can be lost)** — no `audit-project` section
    produces these; the sweep's stale-worktree finding is workspace scope.
  - **Tier 2 (unowned safety bugs)** — § 4 (package metadata) and § 5 (test
    status): package-level quality gaps.
  - **Tier 3 (rules that have bitten with no enforcement)** — none;
    principles enforcement is audited only at workspace scope.
  - **Tier 4 (contradictions in the record)** — § 3: an agent guide whose
    package inventory or file paths disagree with the repo's actual
    `package.xml` files.
  - **Tier 5 (drift)** — § 2 (governance), § 6 (documentation) and § 8
    (workspace integration).
  - § 7 (planning documents) never contributes a finding: absence of a
    planning document is explicitly not a finding
    (`docs/design/planning_document_vocabulary.md`), so nothing from it
    enters the tiers or the diff.

  A tier with **no** section source (1 and 3) can still hold a prior finding
  — from an earlier run, or from a finding whose origin cannot be read off
  its text — and takes the conservative fallback below: `[Not re-examined]`
  unless every one of that repo's sections was fully covered this run.

  - Section fully covered this run (`all N`, `checked`, `X of Y` with
    `X == Y`, or `N of N packages` with no `— run: M of N` shortfall bearing
    on the finding) and the prior finding is absent → **`[Resolved]`**.
  - Section partially covered (`0 of 1`, `N-1 of N — <item>: <reason>`,
    `2 of 3 — layer: SKIPPED`, or `— run: M of N` where the finding came from
    the test run), reported `0 of Y` / `SKIPPED`, reported `absent` without
    the § 2 finding that establishes it, or carrying no coverage row
    at all, and the prior finding is absent → **`[Not re-examined]`**. The
    prior finding is carried forward verbatim, tagged
    `[Not re-examined since <YYYY-MM-DD>]` — the same stamp the workspace
    scope uses, and read from the prior local report when one already carries
    it. Nothing about the finding is known to have changed.
  - A prior finding whose originating section cannot be determined from its
    text → `[Not re-examined]` when *any* of that repo's sections was less
    than fully covered this run, `[Resolved]` only when all seven were fully
    covered. Fail toward "not known to be fixed", the same default the
    workspace scope uses.

  `Unchanged` and `New` are unaffected here as well: a finding present in both
  runs, or only in this one, is what it is regardless of coverage.

### 6. Render, redact, and write the report

This step is **one render pass that produces up to three artifacts**, and
every later step consumes them by the names below — nothing downstream re-renders,
re-extracts, or edits report content. Two earlier revisions of this skill
had the publish step read a variable that was never assigned, and had the
report template carry a field (the PR URL) that cannot be known when the
template is rendered; naming the data flow here is what rules both out.

| Variable | Rendered here as | Consumed by |
|---|---|---|
| `$WORKSPACE_SECTION` | the `## Workspace` section: check-status table (each row's `Detail` carrying that check's coverage), then findings by tier with `New`/`Resolved`/`Unchanged`/`Not re-examined` subsections | composed into both artifacts below |
| `$PROJECTS_SECTION` | the `## Projects` section, per the template | `$REPORT_BODY` only — project scope is never published (§ Overview) |
| `$HEALTH_BODY` | `# Workspace health — <ts>` title + a fixed provenance line + `$WORKSPACE_SECTION` — **exactly the bytes committed to `docs/health.md`**. **Rendered only under `--publish`** | 7b, written verbatim |
| `$REPORT_BODY` | the local report: `## Janitor Sweep — <ts>` header, `$WORKSPACE_SECTION`, `$PROJECTS_SECTION`, the retention footer | written to `$REPORT` in this step; 7h appends the publish outcome |
| `$PROJECT_HEALTH_BODY` | the optional per-project report: `$PROJECTS_SECTION`'s own findings **re-scoped to the project root repo**, re-headed as its own document. **Rendered only when step 1a said `configured:` and check 2 audited that repo this run** | written to `$PROJECT_REPORT` in this step; nothing else consumes it |

Render `$WORKSPACE_SECTION` once and compose both artifacts around it, so the
committed section and the local report's workspace section are the same
bytes by construction, not by a copy step that can drift.

```bash
TS=$(date '+%Y-%m-%d %H:%M %:z')          # one timestamp, used by both artifacts
HEALTH_BODY=""
if [ "$PUBLISH" = "1" ]; then
    HEALTH_BODY=$(printf '# Workspace health — %s\n\n%s\n\n%s\n' "$TS" \
        'Generated by the `janitor-sweep` skill under `--publish`, and replaced wholesale by each such run. The run'"'"'s full record, including project-scope findings, is the local report under `.agent/scratchpad/janitor/` on the host that ran it.' \
        "$WORKSPACE_SECTION")
fi
REPORT_BODY=$(printf '## Janitor Sweep — %s\n\n%s\n\n%s\n\n%s\n' "$TS" \
    "$HEADER_LINES" "$WORKSPACE_SECTION" "$PROJECTS_SECTION")
```

(`$HEADER_LINES` is the two-line **Checks** / **Repo rotation** block from
the template below; the retention footer follows `$PROJECTS_SECTION` in the
template and is part of it.)

**Redact both artifacts, once, before either is written.** The findings text
is synthesized from `audit-workspace` / `audit-project` output (a stale-worktree
finding embeds a path; a clone failure embeds a remote URL) that was never
routed through `redact.sh`. `docs/health.md` is committed to a **public** repo,
and the local report is written to disk and handed around. So this is a
code-level gate on the rendered content, not an authoring reminder:

```bash
[ "$PUBLISH" = "1" ] && HEALTH_BODY=$(redact_text "$HEALTH_BODY")
REPORT_BODY=$(redact_text "$REPORT_BODY")
```

**`$HEALTH_BODY` is not computed on a default run**, and nothing consumes it
there: step 7 does not run, and the local report carries the identical
`$WORKSPACE_SECTION` bytes already. Rendering it anyway would be a second
copy of the same content whose only distinguishing feature is a provenance
line claiming a commit that never happened.

`redact_text` and `$REDACT_PATH_PREFIXES` were set up in step 1 from the
run's `$ROOT` and `$HOME`, and that is the right prefix set here: everything
was rendered relative to the main checkout, before the skill worktree (7a)
exists at a different path. **Nothing is appended to `$HEALTH_BODY` after
this line.** The only later addition to the local report is 7h's publish
outcome, which passes through `redact_text` itself.

**What the committed file deliberately does not contain**: its own PR URL or
publish outcome. Neither is knowable when the content is rendered, and a
file that had to be rewritten after its own commit would need a second
commit or an amend on every run. The commit and the PR *are* the provenance
of `docs/health.md`; the outcome, PR URL included, is recorded in the local
report by 7h.

Now write the local report to a **timestamped** file:

```bash
REPORT="$REPORT_DIR/$(date '+%Y%m%dT%H%M%S')-$$-sweep.md"
```

Generate the timestamp with `date`; never hand-type one (AGENTS.md
§ Documentation Accuracy). A date-only name would let a second run the same day
overwrite the first run's report — the runs are the record, so they must
accumulate. Seconds are not enough on their own for the same reason: two runs
finishing within one second would collide, so the pid disambiguates them. The
name stays sortable, and `*-sweep.md` still matches it (the retention sweep in
step 1 globs on that suffix). **Both local reports use this same
`<ts>-<pid>-` prefix** — this one and step 6's optional per-project
`<ts>-<pid>-<repo>-health.md` — and the prose throughout names them that way,
pid included, so a reader matching a documented shape against a real
directory listing finds the same thing twice.

This write depends on neither network nor auth, so a sweep that reached step 6
produces its record wherever the filesystem is writable. It is the sweep's
durable output — and, being the durable output, the one place the sweep must
not assume success:

```bash
if ! printf '%s\n' "$REPORT_BODY" > "$REPORT"; then
    echo "FAILED(report write: could not write $(redact_text "$REPORT"))"
    # ... print the findings inline here, then stop. The run is over.
    exit 1
fi
```

A failed write (read-only filesystem, no space, an unwritable or absent
`$REPORT_DIR`, a container whose scratchpad volume is not mounted) is
`FAILED(report write: <reason>)` per the status contract: **say it in the
conversation, print the findings inline so the run is not lost with the file,
and never describe the sweep as clean**. Checks that completed are still
reported with their own statuses — the failure is the record, not the checks.

**The stop is part of the state, not a formality.** A report-write failure is
terminal for the run (§ The status contract, fifth state), and the snippet
above has to *say so* in code: nothing after this line may append to `$REPORT`
or report a path that never landed. On `main` the block ended at the `echo`
and nothing followed it that could be fooled, so the missing stop cost
nothing. This change makes it load-bearing — the per-project block below
appends its `## Project health` section to `$REPORT` and can record
`written: <path>` for its own file — so the `exit 1` is added here rather
than left implied by the prose.

#### The optional per-project health report

When step 1a resolved a project root **and** check 2 audited that repo this
run, write a third artifact beside the other two: a health document scoped to
that one project, `.agent/scratchpad/janitor/<ts>-<pid>-<repo>-health.md`. Health
follows the roadmap — the opt-in is the project root having a root
`ROADMAP.md`, nothing else; no declaration file and no schema
([#652](https://github.com/rolker/ros2_agent_workspace/issues/652),
decision 3). It is **local only**: this skill never commits it, and never
publishes it under `--publish` either, which gates the workspace scope alone.

**It is a re-scoping, never a second computation.** `$PROJECT_HEALTH_BODY` is
built from the *same* per-repo findings already rendered into
`$PROJECTS_SECTION` for that repo — the same tier sections, the same inline
`[New/Resolved/Unchanged/Not re-examined …]` tags, the same Planning
Documents sub-table, the same "no findings this run" and tier-omission
rules — filtered to that one repo and re-headed:

```markdown
# Project health — <repo> — <ts>

Generated by the `janitor-sweep` skill's optional per-project report (opt-in:
the project root has a root `ROADMAP.md`). Re-scopes this run's findings for
<repo> from the workspace run's `## Projects` section — not independently
computed. Full record (all repos, all tiers): that run's own local report,
`.agent/scratchpad/janitor/<sweep-report-file>`.

<the `## Projects` tier sections, filtered to this repo>
```

Re-scoping rather than recomputing is what makes this report inherit the
#651 coverage gate for free: its findings are the already-tagged lines, so a
prior finding reads `[Not re-examined]` here for exactly the reasons it does
there. A second, independently-computed pass could disagree with the
`## Projects` section about the same repo in the same run, which is a report
that argues with itself.

**Diff source**: this *is* project scope, narrowed to one repo, so it takes
project scope's existing rule verbatim (§ 5's Project-scope bullet) — the
most recent prior local report for that repo, best-effort, with `no prior
report for this repo` / `prior report covered a different chunk` said plainly
rather than rendered as "no changes". Since the body's findings are the ones
§ 5 already diffed, this report's diff state is **inherited from that
computation, not recomputed**.

**The per-project files are not themselves a diff source, and § 5's lookup
does not read them.** They are a re-scoped copy of a `## Projects` section
that already exists in full in the run's own `*-sweep.md` report — which is
what § 5 reads for this repo, as it did before this change, and which is
written on *every* run rather than only the ones where the project root
happened to fall in the chunk. Adding `*-<repo>-health.md` to that lookup
would put a second, narrower copy of the same findings in front of the
diff for no gain. So there is no new *lookup* logic; there is new
**retention** logic, because the shape does not match `*-sweep.md` and
nothing else would ever prune it — step 1 globs it separately, under its own
last-20 count (§ 1, Retention).

Redact `$PROJECT_HEALTH_BODY` before writing it, exactly as the other two
artifacts are redacted above — it is built from the same
unredacted-until-now findings text. **Redact it where it is written, not
before the guard**: `$PROJECT_HEALTH_BODY` is rendered only on the branch
that writes the report, so on every other path — no project configured, no
root roadmap, project repo excluded from the rotation, project repo not in
this run's chunk, which together are the common case — it was never assigned, and a `redact_text "$PROJECT_HEALTH_BODY"`
above the guard is an unbound read that kills the sweep under `set -u` after
its primary report has already landed. The redaction therefore sits inside
the write branch, immediately before the write it protects:

```bash
PROJECT_HEALTH_STATUS="$PROJECT_ROOT_STATUS"   # 1a's answer, unless we write
if [ "${PROJECT_ROOT_STATUS#configured:}" != "$PROJECT_ROOT_STATUS" ]; then
    if [ -z "${PROJECT_REPO_AUDITED_THIS_RUN:-}" ]; then
        # 1a found a project, but there are no findings for it to re-scope
        # this run. A report rendered from nothing would read as a project
        # with a clean bill of health. Which of the two reasons applies is
        # stated, never collapsed into the friendlier one.
        if [ -n "${PROJECT_REPO_IN_CHUNK:-}" ]; then
            # In the chunk (step 2), but check 2 never reached its audit —
            # check 2 was SKIPPED or FAILED — so the ## Projects section has
            # nothing for this repo either.
            PROJECT_HEALTH_STATUS="SKIPPED(check 2 did not audit $PROJECT_REPO_NAME)"
        elif [ -n "${PROJECT_REPO_EXCLUSION:-}" ]; then
            # Step 2 excluded the repo from the rotation: no week will select
            # it until the exclusion is fixed, so say which rule and why.
            PROJECT_HEALTH_STATUS="SKIPPED(project repo $PROJECT_REPO_EXCLUSION)"
        else
            PROJECT_HEALTH_STATUS="SKIPPED(project repo not in this run's chunk)"
        fi
    else
        PROJECT_HEALTH_BODY=$(redact_text "$PROJECT_HEALTH_BODY")
        PROJECT_REPORT="$REPORT_DIR/$(date '+%Y%m%dT%H%M%S')-$$-${PROJECT_REPO_NAME}-health.md"
        if ! printf '%s\n' "$PROJECT_HEALTH_BODY" > "$PROJECT_REPORT"; then
            PROJECT_HEALTH_STATUS="FAILED(project health write: could not write $(redact_text "$PROJECT_REPORT"))"
        else
            PROJECT_HEALTH_STATUS="written: $(redact_text "$PROJECT_REPORT")"
        fi
    fi
fi
printf '\n## Project health\n\n%s\n' "$(redact_text "$PROJECT_HEALTH_STATUS")" >> "$REPORT"
```

`$PROJECT_HEALTH_STATUS` is appended to the **primary** local report as its
own `## Project health` section, so a reader of that one file always sees
whether a per-project report was written, skipped, or failed — without
having to notice the absence of a second file. The same timestamp-plus-pid
naming rule as `$REPORT` applies, for the same reason: runs accumulate.

**The write failure is its own named state** (the seventh, § The status
contract): `FAILED(project health write: <reason>)`, never folded into
`FAILED(report write: …)` or into a publish failure. It is **not terminal**
— the primary report landed before this step ran — so the run continues and
step 8 reports it alongside everything else.

**Keep the prose free of host identity and absolute local paths** as you
write it — name files relative to the workspace root
(`.agent/knowledge/research_digest.md`, not
`/home/…/.agent/knowledge/research_digest.md`), and do not record the
hostname. The `redact_text` pass above is the backstop for content the
checks produced; it is not a licence to author paths into the prose.
Excluded repo *names* are fine — they are already in the tracked manifests.

Report format — **two top-level sections, `## Workspace` and `## Projects`**,
replacing the single flat `### Check Status` table a prior revision of this
skill used. Each carries its own check-status line and its findings grouped
by tier (§ Classify findings into tiers). Both scopes carry all four diff
states (§ Run-over-run diff); the Workspace section splits each tier into
`New` / `Resolved` / `Unchanged` / `Not re-examined` subsections, while the
Projects section tags each per-repo finding inline with one of the same four.
A tier is omitted from that scope's section entirely — rather than rendered
as an empty heading — **only when it has no entries in any of its
subsections**, carried-forward `Not re-examined` entries included
([#651](https://github.com/rolker/ros2_agent_workspace/issues/651)). "No
findings *this run*" is not the test; "nothing to show at all" is. A tier
whose only content is a `Not re-examined` entry is still rendered: that entry
is an open finding this run did not look at, and dropping its tier would make
it vanish from the report entirely — the exact silent vanishing this state
exists to prevent, arriving one level up. The committed 2026-09-21
`docs/health.md` renders a tier with zero current-run findings for precisely
this reason. Within a rendered workspace tier, `New`/`Resolved`/`Unchanged` are
always present (empty ones say why, as the first-run note does); the
`Not re-examined` subsection is rendered **only when it has entries**. That
is a rule specific to this subsection, new with it: on a full-coverage run
it is empty by design, and an always-present empty `Not re-examined` would
suggest partial coverage where there was none. The `## Workspace` section
carries no publish line: the outcome is appended to the end of this file by
7h as its own `## Publish outcome` section, once it is known.

**Two coverage grains, kept visibly apart.** The top-line **`Checks: X of 4
completed`** counts the sweep's four checks — did each *run*. The
`Detail` cell of **every** check row, in both sections, carries that check's
**own** coverage — how much of its input it *looked at*: for
`audit-workspace`, its seven-row `### Coverage` table folded into one line
(`principles 2 of 10 (names); ADRs 1 of 19 (0017); scripts all 58; templates
all 12; …`); for research-digest freshness, the one file read; for
`audit-project`, each audited repo's own seven-row `### Coverage` table,
folded the same way, one clause per repo, naming its checkout mode and every
section short of full — that is the data § 5's project-scope gate keys on, so
the rollup must show it rather than a bare repo count; for `issue-triage`,
how many of the scanned repos' issue lists were actually fetched. `4 of 4
completed` over `principles 2 of 10`, or over `integration 2 of 3 — layer:
SKIPPED`, is a complete sweep of a shallow look, and the report must let a
reader see both numbers.

```markdown
## Janitor Sweep — <YYYY-MM-DD HH:MM ±HH:MM>

**Checks**: X of 4 completed, Y skipped, Z failed
**Repo rotation**: chunk <i> of <n>, ISO week <YYYY-Www>

## Workspace

| Check | Status | Detail |
|---|---|---|
| Workspace governance (`audit-workspace`) | OK / FINDINGS / SKIPPED(...) / FAILED(...) | <findings summary>. Coverage: principles X of Y (<names when X<Y>); ADRs X of Y (<numbers when X<Y>); scripts all N; templates all N; consequences-map items all N; adapters all 3; worktrees all N |
| Research-digest freshness | ... | last updated <date>, <n> days. Coverage: the one digest file, read |

**Diffed against**: <one of — `committed docs/health.md @ <short-sha>` (the
`--publish` branch, a prior document read) / `none — first committed run, no
prior health document` (the `--publish` branch, `git show` found none) /
`local report <ts>-<pid>-sweep.md` (the default branch, a prior report
found) / `none — first local run, no prior report` (the default branch,
none in the directory) >

<!-- Rendered on every run, directly under the check table (§ 5). The two
     branches diff against different sources, and which one a reader is
     looking at changes what "Resolved" means — so the source is stated,
     never inferred from whether a PR exists. -->

<!-- First committed run for this repo (git show HEAD:docs/health.md fails,
     or docs/health.md has never been committed): say so explicitly here —
     "none — first committed run, no prior health document", the string
     § 5's --publish branch assigns to $PREV_SOURCE — rather than
     rendering empty Resolved/Unchanged subsections that would read as
     "nothing changed". -->

### 1. Work that can be lost
#### New
- ...
#### Resolved
- ...
#### Unchanged
- ...
#### Not re-examined
- <prior finding, verbatim> (since <YYYY-MM-DD>)
  <!-- Only when non-empty (§ 5). The `since` date is the date of the health
       document that FIRST carried this finding forward, copied verbatim on
       every later run — not this run's date (§ 5). Tier 1's findings come
       from section 7, stale worktrees, which never samples — so this
       subsection is almost always empty here; see tier 3 below for the case
       it is actually for. -->

### 2. Unowned safety bugs
(same New/Resolved/Unchanged/Not re-examined shape as tier 1 — the shorthand
below stands for those four subsections in full, under the § 6 rules above:
`New`/`Resolved`/`Unchanged` always present, `Not re-examined` only when it
has entries, and the whole tier omitted only when all four are empty)

### 3. Rules that have bitten with no enforcement
#### New
- ...
#### Resolved
- ...
#### Unchanged
- ...
#### Not re-examined
- <prior finding, verbatim> (since <YYYY-MM-DD>)
  <!-- The state's home tier: these findings come from section 1
       (principles), one of the two sections that may sample. Example: the
       prior run found "principle X has no enforcement"; this run examined
       2 of 10 principles and X was not among them, so the finding is absent
       from this run's findings but is NOT resolved — it is carried forward
       verbatim here, stamped with the date it was first carried forward.
       Absence from this subsection is not resolution; presence is not a new
       finding. Rendered only when non-empty. -->

### 4. Contradictions in the record
(same shape)

### 5. Drift
(same shape)

## Projects

| Check | Status | Detail |
|---|---|---|
| Project governance (`audit-project`) | ... | <n> repos audited (<n> layer, <n> clone). Coverage: <repo> governance all 6, agent guide checked, packages N of N, tests N of N (run: M of N), docs all 4, planning 4 of 4, integration 2 of 3 — layer: SKIPPED; <repo> ... — one clause per repo, naming every section short of full |
| Issue staleness (`issue-triage`) | ... | <n> repos scanned. Coverage: <n> of <n> repos' issue lists fetched |

**Publish**: report-only, not committed (project health-rollup shape not yet
decided — § Deferred: the project-scope rollup and the trigger)

Every tier below carries the same diff-state tag per finding —
`[New/Resolved/Unchanged/Not re-examined since <YYYY-MM-DD>, or "no prior
report for this repo" / "prior report covered a different chunk"]` — per § Run-over-run diff's "for
**each scope**" mandate. Project scope renders it inline per finding
(`- **<repo>** [tag]: ...`) rather than as subsections, because unlike
the workspace scope's single committed `docs/health.md`, project-scope
findings span many repos with independent, best-effort local-report history —
a per-finding tag is legible where a per-tier subsection split across dozens
of repos would not be. This is a deliberate shape difference from the
Workspace section above, not an omission: every tier still carries run-over-run
information, and the same four states, just in this scope's own shape.
`[Not re-examined]` is the coverage-gated state (§ Run-over-run diff), keyed
on the repo's own `audit-project` `### Coverage` table — most often a repo
audited in `clone` mode this run whose prior finding came from a check that
mode skips.

### 1. Work that can be lost
- **<repo>** [New/Resolved/Unchanged/Not re-examined since <YYYY-MM-DD>, or
  "no prior report for this repo" / "prior report covered a different
  chunk"]: ...

### 2. Unowned safety bugs
- **<repo>** [New/Resolved/Unchanged/Not re-examined since <YYYY-MM-DD>, or
  "no prior report for this repo" / "prior report covered a different
  chunk"]: ...

### 3. Rules that have bitten with no enforcement
- **<repo>** [New/Resolved/Unchanged/Not re-examined since <YYYY-MM-DD>, or
  "no prior report for this repo" / "prior report covered a different
  chunk"]: ...

### 4. Contradictions in the record
- **<repo>** [New/Resolved/Unchanged/Not re-examined since <YYYY-MM-DD>, or
  "no prior report for this repo" / "prior report covered a different
  chunk"]: ...

**Provisional decisions with no review scheduled** (distinct labeled
sub-list, not merged into the tier's other findings — not diffed: a
provisional decision either still has no review scheduled or it doesn't,
which is what the row itself says, so a New/Resolved/Unchanged tag on top
would be redundant):
- **<repo>**: <decision text>, made <date>, recorded in <where>, review owed:
  <text> — unscheduled, <n> days since decision
- (or, if no repo in this run's chunk has any `## Decisions made this
  deployment` heading): "no provisional decisions found this run"

### 5. Drift
- **<repo>** [New/Resolved/Unchanged/Not re-examined since <YYYY-MM-DD>, or
  "no prior report for this repo" / "prior report covered a different
  chunk"]: ...

#### Planning Documents — <repo> (mode: layer/clone)

(embedded from `audit-project`'s own report section — no separate call site
here; see `audit-project` § 7. Descriptive only, never counted as a finding
or diffed — same non-scoring rule `audit-project` itself states. If
`audit-project` reported the probe `SKIPPED`, embed that single note instead
of the table below — there is no per-kind data from a probe that did not
run):

| Kind | Status | Location |
|---|---|---|
| vision | Present / Not found | `README.md` (Present) / — (Not found) |
| roadmap | Present / Not found | `ROADMAP.md` (Present) / — (Not found) |
| decision | Present / Not found | `docs/decisions` (Present) / — (Not found) |
| health | Present / Not found | `docs/health.md` (Present) / — (Not found) |

<!-- Location matches what planning_doc_probe.sh actually emits: the kind's
     path on Present, an EMPTY field on Not found. Render the empty field as
     "—", not as the expected path repeated. -->

<!-- The mode is not decoration: `clone` means the manifests chose the tree
     (their url, their pinned version), `layer` means the operator's working
     tree was audited as is, with its origin and checked-out ref deliberately
     unverified against the manifests. A reader deciding whether a finding
     applies to the pinned code or to this host's code needs that. -->

### Repos not audited this run

| Repo | Reason |
|---|---|
| <repo> | excluded: non-GitHub origin |
| <repo> | excluded: no root AGENTS.md |
| <repo> | excluded: optional layer `site`, not accessible from this host |
| <repo> | FAILED(repo probe: gh unauthenticated) |
| <repo> | not in this week's chunk |

---
Reports here are kept for the last 20 of each shape (`*-sweep.md`, and
`*-health.md` where a per-project report is written). The shallow clones under
`.agent/scratchpad/janitor-repos/` (<size>) and
`.agent/scratchpad/manifest-repo/` (<size>) are caches, never pruned by this
skill and the only output here that grows without bound. Deleting either
directory whole is safe while no sweep or audit is running on this host — they
are re-cloned on demand. Do not delete one mid-run: a sweep or audit reads its
checkout unlocked, and would report a phantom finding on the tree that vanished
under it.
```

Fill the **Checks** line from the status table, not from impression. If any
check is `SKIPPED` or `FAILED`, the report may not describe the workspace as
clean. The `## Workspace` section, **including its `New`/`Resolved`/`Unchanged`/
`Not re-examined` run-over-run-diff subsections**, is what `$HEALTH_BODY`
carries into `docs/health.md` in the next step **when `--publish` was
passed**: those subsections are exactly the run-over-run record the issue
asked for. Committing the diff sections (not stripping them) is the
deliberate choice there. On a default run the same section is the run's
record in the local report, and the run-over-run history is the sequence of
retained local reports rather than `docs/health.md`'s git history — which is
why step 1 retains the last 20 rather than one.

### 7. Publish the workspace scope

**7a–7g run only when `--publish` was passed.** On a default run none of
them happen — no worktree, no commit, no push, no PR, no GitHub call of any
kind. **7h always runs**, on both paths: it writes one of two headings to the
local report, so the report states the publish state explicitly rather than
leaving a reader to infer it from an absent section. The sweep is
local-first: the local report from step 6 is the run's record, and it has
already landed by the time control reaches here.

**Workspace scope only** — never commit `docs/health.md` into a project repo
in this slice; project-scope findings stay in the local
`.agent/scratchpad/janitor/<ts>-<pid>-sweep.md` report exactly as before this
change. This step runs after step 6 wrote the local report, and only when
check 1 and check 4 both reached a terminal status
(`OK`/`FINDINGS`/`SKIPPED`/`FAILED` — a crash mid-check never reaches here).

**Concurrency**: this whole step assumes **only one sweep publishes at a
time** — two concurrent runs would race on 7e's list/close/delete against
each other's GitHub state (each seeing the other's just-opened PR as "old"
and closing it), and on 7a's orphan removal against each other's worktree.
Nothing in this step serialises that; the invariant must be enforced by
whatever triggers a sweep. Wiring the weekly trigger is
[#636](https://github.com/rolker/ros2_agent_workspace/issues/636)'s job — it
must ensure a scheduled run and a hand-run (or two hand-runs) never publish
concurrently, e.g. by locking or by refusing to start a second publish while
one is in flight.

**Inputs this step consumes, and nothing else**: `$HEALTH_BODY` (step 6,
already redacted), `$REPORT` (step 6, the local report path), `$ROOT`
(step 1). **Values this step captures and every later sub-step reuses**:
`$WT_PATH` and `$NEW_BRANCH` (7a), `$NEW_PR_URL` (7d), and
`$PUBLISH_LINE` — the one-line outcome, assigned by whichever sub-step
**ends the publish attempt**: each failure exit in 7a–7d assigns it and
skips to 7h, and 7d assigns it on success, so by 7h it is set on every
path. Only 7h and step 8 read it. Cleanup and
replacement key on these captured values — never on a glob or a
"most recent match" lookup, which is how an earlier revision's cleanup
command could delete the live run's worktree instead of the orphan.

The sub-steps are labeled **7a–7h** (not 1–8) so they cannot be confused
with the document's top-level numbered steps.

7a. **Remove orphans, then create this run's worktree.** A run that
   completes removes its own worktree in 7g, so any `skill-*-janitor-sweep-*`
   worktree present now is left over from a run that failed partway (and,
   by the concurrency invariant, no other run is in flight). Remove each by
   its **exact path**, from the main checkout:

   ```bash
   cd "$ROOT"
   for wt in "$ROOT"/.workspace-worktrees/skill-*-janitor-sweep-*; do
       [ -d "$wt" ] || continue
       br=$(git -C "$wt" branch --show-current)
       echo "note: removing orphaned sweep worktree ${wt#"$ROOT"/}${br:+ (branch $br)}"
       if git worktree remove --force "$wt"; then
           if [ -n "$br" ] && ! git branch -D "$br"; then
               echo "note: could not delete local branch $br — left in place"
           fi
       else
           echo "note: could not remove ${wt#"$ROOT"/} — left in place"
       fi
   done
   ```

   An orphan's *remote* branch, if it was ever pushed, is handled in 7e.
   A failed run's worktree is therefore inspectable until the next
   **publishing** run starts — step 7 is the only caller of this cleanup,
   and it runs only under `--publish`
   ([#652](https://github.com/rolker/ros2_agent_workspace/issues/652)). Where
   `--publish` is occasional, an orphaned worktree (and the stray branch 7e
   would reap) can outlive many default runs, holding a checkout and a local
   branch the whole time. That is the cost of making publish opt-in, and it
   is stated rather than discovered: inspect an orphan while it lasts, or
   remove it by hand with the same two commands — a default run will not do
   it for you. `worktree_remove.sh --skill janitor-sweep` is **not** the
   tool for this: `find_worktree_by_skill` (`_worktree_helpers.sh`) picks
   the newest match when several exist, with only a stderr warning.

   Then create this run's worktree (`janitor-sweep` is in
   `.agent/scripts/worktree_create.sh`'s `ALLOWED_SKILLS`):

   ```bash
   if ! CREATE_ERR=$(.agent/scripts/worktree_create.sh --skill janitor-sweep --type workspace 2>&1); then
       PUBLISH_LINE="FAILED(workspace publish: worktree create: $CREATE_ERR)"
       # -> skip to 7h
   fi
   source .agent/scripts/worktree_enter.sh --skill janitor-sweep
   WT_PATH="$WORKTREE_ROOT"                   # exported by worktree_enter.sh
   NEW_BRANCH=$(git branch --show-current)    # skill/janitor-sweep-<ts>-<nano>
   ```

   `worktree_enter.sh` `cd`s into the worktree and exports `WORKTREE_ROOT`;
   it locates the worktree by the same newest-match lookup, which is safe
   here **only because the loop above just removed every other match** —
   that ordering is the reason the orphan removal comes first. Both captured
   values are what 7g uses; do not re-discover them later. If creation
   fails, `$PUBLISH_LINE` is `FAILED(workspace publish: worktree create:
   <reason>)`; skip to 7h.

7b. **Write `docs/health.md`** — `$HEALTH_BODY` verbatim, at the repo root
   of the worktree (the path the design draft's expected-location table
   fixes), **replaced wholesale each run** (the "health" kind's definition
   in the design draft), never appended to. It was redacted in step 6 and
   nothing has been added to it since:

   ```bash
   if ! { mkdir -p docs && printf '%s\n' "$HEALTH_BODY" > docs/health.md; }; then
       PUBLISH_LINE="FAILED(workspace publish: health write: could not write docs/health.md)"
       # -> skip to 7h
   fi
   ```

   A write failure here is named as its own cause (`health write`), not
   left to surface one sub-step later as a commit with nothing staged.
   The trailing `\n` is load-bearing: the first live run wrote the file
   without one, `end-of-file-fixer` rewrote it during the commit hook, and
   7c reported `FAILED(workspace publish: commit: …)` for a file that was
   correct in every other way (#649).

7c. **Commit** with per-invocation identity (AGENTS.md § Agent Commit
   Identity): for this PR's hand-run testing, the **implementing agent's own
   identity**, not `Janitor Sweep Agent` — standing up that bot identity is
   [#636](https://github.com/rolker/ros2_agent_workspace/issues/636)'s job,
   not this one's.

   ```bash
   git add docs/health.md
   if ! COMMIT_ERR=$(git -c user.name="$AGENT_NAME" -c user.email="$AGENT_EMAIL" \
           commit -m "Janitor sweep: workspace health $(date '+%Y-%m-%d')" 2>&1); then
       PUBLISH_LINE="FAILED(workspace publish: commit: $COMMIT_ERR)"
       # -> skip to 7h
   fi
   ```

   A commit failure (typically a pre-commit hook rejection) sets
   `$PUBLISH_LINE` to `FAILED(workspace publish: commit: <reason>)`; skip to 7h. The worktree
   stays for inspection until the next run's 7a.

7d. **Push and open a non-draft PR — before touching any prior PR.**
   Copilot code review does not review draft PRs, so open it non-draft (the
   default for `gh pr create` without `--draft`) so the review actually
   fires. Build the PR body with the usual `mktemp` + heredoc pattern
   (AGENTS.md § Use `--body-file`, Not `--body`):

   ```bash
   BODY_FILE=$(mktemp /tmp/gh_body.XXXXXX.md)
   cat << 'EOF' > "$BODY_FILE"
   Checks: X of 4 completed.
   Full record (workspace and project findings): .agent/scratchpad/janitor/<report-file>
   EOF

   if ! PUSH_ERR=$(git push -u origin HEAD 2>&1); then
       PUBLISH_LINE="FAILED(workspace publish: push: $PUSH_ERR)"
       # -> skip to 7h
   elif ! NEW_PR_URL=$(gh pr create --title "Janitor sweep: workspace health $(date '+%Y-%m-%d')" \
                           --body-file "$BODY_FILE" 2>&1); then
       PUBLISH_LINE="FAILED(workspace publish: pr create: $NEW_PR_URL)"; NEW_PR_URL=""
       # -> skip to 7h
   else
       PUBLISH_LINE="committed to docs/health.md — PR $NEW_PR_URL"
   fi
   rm -f "$BODY_FILE"
   ```

   `gh pr create` prints the new PR's URL to stdout on success; `$NEW_PR_URL`
   is captured from that, so there is no placeholder anywhere to fill in
   later. Title format: `Janitor sweep: workspace health <YYYY-MM-DD>`. The
   body states the checks line (`X of 4 completed` — labelled `Checks`, not
   `Coverage`, since per-check coverage is a different grain and lives in
   the health doc's `Detail` cells, § 6) and names the local
   report, by workspace-relative path, for the full record (workspace
   **and** project findings — `docs/health.md` carries only the workspace
   half).

   **If this sub-step fails** (push rejected, or `gh pr create` errors):
   `$PUBLISH_LINE` is `FAILED(workspace publish: <reason>)`; skip to 7h.
   On success `$PUBLISH_LINE` is assigned here, immediately — 7e and 7g can
   only add notes, never change the outcome. Do
   **not** run 7e: any prior `skill/janitor-sweep-*` PR is left untouched
   and stays open, the local report from step 6 already carries this run's
   findings, and the worktree stays for inspection until the next run's 7a.
   If the push succeeded but `gh pr create` did not, the pushed branch with
   no PR is cleaned up by the **next publishing** run's 7e (its stray-branch
   loop) — nothing on origin is stranded permanently, but "next run" here
   means the next `--publish` one, which may be many default runs away
   (7a above). The branch is inert in the meantime; it is not a leak, just a
   slower reap.

7e. **Replace, don't stack, prior sweep PRs and branches — only after 7d's
   new PR exists.** List open PRs whose head matches the prefix, excluding
   this run's branch, and close each with a comment naming the new PR.
   **The branch is deleted only after `gh pr close` itself reports success**
   — a close failure leaves the old PR and its branch untouched, rather than
   risking an open PR pointing at a deleted head, which GitHub does not
   auto-close and which then has to be cleaned up by hand:

   ```bash
   gh pr list --state open --json number,headRefName \
       --jq '.[] | select(.headRefName | startswith("skill/janitor-sweep-")) | "\(.number)\t\(.headRefName)"' \
   | while IFS=$'\t' read -r OLD_PR OLD_BRANCH; do
       [ "$OLD_BRANCH" = "$NEW_BRANCH" ] && continue   # this run's own PR
       if ! gh pr comment "$OLD_PR" --body "Superseded by this run's sweep PR: $NEW_PR_URL."; then
           echo "note: could not comment on old PR #$OLD_PR — closing anyway"
       fi
       if ! CLOSE_ERR=$(gh pr close "$OLD_PR" 2>&1); then
           echo "note: could not close old PR #$OLD_PR: $(redact_text "$CLOSE_ERR") — branch $OLD_BRANCH left in place"
           continue
       fi
       if ! DELETE_ERR=$(git push origin --delete "$OLD_BRANCH" 2>&1); then
           # Surface, don't swallow: "already gone" is benign, an auth error
           # is not; either way the PR is closed, so a leftover branch
           # strands nothing (merge_pr.sh's own delete-is-best-effort rule).
           echo "note: could not delete branch $OLD_BRANCH: $(redact_text "$DELETE_ERR")"
       fi
   done
   ```

   Then the stray branches — pushed by a run whose `gh pr create` failed
   (7d), so no PR points at them and the loop above never saw them:

   ```bash
   OPEN_HEADS=$(gh pr list --state open --json headRefName --jq '.[].headRefName')
   git ls-remote --heads origin 'skill/janitor-sweep-*' | awk '{print $2}' | sed 's#^refs/heads/##' \
   | while read -r BR; do
       [ "$BR" = "$NEW_BRANCH" ] && continue
       grep -qxF "$BR" <<< "$OPEN_HEADS" && continue     # has an open PR: handled above, or left in place
       if ! DELETE_ERR=$(git push origin --delete "$BR" 2>&1); then
           echo "note: could not delete stray branch $BR: $(redact_text "$DELETE_ERR")"
       fi
   done
   ```

   This is the load-bearing ordering rule: **push-and-open the new PR first
   (7d), close-and-delete the old one second (7e) — never the reverse, and
   never "either order."** If 7d failed after an old PR had already been
   closed, the previously-published record would be gone with nothing left
   open and no recovery path; running 7d first means a failure there always
   leaves the old PR intact. The same reasoning applies inside 7e: a branch
   is deleted only once its PR is confirmed closed, so the delete is gated on
   `gh pr close`'s own exit status.

7f. **A human still merges** (AGENTS.md § Merging, "green CI is not review").
   This step opens the PR; it never merges it.

7g. **Remove this run's worktree.** The branch is on origin and the PR is
   open, so the worktree has nothing left to do — and removing it here is
   what makes 7a's rule ("anything present at start is an orphan") sound.
   Key on the values captured in 7a, never on a lookup:

   ```bash
   cd "$ROOT"
   if git worktree remove "$WT_PATH"; then
       git branch -D "$NEW_BRANCH"        # local copy only; origin's is the PR head
   else
       echo "note: could not remove ${WT_PATH#"$ROOT"/} — the next run's 7a will"
   fi
   ```

   A failure here is a note, not a publish failure: the PR is already open.

7h. **Append the publish outcome to the local report** — on **every** path:
   every exit from 7a–7g, success or failure, *and* the default path where
   7a–7g did not run at all. This is where the PR URL lives; it
   is the one place the outcome is recorded, and it is appended after the
   file's step-6 content rather than rendered into it, because it is not
   known until now:

   ```bash
   if [ "$PUBLISH" = "1" ]; then
       # $PUBLISH_LINE was assigned by the sub-step that ended the attempt
       # (7a/7b/7c/7d on failure, 7d on success) and is one of:
       #   committed to docs/health.md — PR <url>
       #   FAILED(workspace publish: <reason>)
       printf '\n## Publish outcome\n\n%s\n' "$(redact_text "$PUBLISH_LINE")" >> "$REPORT"
   else
       printf '\n## Publish outcome: not requested (--publish off)\n' >> "$REPORT"
   fi
   ```

   On the default path the **heading itself carries the reason**, so a reader
   scanning headings sees the state without opening the section and there is
   no body line to write. `$PUBLISH_LINE` is never assigned on that path —
   7a–7g did not run, so there is no outcome to report, and inventing one
   ("not published") would be a sentence about a step that never happened.

   The reason string of a failure can carry a path or a remote URL, hence
   `redact_text`. The same line is what step 8 states to the operator.

**Failure is a named, separate state** (the sixth state, § The status
contract) — on a run that asked to publish. A worktree that could not be
created (7a), a commit that failed (7c), a push that was rejected or a PR
that could not be opened (7d) is
reported as `FAILED(workspace publish: <reason>)`, next to — not instead of —
the local report's own status, and recorded in the local report by 7h. The
local report has already landed by the time this step runs (step 6 precedes
it), so a publish failure never loses the run's findings; it only means they
are not yet committed.

### 8. Report to the operator

Summarise in the conversation and name the report file by its path **relative
to the workspace root** (`.agent/scratchpad/janitor/<file>`). Lead with the
checks line — `X of 4 completed` — so a partial sweep cannot read as a clean
one, and follow it with `audit-workspace`'s coverage line for the two
sections that may sample (`principles X of Y; ADRs X of Y`) so a shallow
audit cannot either; name any `Not re-examined` entries the diff produced.
If the local report write failed, lead with that instead and print the
findings inline. State the workspace-scope publish outcome explicitly, using whichever
heading 7h wrote: the PR URL on success, `FAILED(workspace publish:
<reason>)` on failure, or `not requested (--publish off)` on a default run —
and say which source the workspace scope diffed against (§ 5's
`$PREV_SOURCE`), since that decides what `Resolved` meant this run. State
plainly that project-scope findings are **not** published anywhere beyond
the local report(s).

State the **per-project health outcome** on its own line, from
`$PROJECT_HEALTH_STATUS` (§ 6), in the words the run produced:

- `no project configured on this checkout` — step 1a found no pointer, or no
  layer checkout of the repo it names. Nothing is wrong
- `<repo> has no root ROADMAP.md — no per-project report`
- `<repo>: excluded by rotation rule <n>: <reason> — no per-project report`
  (no week will select it until the exclusion is fixed)
- `<repo>: not in this run's chunk — no per-project report`
- `<repo>: check 2 did not audit it — no per-project report` (it *was* in the
  chunk; check 2's own status says why)
- `<repo>: FAILED(project root probe: <reason>)` — the checkout is there but
  `planning_doc_probe.sh` could not read it, so whether it has a roadmap is
  unknown. Never rendered as "no roadmap"
- `<repo>: <report path, workspace-relative>`
- `<repo>: FAILED(project health write: <reason>)`

Name the file by its path relative to the workspace root, as with the
primary report. A failure here never changes the headline: the primary
report already landed.

## Known limitations

Deliberate, and each is for the deferred decision below to close — they are
listed here so that decision meets them rather than rediscovering them.

- **The local report is host-local and ephemeral — on a default run, that is
  all there is.** `$REPORT_DIR` lives under `.agent/scratchpad/`, which is
  gitignored. Anchoring it at the main workspace root (step 1) means every
  *worktree on a host* shares one report directory, but a sweep run on
  another host, or in an ephemeral container, leaves a local record nobody
  else can read — and a container's copy dies with the container. **Only a
  `--publish` run escapes this**, and only for the workspace scope: it
  commits `docs/health.md` (step 7), which is durable and shared regardless
  of which host ran the sweep. This is the deliberate trade of local-first —
  the default run is cheap, offline-capable and side-effect-free, and its
  durability is the host's disk. **Project scope never escapes it at all**,
  pending the health-rollup shape decision, § Deferred below — and that
  includes the optional per-project report, which is written to the same
  gitignored directory and is exactly as host-local and ephemeral as the
  primary one. It is also **rotation-limited**: written only on the runs
  where the project root repo fell in this week's chunk, so its cadence is
  the rotation's, not the sweep's.
- **The rotation has no coverage guarantee while the trigger is deferred.**
  `ISO-week mod chunk-count` cycles every repo in `ceil(N/3)` weeks *only under
  a real weekly trigger*. Two hand-runs in the same week re-audit the same
  chunk, and a week with no run is never made up. No cursor state is kept for
  this on purpose — statelessness is what lets the sweep run identically from a
  worktree, a container, or a fresh clone. The report names the chunk index and
  ISO week so a reader can see which slice was covered.

## Deferred: the project-scope rollup and the trigger

**Workspace-scope publish is opt-in** as of this change (step 7,
[#652](https://github.com/rolker/ros2_agent_workspace/issues/652)): the
committed, PR-reviewed `docs/health.md` at the workspace repo root that the
design draft's *Publishing what the sweep finds* describes is what
`--publish` produces, and the default run produces a local report instead.
The step was kept rather than deleted because its eventual caller is parked,
not cancelled: the cloud trigger
([#636](https://github.com/rolker/ros2_agent_workspace/issues/636)) is the
consumer of `--publish`, and unparking it is a change to the trigger, not a
re-implementation of publishing. What remains deferred:

- **The project-repo health-rollup shape.** The operator's narrowing comment
  on this issue draws the line explicitly: publish-by-commit ships for the
  workspace check only in this slice; project-repo checks stay report-only
  until the rollup shape is decided — three candidates are on the table
  (per-repo PR, one document per project, health-follows-roadmap), and the
  choice is explicitly **not made** by this change. Project-scope findings
  stay in the local `.agent/scratchpad/janitor/<ts>-<pid>-sweep.md` report exactly
  as before.
- **The (weekly, unattended) trigger** — decided in principle (operator,
  2026-09-14: a weekly Claude Code cloud Routine, on stated grounds of
  smallest credential surface and exact cadence — see the design draft's
  *The trigger* for the alternatives weighed), but **backburnered
  2026-09-21** in favour of the local-first report this change ships, and
  wiring it under a dedicated `Janitor Sweep Agent` identity is
  [#636](https://github.com/rolker/ros2_agent_workspace/issues/636), not this
  change.

  **What #636 has to implement when it is unparked** — recorded here so the
  design is not rediscovered then. A cloud run is to be **workspace scope
  only**: a cloud checkout of this repo is configured for no project (the
  project is per-clone configuration, not a property of the GitHub repo,
  [#652](https://github.com/rolker/ros2_agent_workspace/issues/652)'s
  decision 4), so #636 must add a **cloud-mode switch that skips checks 2 and
  3 outright**, recording each as
  `SKIPPED(cloud run: workspace scope only)` — a *check-level* status of its
  own, deliberately not step 1a's per-project
  `SKIPPED(no project configured on this checkout)`, which grades whether a
  per-project health report can be written and says nothing about whether a
  check ran. The two strings must stay distinct even though the underlying
  reason is shared. That switch must **not** clone manifests to manufacture
  a project to audit.

  **None of that is true today**, and this change does not make it true: a
  checkout with no `layers/` still follows step 2's manifest-clone row —
  the manifest repo is shallow-cloned from `configs/project_bootstrap.url`,
  the repos enumerate from that clone, and checks 2 and 3 run over them,
  `audit-project` cloning each repo it audits. Step 1a is the only part of
  the sweep that refuses to clone, and it governs the per-project report
  alone. So a cloud run today would do the full project scope over shallow
  clones; making it workspace-scope-only is work #636 owns, not a property
  the sweep already has.

  Hand-run `--publish` testing uses the implementing agent's own identity for
  the workspace-scope commit (step 7).

A future project-scope publish step is still a GitHub **write**, which is the
same crux the workspace-scope one already crossed:

- [ADR-0015](../../../docs/decisions/0015-dispatch-handoff-context-contract.md)
  — a dispatched container has no GitHub write auth; the container produces and
  the host publishes. A container-based trigger inherits that split.
- [ADR-0019](../../../docs/decisions/0019-what-contains-a-dispatched-agent.md)
  — what containment does and does not buy, before assuming a sandboxed runner
  is equivalent.

Cite these rather than re-deriving them. Whatever rollup shape is chosen also
inherits **Known limitations** above, and must settle what a report may say
on a public repo before anything is posted there — which is why step 6
already keeps the host and absolute paths out of it, workspace and project
scope alike.

## Why no `progress.md` entry

`progress.md` is keyed by issue (ADR-0013), and this skill is not issue-scoped
— like its three periodic siblings (`audit-workspace`, `audit-project`,
`issue-triage`), none of which write one. Its durable record is the local
report file(s) written in step 6, and — for the workspace scope, and only on
a `--publish` run — the committed `docs/health.md` from step 7.

## Guidelines

- **Report, don't fix** — this skill identifies staleness. Fixing it is
  separate work with its own issues, decided by the operator. Do not file
  per-finding issues or comments; the operator has asked explicitly for one
  report over issue spam. The workspace-scope commit-and-PR (step 7) is the
  one deliberate exception to "no PRs" — it publishes the sweep's own
  measurement, never a fix, and only when `--publish` asked for it.
- **Publish nothing beyond the workspace-scope `docs/health.md` PR, and
  nothing at all without `--publish`.** No issue, no comment on a project
  repo, no PR against a project repo. A default run makes no GitHub write of
  any kind. Project scope stays report-only pending the rollup-shape
  decision above, `--publish` or not.
- **Never report a check you did not run** — `SKIPPED` and `FAILED` carry a
  reason, and both are visible in the status table and the headline count.
- **Be specific** — "`research_digest.md` last updated 2026-05-02, 132 days, 6
  entries past the 90-day threshold" is actionable; "digest may be stale" is
  not.
- **Don't nitpick** — inherit each chained skill's own judgement about what is
  worth flagging; the sweep aggregates, it does not re-grade.
