---
name: janitor-sweep
description: Run the workspace's staleness and drift detectors in one pass, split into a workspace scope and a project scope. Workspace scope publishes a committed docs/health.md via PR; project scope stays report-only to a local file. Neither scope files per-finding issues.
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
chains them into **one sweep** and splits the result into two scopes:

- **Workspace scope** — check 1 (`audit-workspace`) + check 4 (research-digest
  freshness). Both grade the workspace repo itself.
- **Project scope** — check 2 (`audit-project`, per repo in the rotation
  chunk) + check 3 (`issue-triage`, which already scans only overlay/project
  repos, never the workspace repo).

**Workspace scope publishes: it commits a `docs/health.md` at the repo root
and opens a PR** (§ Publish the workspace scope). **Project scope stays
report-only** — its findings land only in the local sweep report under
`.agent/scratchpad/janitor/`, pending the operator's decision on the
project-repo health-rollup shape (per-repo PR / one document per project /
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
sixth, separate state**, on top of the five above: the local report can land
even when the `docs/health.md` publish fails (a worktree that could not be
created, a push that was rejected, a PR that could not be opened), and the
reverse also holds (a report-write failure per the fifth state above is
already terminal for the run and the publish step is never reached). The two
never stand in for each other — a failed publish is stated by name, next to
the local report's own status, never folded into it.

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

- `$REPORT_DIR` — keep the **last 20** reports; delete older ones at the end of
  a run (`ls -1t "$REPORT_DIR"/*-sweep.md | tail -n +21 | xargs -r rm -f`). The
  reports are small, but nothing else will ever prune them.
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
  inside the audit and makes the check `FINDINGS` at worst, never `OK`. Probe
  the inputs before reporting the check's status; do not infer it from the
  narrative.
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
stable key: `tier + check + one-line description`. Render three subsections
under each tier that has any diff activity: `New`, `Resolved`, `Unchanged`.

- **Workspace scope** — the previous run's state is the last **committed**
  `docs/health.md`, read from the janitor-sweep worktree **before** the new
  commit:

  ```bash
  PREV_HEALTH=$(git show HEAD:docs/health.md 2>/dev/null) || PREV_HEALTH=""
  ```

  **First run — no prior `docs/health.md` in history** (the normal case for
  this skill's very first publish, and for any repo where the file was never
  committed, or was removed from the base branch): `git show` fails, `PREV_HEALTH`
  is empty, and this is **not an error**. Every finding renders under `New`
  in every tier, with an explicit note in the report — "first committed
  run — no prior health document" — rather than an empty `Resolved`/
  `Unchanged` subsection that would read as "nothing changed" on a repo that
  has never had a health document before.
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

### 6. Write the report

Write the full report to a **timestamped** file:

```bash
REPORT="$REPORT_DIR/$(date '+%Y%m%dT%H%M%S')-$$-sweep.md"
```

Generate the timestamp with `date`; never hand-type one (AGENTS.md
§ Documentation Accuracy). A date-only name would let a second run the same day
overwrite the first run's report — the runs are the record, so they must
accumulate. Seconds are not enough on their own for the same reason: two runs
finishing within one second would collide, so the pid disambiguates them. The
name stays sortable, and `*-sweep.md` still matches it (the retention sweep in
step 1 globs on that suffix).

This write depends on neither network nor auth, so a sweep that reached step 6
produces its record wherever the filesystem is writable. It is the sweep's
durable output — and, being the durable output, the one place the sweep must
not assume success:

```bash
if ! printf '%s' "$REPORT_BODY" > "$REPORT"; then
    echo "FAILED(report write: could not write $REPORT)"
fi
```

A failed write (read-only filesystem, no space, an unwritable or absent
`$REPORT_DIR`, a container whose scratchpad volume is not mounted) is
`FAILED(report write: <reason>)` per the status contract: **say it in the
conversation, print the findings inline so the run is not lost with the file,
and never describe the sweep as clean**. Checks that completed are still
reported with their own statuses — the failure is the record, not the checks.

**Keep the report free of host identity and absolute local paths.** Name files
relative to the workspace root (`.agent/knowledge/research_digest.md`, not
`/home/…/.agent/knowledge/research_digest.md`), and do not record the hostname.
The workspace section of this same report is also committed into
`docs/health.md` on a public repo (§ Publish the workspace scope) — the local
report and the published one must both be scrubbed the same way, since the
local file is written first and the workspace section is lifted from it
verbatim. Excluded repo *names* are fine — they are already in the tracked
manifests.

Report format — **two top-level sections, `## Workspace` and `## Projects`**,
replacing the single flat `### Check Status` table a prior revision of this
skill used. Each carries its own check-status line and its findings grouped
by tier (§ Classify findings into tiers), each tier further split into `New`
/ `Resolved` / `Unchanged` (§ Run-over-run diff). A tier with no findings this
run is omitted from that scope's section entirely — not rendered as an empty
heading:

```markdown
## Janitor Sweep — <YYYY-MM-DD HH:MM ±HH:MM>

**Checks**: X of 4 completed, Y skipped, Z failed
**Repo rotation**: chunk <i> of <n>, ISO week <YYYY-Www>

## Workspace

| Check | Status | Detail |
|---|---|---|
| Workspace governance (`audit-workspace`) | OK / FINDINGS / SKIPPED(...) / FAILED(...) | ... |
| Research-digest freshness | ... | last updated <date>, <n> days |

**Publish**: committed to `docs/health.md`, PR <url> / not published: <reason>
(§ Publish the workspace scope)

<!-- First committed run for this repo (git show HEAD:docs/health.md fails,
     or docs/health.md has never been committed): say so explicitly here —
     "first committed run — no prior health document" — rather than
     rendering empty Resolved/Unchanged subsections that would read as
     "nothing changed". -->

### 1. Work that can be lost
#### New
- ...
#### Resolved
- ...
#### Unchanged
- ...

### 2. Unowned safety bugs
(same New/Resolved/Unchanged shape)

### 3. Rules that have bitten with no enforcement
(same shape)

### 4. Contradictions in the record
(same shape)

### 5. Drift
(same shape)

## Projects

| Check | Status | Detail |
|---|---|---|
| Project governance (`audit-project`) | ... | <n> repos audited |
| Issue staleness (`issue-triage`) | ... | <n> repos scanned |

**Publish**: report-only, not committed (project health-rollup shape not yet
decided — § Deferred: the project-scope rollup and the trigger)

### 1. Work that can be lost
- **<repo>** [New/Resolved/Unchanged, or "no prior report for this repo" /
  "prior report covered a different chunk"]: ...

### 2. Unowned safety bugs
- **<repo>**: ...

### 3. Rules that have bitten with no enforcement
- **<repo>**: ...

### 4. Contradictions in the record
- **<repo>**: ...

**Provisional decisions with no review scheduled** (distinct labeled
sub-list, not merged into the tier's other findings):
- **<repo>**: <decision text>, made <date>, recorded in <where>, review owed:
  <text> — unscheduled, <n> days since decision
- (or, if no repo in this run's chunk has any `## Decisions made this
  deployment` heading): "no provisional decisions found this run"

### 5. Drift
- **<repo>**: ...

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
Reports here are kept for the last 20 runs. The shallow clones under
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
clean. The `## Workspace` section's tiered findings (minus the
run-over-run-diff subsections, which have no meaning outside this report) are
what gets lifted verbatim into `docs/health.md` in the next step.

### 7. Publish the workspace scope

**Workspace scope only** — never commit `docs/health.md` into a project repo
in this slice; project-scope findings stay in the local
`.agent/scratchpad/janitor/<ts>-sweep.md` report exactly as before this
change. This step runs after the workspace-scope checks, tiering and diff are
done, and only when check 1 and check 4 both reached a terminal status
(`OK`/`FINDINGS`/`SKIPPED`/`FAILED` — a crash mid-check never reaches here).

1. **Create the skill worktree** (now allowlisted —
   `.agent/scripts/worktree_create.sh`'s `ALLOWED_SKILLS`):

   ```bash
   .agent/scripts/worktree_create.sh --skill janitor-sweep --type workspace
   source .agent/scripts/worktree_enter.sh --skill janitor-sweep
   ```

   This produces the timestamped branch `skill/janitor-sweep-<ts>-<nano>`
   (`worktree_create.sh`'s `SYNTHETIC_ID` convention, the same one `research`
   uses).

2. **Read the previous state** (§ Run-over-run diff, workspace half) from
   this worktree, **before** writing the new file:

   ```bash
   PREV_HEALTH=$(git show HEAD:docs/health.md 2>/dev/null) || PREV_HEALTH=""
   ```

3. **Write `docs/health.md`** at the repo root — the path the design draft's
   expected-location table fixes — with the `## Workspace` section's content
   from step 6 (tiers, `New`/`Resolved`/`Unchanged`, generated with `date`,
   never hand-typed). The file is **replaced wholesale each run** (the
   "health" kind's definition in the design draft), not appended to.

4. **Commit** with per-invocation identity (AGENTS.md § Agent Commit
   Identity): for this PR's hand-run testing, the **implementing agent's own
   identity**, not `Janitor Sweep Agent` — standing up that bot identity is
   [#636](https://github.com/rolker/ros2_agent_workspace/issues/636)'s job,
   not this one's.

   ```bash
   git -c user.name="$AGENT_NAME" -c user.email="$AGENT_EMAIL" \
       commit -m "Janitor sweep: workspace health $(date '+%Y-%m-%d')"
   ```

5. **Replace, don't stack, any existing open PR from a prior
   `skill/janitor-sweep-*` branch.** Before pushing the new branch, list open
   PRs whose head matches the prefix and close them with a comment pointing
   at the new one, then delete their branches:

   ```bash
   gh pr list --state open --json number,headRefName \
       --jq '.[] | select(.headRefName | startswith("skill/janitor-sweep-")) | "\(.number)\t\(.headRefName)"' \
   | while IFS=$'\t' read -r OLD_PR OLD_BRANCH; do
       gh pr comment "$OLD_PR" --body "Superseded by this run's sweep PR: <new PR URL, filled in after step 6 opens it>."
       gh pr close "$OLD_PR"
       git push origin --delete "$OLD_BRANCH" || true   # may already be gone (auto-delete-on-merge/close)
   done
   ```

   The comment references the new PR's URL, so run this **after** step 6
   below has opened it, filling in the placeholder — or open the new PR first
   and close-and-comment on the old ones second; either order is fine as long
   as the comment on the old PR names the real new URL, never a placeholder
   left unfilled.
6. **Push and open a non-draft PR** (Copilot does not review draft PRs —
   `reference_copilot_skips_draft_prs.md`):

   ```bash
   git push -u origin HEAD
   gh pr create --title "Janitor sweep: workspace health $(date '+%Y-%m-%d')" \
       --body-file "$BODY_FILE"
   ```

   Title format: `Janitor sweep: workspace health <YYYY-MM-DD>`. The body
   states the coverage line (`X of 4 completed`) and links the local report
   for the full record (workspace **and** project findings — `docs/health.md`
   carries only the workspace half).
7. **A human still merges** (AGENTS.md § Merging, "green CI is not review").
   This step opens the PR; it never merges it.

**Failure is a named, separate state** (the sixth state, § The status
contract): a worktree that could not be created, a commit that failed (e.g.
pre-commit hook rejection), a push that was rejected, or a PR that could not
be opened is reported as `FAILED(workspace publish: <reason>)`, next to — not
instead of — the local report's own status. The local report has already
landed by the time this step runs (step 6 precedes it), so a publish failure
never loses the run's findings; it only means they are not yet committed.

### 8. Report to the operator

Summarise in the conversation and name the report file by its path **relative
to the workspace root** (`.agent/scratchpad/janitor/<file>`). Lead with the
coverage line — `X of 4 completed` — so a partial sweep cannot read as a clean
one. If the local report write failed, lead with that instead and print the
findings inline. State the workspace-scope publish outcome explicitly — the
PR URL on success, or `FAILED(workspace publish: <reason>)` — and state
plainly that project-scope findings are **not** published anywhere beyond the
local report.

## Known limitations

Deliberate, and each is for the deferred decision below to close — they are
listed here so that decision meets them rather than rediscovering them. The
first is now **workspace-scope only** — the workspace half of this limitation
closed when step 7 shipped a committed `docs/health.md`; the project half is
unchanged.

- **The local report is host-local and ephemeral; the workspace half no
  longer is.** `$REPORT_DIR` lives under `.agent/scratchpad/`, which is
  gitignored. Anchoring it at the main workspace root (step 1) means every
  *worktree on a host* shares one report directory, but a sweep run on
  another host, or in an ephemeral container, leaves a local record nobody
  else can read — and a container's copy dies with the container. The
  workspace-scope portion of that same record is now also committed to
  `docs/health.md` (step 7), which is durable and shared regardless of which
  host ran the sweep. **Project scope is still only as durable as the host it
  ran on** — pending the health-rollup shape decision, § Deferred below.
- **The rotation has no coverage guarantee while the trigger is deferred.**
  `ISO-week mod chunk-count` cycles every repo in `ceil(N/3)` weeks *only under
  a real weekly trigger*. Two hand-runs in the same week re-audit the same
  chunk, and a week with no run is never made up. No cursor state is kept for
  this on purpose — statelessness is what lets the sweep run identically from a
  worktree, a container, or a fresh clone. The report names the chunk index and
  ISO week so a reader can see which slice was covered.

## Deferred: the project-scope rollup and the trigger

**Workspace-scope publish is live** as of this change (step 7): a committed,
PR-reviewed `docs/health.md` at the workspace repo root, per the design
draft's *Publishing what the sweep finds*. What remains deferred:

- **The project-repo health-rollup shape.** The operator's narrowing comment
  on this issue draws the line explicitly: publish-by-commit ships for the
  workspace check only in this slice; project-repo checks stay report-only
  until the rollup shape is decided — three candidates are on the table
  (per-repo PR, one document per project, health-follows-roadmap), and the
  choice is explicitly **not made** by this change. Project-scope findings
  stay in the local `.agent/scratchpad/janitor/<ts>-sweep.md` report exactly
  as before.
- **The (weekly, unattended) trigger** — decided in principle (operator,
  2026-09-14: a weekly Claude Code cloud Routine, on stated grounds of
  smallest credential surface and exact cadence — see the design draft's
  *The trigger* for the alternatives weighed), but wiring it under a
  dedicated `Janitor Sweep Agent` identity is
  [#636](https://github.com/rolker/ros2_agent_workspace/issues/636), not this
  change. Hand-run testing in this PR uses the implementing agent's own
  identity for the workspace-scope commit (step 7).

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
report file written in step 6, and — for the workspace scope only — the
committed `docs/health.md` from step 7.

## Guidelines

- **Report, don't fix** — this skill identifies staleness. Fixing it is
  separate work with its own issues, decided by the operator. Do not file
  per-finding issues or comments; the operator has asked explicitly for one
  report over issue spam. The workspace-scope commit-and-PR (step 7) is the
  one deliberate exception to "no PRs" — it publishes the sweep's own
  measurement, never a fix.
- **Publish nothing beyond the workspace-scope `docs/health.md` PR.** No
  issue, no comment on a project repo, no PR against a project repo. Project
  scope stays report-only pending the rollup-shape decision above.
- **Never report a check you did not run** — `SKIPPED` and `FAILED` carry a
  reason, and both are visible in the status table and the headline count.
- **Be specific** — "`research_digest.md` last updated 2026-05-02, 132 days, 6
  entries past the 90-day threshold" is actionable; "digest may be stale" is
  not.
- **Don't nitpick** — inherit each chained skill's own judgement about what is
  worth flagging; the sweep aggregates, it does not re-grade.
