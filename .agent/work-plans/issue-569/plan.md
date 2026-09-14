# Plan: Scheduled janitor for staleness/drift detectors — slice one, the sweep

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/569

## Context

Four staleness/drift detectors exist and are all manual-trigger: `audit-workspace`,
`audit-project`, `issue-triage`, and the research-digest freshness nag in
`.agent/knowledge/research_digest.md`. Nothing chains them and nothing persists their
findings — each reports only into the conversation that ran it.

Per the 2026-09-11 scope-decision comment on the issue, this slice delivers **the sweep
only**. The trigger (Routine / anacron / Actions cron) is explicitly deferred, and the
former #564 sequencing gate is lifted — the first sweep *produces* #564's enforcement
inventory. The sweep opens no PRs and files no per-finding issues. As re-scoped at the
pre-push review checkpoint (see the 2026-09-11 revision below) it also
**publishes nothing**: its output is one local report file. It must not assume
`layers/` exists.

Four items were left for this plan by the `## Issue Review` entry; each is decided below
and marked **[D1]**–**[D4]**.

### Plan revision 2026-09-11 (post-`## Plan Review`)

The `## Plan Review` entry returned **changes-requested** with two must-fix and three
should-fix findings, plus four suggestions. At the plan checkpoint the operator directed:
fix **all five** must-fix/should-fix findings in the plan before implementing; drop the
`run_script_tests.sh` registration row (suggestion 6); create the rolling issue with plain
`gh issue create` (suggestion 7); record the rotation's lack of a coverage guarantee as a
known limitation rather than building state for it (suggestion 8); and land the
consequences-map clarifying clause (suggestion 9). The operator also **approved the
instruction-file edits** this PR needs, closing Open Question 1. Each resolution is marked
**[R1]**–**[R9]** below.

### Plan revision 2026-09-11 (post-`## Local Review (Pre-Push)`, round 1)

The pre-push review returned **changes-requested** with 14 must-fix findings.
At that checkpoint the operator made a **scope decision**: *local report only
for now*. The rolling GitHub issue and everything around it — the exact-title
lookup, the create-once path, the `POST FAILED` headline, the `.unpublished`
backlog, and `--dry-run` (which existed only to suppress publishing) — are
**deferred, not abandoned**, and are decided together with the trigger, since
where a sweep's findings live durably depends on what runs the sweep. The
deliverables below are marked **[DEFERRED-PUB]** where that applies; their
design text is kept so the later decision starts from it rather than from
scratch.

Three must-fix findings are therefore **resolved by removal** rather than on
their merits — public-repo exposure of the host/paths/excluded-repo names, the
duplicate-rolling-issue race, and `--dry-run` publishing the backlog. Two
consequences of the decision still land here: the local report is
**timestamped** so successive runs accumulate, and it is written **free of the
hostname and absolute local paths** so the deferred publish decision inherits a
format that needs no scrubbing.

The remaining must-fix findings are fixed on their merits in this round —
the `$ROOT` anchoring of every path the sweep touches, five defects in the
resolver (its empty-success paths, the unvalidated repo name, the version pin,
the cached-clone origin check and the shared-cache race) plus the two skill-side
ones the review raised alongside them, FAILED criteria for the two sub-skill
checks (with the empty-list guard added to `issue-triage` itself rather than to
the janitor's wrapper), and the consequences-map clause, which claimed a durable
output for three skills that have none.

### Plan revision 2026-09-11 (post-`## Local Review (Pre-Push)`, round 2)

The second pre-push review returned **changes-requested** with 9 must-fix
findings and 20 suggestions, all new ground rather than re-opened round-1
items. Three were design-level and changed what this slice contains:

- **The manifest is the first thing "it clones what it needs" must cover.**
  `configs/manifest` is a *symlink into* `layers/main/core_ws/src/<manifest
  repo>/config`, so the claim that a no-`layers/` host still enumerates repos
  could not be true — the manifests live behind the very thing that is missing.
  New `.agent/scripts/manifest_fallback.sh` derives the manifest repo and branch
  from the **tracked** `configs/project_bootstrap.url` pointer, shallow-clones it
  into `.agent/scratchpad/manifest-repo/`, and hands back a config directory;
  `list_overlay_repos.py --config-dir` (new, repeatable) **adds** it to the search
  path rather than replacing it. Resolver exit 3 now means "no manifest **and** no
  bootstrap pointer"; a manifest clone that was attempted and failed is exit 5.
- **One allowlist, reachable without a checkout.** The rotation classified
  origins with `is_field_mode`, which needs a checkout on disk — with none it
  returns "dev mode", silently *including* every gitcloud repo. The URL
  classification is factored out as `is_field_url` in the same file
  (`field_mode.sh`), so `is_field_mode` calls it and the sweep uses it directly:
  one authority (ADR-0011), no second copy of the host list.
- **`issue-triage` enumerates against `$ROOT`**, like the resolver and the sweep.
  It was worktree-relative, which enumerates 0 repos where the main root
  enumerates 35 — so the empty-list guard this PR adds fired as a false FAILED on
  the sweep's primary environment.

The other six must-fix are the `version:` validation (an unvalidated pin reached
`git fetch` as an *option*, executing `--upload-pack=` and still exiting 0 with the
pin unhonoured), check 2's rollup over an empty chunk, the report-write failure
state, the ssh/unattended guarantee, the plan's last rolling-issue line, and the
publish-decision citation — now the issue comment that records it.

Suggestions taken in the same round: bounded `flock` wait plus an honest account
of what the lock does not buy, `optional_layers.txt` honoured in the rotation, the
`10#` form for `date +%V`, url redaction in resolver stderr, the underlay exclusion
named in exit 4, exit-vocabulary consistency between the script header and
`AGENTS.md`, a documented **retention policy** for both scratchpad outputs (keep
the last 20 reports; the clone caches are disposable — this host has hit 100% disk),
the `audit-project` mode/`package.xml`/`$ROOT` fixes, and the plan/progress staleness
lines this section is part of.

### Plan revision 2026-09-11 (post-`## Local Review (Pre-Push)`, round 4 — after publishing)

Round 4 graded the branch **ship: recommended** with three must-fix, and the
operator approved publishing with those three fixed on the PR afterwards
(PR #625, already open). They are three separate false-green paths, all in the
same funnel:

- **All four checks must grade the same tree.** Check 1 delegates to
  `/audit-workspace`, which addresses every input by bare relative path and so
  audits the *current directory* — while checks 2-4 are anchored at `$ROOT`. A
  sweep run from a worktree (this skill's stated normal case) graded that
  branch's governance docs for check 1 and the main root for the rest, and
  reported the mixture as one workspace state. `audit-workspace` takes no root
  argument, so the step now says to `cd "$ROOT"` for that check.
- **A manifest on disk is never bypassed for a clone.** `manifest_fallback.sh`'s
  early return recognised only `configs/manifest/repos`, while
  `get_overlay_repos` also reads `configs/*.repos` — so on that supported layout
  the helper attempted a network clone whose failure the resolver turned into
  exit 5 before the manifest right there was ever opened. A false RED over a
  working state; the early return now covers both layouts (`underlay.repos`
  excluded, since that search ignores it).
- **A refresh that failed is its own outcome.** A cached manifest clone that
  could not be refreshed returned 0 with a stderr warning; both callers capture
  stdout only, so a sweep could report `4 of 4 completed` over a repo list it
  could not verify. It is now rc **6**, mapped to
  `FAILED(manifest refresh: <reason>)` by `janitor-sweep` and `issue-triage`,
  and to exit 5 by the resolver. `AGENTS.md`'s exit-5 row described behaviour
  the code did not have; it now says what the code does.

Adjacent suggestions taken in the same pass: `audit-project`'s second `$ROOT`
site (its rationale contradicted step 1's own caveat — fix every site),
`issue-triage`'s bare enumeration ahead of the fallback snippet (one
enumeration site, on the success path, as `janitor-sweep` already had), the
`AGENTS.md` exit-row wording above, and a retention note naming the clone
caches as the output that actually fills a disk. Round 3's changes remain
recorded as "Round 3 —" clauses in the Files-to-Change rows below; the
remaining round-4 suggestions (trust model, lock scope, and the rest) are
recorded in `progress.md` for the re-review.

## Approach

1. **[D1] Fix the repo-location gap at its source, not in the janitor.** `audit-project`
   step 1 resolves a repo with `find layers/main/*/src/<repo>` — a hard dependency on a
   local layer checkout. Add `.agent/scripts/resolve_repo_checkout.sh <repo-name>`, which
   prints `<path>\t<layer|clone>` on stdout and exits non-zero with a reason on stderr
   when it cannot produce a checkout. Resolution order: (a) an existing
   `layers/main/*/src/<repo>` checkout; (b) otherwise a shallow clone (`--depth 1`) of the
   URL from `list_overlay_repos.py` into `<main-workspace-root>/.agent/scratchpad/janitor-repos/<repo>`
   (gitignored), refreshed in place if already present. `--depth 1` only — a
   `--filter=blob:none` clone that still checks out a working tree refetches every blob
   immediately, so the filter would buy nothing here.

   **[R1] Distinct, named failure statuses — no empty success.** The script's exit codes
   separate the cases the `## Plan Review` found collapsed:

   | Exit | Meaning |
   |---|---|
   | 0 | Resolved — `path<TAB>layer` or `path<TAB>clone` on stdout |
   | 2 | Usage error |
   | 3 | **No repo manifest configured AND none derivable** — no `configs/manifest`, *and* no bootstrap pointer a manifest clone can be derived from, so *zero* repos are enumerable. `list_overlay_repos.py` prints an empty list at exit 0 in this state, which is exactly the #609 false-green the issue forbids; it is a loud FAILED here, never "repo not found". Round 2 narrowed this: the resolver resolves against the **main** root (so a worktree on a set-up host never reaches it), and where a pointer *is* derivable — as it is in this checkout — the manifest repo is cloned instead (`manifest_fallback.sh`). A manifest clone that was attempted and failed is 5, never 3. |
   | 4 | Repo not listed in any manifest (manifests *were* read) |
   | 5 | No checkout could be produced: a clone or refresh failed (including the manifest repo's own clone, and a `WORKSPACE_MANIFEST_GIT_BASE` refused before it reaches git), a layer checkout exists but cannot be read, or the local scaffolding could not be set up (temp dir, cache directory, the per-repo lock — including its bounded wait timing out — or, round 5, the sibling `redact.sh` every diagnostic is routed through). Round 5 also lands here: the cloned `bootstrap.yaml` names a different repo or branch than the pointer derived, or an unsafe `config_path` |
   | 6 | Manifest unreadable, or the repo's entry is malformed: no `url:`, a url in no recognised form, or (round 2) a `version:` that is neither a full commit SHA nor a ref-safe branch/tag name — an unvalidated one reaches `git clone --branch` as an **option** |
   | 7 | The repo is declared in two manifests with a **conflicting url or a conflicting `version:` pin** — the url alone is not the identity of a checkout, so two manifests agreeing on the url and disagreeing on the version are just as ambiguous (round 5); never a silent first-match |

   **[R3]** Rewrite `audit-project` step 1 to call the resolver. In `clone` mode the only
   genuinely layer-dependent items report **SKIPPED (no layer checkout)**, never OK: the
   *optional* `colcon test` invocation inside step 5, and step 7's "correct layer" check.
   Step 5 as a whole is **not** layer-dependent — `audit-project`'s own Guidelines say
   "Don't run tests by default", so its default behaviour is a test-file-existence check
   that works fine against a clone, and marking the whole step SKIPPED would under-report
   a check that did run.

   Rationale for fixing it here rather than in the janitor: the janitor is not the only
   caller that will run outside a full layer tree, and a janitor-local workaround would
   leave the defect in place for every other caller.

   **[R2] All five hardcoded `layers/main/...` sites in `audit-project` change**, not
   just step 1 (SKILL.md lines 34, 40, 62, 106, 130). Line 62's root-`AGENTS.md` currency
   check is precisely what the janitor's onboarding signal reads, and line 130's
   `**Location**` report header must be able to say a clone path. The two remaining
   layer-only sites (line 106's `colcon test`, line 130's layer field) stay layer-shaped
   but are explicitly labelled as such.
2. **Write the janitor skill** at `.claude/skills/janitor-sweep/SKILL.md` (Utility/periodic,
   matching its three siblings). Usage: `/janitor-sweep [--repos <a,b,c>]` — `--dry-run` is
   gone with the publish step it suppressed. Per-check contract: every check ends as
   **OK / FINDINGS / SKIPPED(reason) / FAILED(reason)**; a check that could not run is
   never rendered as a pass, and **each of the four checks names its own FAILED
   evidence** — two of them are sub-skills that report into the conversation rather than
   returning an exit code, so "it seemed to run" is not a status. Where a chained skill
   lacked the guard (`issue-triage`'s empty repo list), it is added to that skill, not to
   the janitor's wrapper. Every path the sweep touches is addressed through the **main
   workspace root**: the sweep's primary environment is a set-up host, and it is usually
   invoked from a worktree, where `layers/` and `configs/manifest` do not exist.
3. **Rotation, stateless by construction.** Candidate set = overlay repos from
   `list_overlay_repos.py` (run through `$ROOT`) filtered to (i) GitHub origins per
   `field_mode.sh`, the authoritative allowlist AGENTS.md names — which also admits
   `ssh.github.com` — a non-GitHub origin (gitcloud) being listed as **excluded: not
   reachable from a generic runner**, not silently dropped — and (ii) repos onboarded far
   enough to audit, probed remotely in **two steps**: `gh api repos/<owner>/<repo>` first,
   then `gh api repos/<owner>/<repo>/contents/AGENTS.md`. One probe cannot tell "no root
   `AGENTS.md`" from "not visible to this token" — both are 404 — and publishing the second
   as the first states a check that never ran as a finding about the repo. The slug is
   derived from the manifest url, never hardcoded (ADR-0003). The gate is `AGENTS.md`
   **presence**; ADR-0017's currency marker is `audit-project`'s check, not the rotation's.
   Sort the survivors by name, chunk by 3, and pick chunk `ISO-week mod chunk-count`.
   Deterministic, needs no persisted cursor. `--repos` replaces the chunk selection only —
   every other rule still applies. The report always lists the full candidate set with
   each repo's in/out status and reason.

   **[R1]** An **empty candidate set is never "no repos to audit"**. Zero repos enumerated
   at all (resolver exit 3 / the same empty-manifest state) is
   `FAILED(no repo manifest — run make setup-all)`. Repos enumerated but all filtered out
   is `SKIPPED(no eligible repos: <reasons>)`, with the full exclusion list. The two are
   reported distinctly.

   **[R8]** The rotation carries **no coverage guarantee while the trigger is deferred**:
   the ISO-week modulus cycles all repos in `ceil(N/3)` weeks *only under a real weekly
   trigger*. Repeated hand-runs inside one week re-audit the same chunk, and a week with
   no run is never made up. This is recorded as a known limitation in the skill, to be
   revisited with the trigger decision — no cursor state is built for it now. The report
   names the **chunk index and ISO week** it used, so a reader can see which slice was
   covered and which were not.
4. **Chain the four checks**: `audit-workspace` (full), `audit-project` on the rotation
   chunk, `issue-triage --stale-days 90`, and a digest-freshness check that reads the
   `<!-- Last updated: YYYY-MM-DD -->` header of `.agent/knowledge/research_digest.md`
   against the 30/90-day thresholds the file itself declares on the next line, plus each
   entry's `**Updated**:` stamp for the 90-day per-entry threshold.
5. **[D3] Durable output: one local report file.** Write the full report to
   `<main-workspace-root>/.agent/scratchpad/janitor/<YYYYmmddTHHMMSS>-<pid>-sweep.md`. The
   filename is **timestamped** (generated with `date`, never hand-typed) so a second run
   on the same day cannot overwrite the first — the runs are the record — and carries the
   pid (round 5) so two runs finishing inside the same second cannot collide either. This write
   depends on neither network nor auth, so the sweep always produces its record.

   The report is written **free of the hostname and of absolute local paths** (files are
   named relative to the workspace root). Nothing publishes it today, but the deferred
   publish decision lands on a repo that may be public, so the format is already one that
   needs no scrubbing. Excluded repo *names* may appear — they are already in the tracked
   manifests.

   **[DEFERRED-PUB]** The rolling GitHub issue is deferred with the trigger. The design
   that was settled and is now waiting: locate it by **exact** title
   `Janitor sweep report (rolling)` on the workspace repo — slug derived
   (`gh repo view --json nameWithOwner`), never hardcoded (ADR-0003); **zero matches →
   create once** with plain `gh issue create` **[R7]** (not `gh_create_issue.sh`, which
   auto-injects `Part of #<WORKTREE_ISSUE>` at gh_create_issue.sh:82-95 and would
   permanently mis-parent the rolling issue to whatever issue the operator happened to be
   on) plus the AGENTS.md AI signature; **two or more matches → FAILED, never guess** —
   though the review showed the search index lags enough that two close runs could both
   create one, so the create path needs a race-safe design before it ships. Body updated
   in place, sweep posted as a comment, and a named `POST FAILED` state so an unpublished
   run cannot read as "nothing to report".

   **[R4] The report is host-local, and the skill says so.** The `## Plan Review` is right
   that `.agent/scratchpad/` is gitignored and, in a worktree, per-worktree. The path is
   anchored at the **main workspace root**, not the worktree — round 5 replaced the
   `dirname $(git rev-parse --git-common-dir)` derivation with `workspace_root.sh`, since
   the former answers with *whatever repo you are standing in* and so returned the project
   repo's root from a layer worktree — so every worktree on a host shares one report
   directory; the residual
   limitation — a sweep run on a *different host*, or in an ephemeral container, leaves a
   record nobody else can read — is stated as a **known limitation** in the skill rather
   than papered over, and is one of the two limitations the deferred decision inherits.

   No findings are ever converted into individual issues; the operator triages from the
   report.
6. **Report shape.** The report names no host and no absolute path. It leads with a coverage header — `Checks: X of 4 completed,
   Y skipped, Z failed` — followed by the per-check status table, then findings grouped by
   check, then `Repos not audited this run (reason)`. The words "clean" / "no findings"
   may only appear when all four checks completed. This is the #609 false-green rule
   applied to a report.
7. **[D2] Land the cross-framework list updates in the same PR** (Consequences Map,
   "Workflow skill list (add/remove a skill)"): `.github/copilot-instructions.md`,
   `.agent/instructions/gemini-cli.instructions.md`, `.agent/AGENT_ONBOARDING.md` (all
   three carry the same `Available workflow skills:` enumeration), plus the Utility-skills
   table in `.agent/knowledge/skill_workflows.md`, plus the `AGENTS.md` script-reference
   row for the new script. **[R9]** Also land the one-line clarifying clause on the
   consequences-map row "Add a workflow skill that produces durable findings" in
   `.agent/knowledge/principles_review_guide.md`: a periodic, non-issue-scoped skill (the
   janitor) persists its record to its own local report file instead of a
   `progress.md` entry, which is keyed by issue — while its three siblings persist
   nothing at all, which the clause says rather than implying a pattern. Without the clause the row reads
   unconditionally and every future `audit-workspace` / `review-code` pass re-flags this
   skill. All of these edits are **operator-approved** (Open Question 1, closed).
8. **[D4] Forward-looking note, non-blocking.** In the skill's
   "Deferred: publishing and the trigger" section — the two are decided together —
   record that updating a rolling issue is a GitHub **write**, so whichever trigger is
   chosen must satisfy [ADR-0015](../../../docs/decisions/0015-dispatch-handoff-context-contract.md)
   (container produces, host publishes — a dispatched container has no GitHub write auth)
   and [ADR-0019](../../../docs/decisions/0019-what-contains-a-dispatched-agent.md)
   (what containment does and does not buy). The later decision cites these rather than
   re-deriving them, and also inherits the two limitations recorded in [R4] and [R8]. No
   ADR is written in this slice — no lasting architecture decision is made here.
9. **Test** `resolve_repo_checkout.sh` in `.agent/scripts/tests/test_resolve_repo_checkout.sh`.
   **[R6]** No edit to `run_script_tests.sh` is needed — it globs `"$TESTS_DIR"/test_*.sh`,
   so a correctly named file is picked up automatically.

   **[R5] Hermetic, per that suite's contract** ("temp sandboxes, stubbed `gh`, no
   network"). Each case builds a throwaway workspace root — a temp dir holding
   **`.agent/scripts` as a symlink** to the real one (so the script's own root resolution
   and `list_overlay_repos.py`'s land on the fake root, since bash's `cd` is logical and
   Python's `abspath` does not resolve symlinks) plus a `configs/*.repos` manifest — and
   the clone cases point that manifest at a **local `file://` bare origin created in the
   same temp tree**. No remote is ever contacted. *Only* `.agent/scripts` is linked, never
   the whole `.agent/`: the resolver's clone cache is `<root>/.agent/scratchpad/`, which
   must stay inside the temp tree, or the cases see each other's clones and write into the
   real workspace (caught while writing the test — linking all of `.agent/` made the
   clone-failure case pass a stale cached clone instead). Forty-five cases (round 5), one per resolution
   rule and one per named failure: prefers a **non-empty** layer checkout; an *empty*
   `src/<repo>` (what a partial `vcs import` leaves) falls through to a clone rather than
   resolving as mode `layer`; an *unreadable* one is exit 5, not a silent fallback; clones
   when `layers/` is absent; the clone honours the manifest's `version:` pin; a refresh is
   distinguishable from a re-clone (an untracked sentinel survives one and not the other)
   and actually advances the tree; a cached clone whose `origin` no longer matches the
   manifest is re-cloned; **exit 5 with a reason when the clone fails**; exit 4 when the
   repo is absent from a manifest that *was* read; **exit 3 when no manifest is configured
   at all** — the false-green path [R1] names; exit 6 for an unparseable manifest, for a
   listed repo with no `url:`, and for a url in no recognised form; exit 7 when one name
   carries conflicting urls; exit 2 for a missing argument, extra arguments, and every
   repo name that is not a single path segment; and — the environment the sweep actually
   runs in — the resolver invoked from a **real git worktree** of the sandbox resolving
   against the MAIN root. Every failure case also asserts **stdout is empty**.

## Files to Change

| File | Change |
|------|--------|
| `.claude/skills/janitor-sweep/SKILL.md` | New — the sweep procedure, per-check status contract (including named FAILED evidence for each of the four checks), report format, timestamped local report write, known limitations, deferred publishing-and-trigger note. Round 4 — check 1 runs from `$ROOT` so all four checks grade the same tree; rc 6 from the manifest fallback is `FAILED(manifest refresh: ...)`; the retention note names the clone caches as the unbounded output Round 5 — the same `workspace_root.sh` opener; the report filename carries the pid; the report template says what the per-repo mode means |
| `.agent/scripts/resolve_repo_checkout.sh` | New — layer-checkout-or-shallow-clone resolver; clones the manifest's pinned version, verifies a cached clone's origin, validates the `version:` pin before it reaches git and verifies a SHA pin after the fact (round 2), serialises the shared cache with a **bounded** `flock` wait (round 2), sources `manifest_fallback.sh` so a host with no `layers/` still has manifests (round 2), validates the repo name before it reaches a path handed to `rm -rf`; prints `path\tmode` and nothing at all on a failure path; fails loud with the distinct exit codes in [R1] Round 5 — a conflicting `version:` pin is as ambiguous as a conflicting url (exit 7); every diagnostic prints through one `say()` funnel that applies `redact_text`, so git's captured output cannot leak the userinfo the message's first half redacted and `$MAIN_ROOT`/`$HOME` do not travel into the report; the contract states what mode `layer` does and does not guarantee (the checkout is accepted as is — comparing its origin or ref against the manifests would manufacture findings on healthy workspaces) |
| `.agent/scripts/tests/test_resolve_repo_checkout.sh` | New — the hermetic cases in [R5] |
| `.agent/scripts/redact.sh` | New (round 5) — `redact_url` / `redact_text`, shared by the resolver and the manifest fallback because both print into the same handed-around report; two copies of the regex drift, and the one that drifts is the one that leaks |
| `.agent/scripts/workspace_root.sh`, `.agent/scripts/tests/test_workspace_root.sh` | New (round 5) — resolve the workspace root properly (validated `$WORKSPACE_ROOT`, else the script's own location, then a hop to the main checkout when that root is a worktree) instead of asking the operator to remember to pass it; 7 hermetic cases including the layer-worktree case the old derivation got wrong |
| `.claude/skills/audit-project/SKILL.md` | All five `layers/main/...` sites: step 1 uses the resolver; the AGENTS.md currency check and the report `**Location**` header accept a clone path; the optional `colcon test` and step 7's "correct layer" report SKIPPED in `clone` mode Round 5 — `$ROOT` comes from `workspace_root.sh` instead of `git --git-common-dir`, which returned the *project repo's* root from the documented layer worktree; the mode's meaning (what a `layer` checkout was and was not verified against) is stated for the report header |
| `AGENTS.md` | Script-reference row for `resolve_repo_checkout.sh` (instruction file — **operator-approved**) |
| `.github/copilot-instructions.md`, `.agent/instructions/gemini-cli.instructions.md`, `.agent/AGENT_ONBOARDING.md` | Add `janitor-sweep` to the skill enumeration (instruction files — **operator-approved**) |
| `.agent/knowledge/skill_workflows.md` | Add `janitor-sweep` to the Utility-skills table |
| `.agent/knowledge/principles_review_guide.md` | [R9] clarifying clause on the durable-findings consequences-map row — stating only what is true: three of the four periodic skills persist nothing at all |
| `.claude/skills/issue-triage/SKILL.md` | Empty-manifest / unreadable-manifest / failed-per-repo-list guards in step 1 — the janitor chains it, and the guard belongs in the skill every caller shares — with the enumeration anchored at `$ROOT` so the guard cannot fire on a worktree run. Round 4 — one enumeration site, on the fallback's success path (the bare call ahead of it printed `[]` at exit 0 on a no-`layers/` host), plus the rc 6 arm Round 5 — the same `workspace_root.sh` opener |
| `.agent/scripts/manifest_fallback.sh` | New (round 2) — derive the manifest repo from the tracked bootstrap pointer and shallow-clone it, so a host with no `layers/` has manifests to read; sourceable, used by the resolver and the sweep. Round 3 — re-clone a cached manifest whose `origin` no longer matches the url the current pointer derives, validate `WORKSPACE_MANIFEST_GIT_BASE` before it reaches `git clone`, and refuse to be executed rather than exiting 0 having done nothing. Round 4 — the early return recognises every layout `get_overlay_repos` reads (`configs/manifest/repos` **and** `configs/*.repos`), and a cached clone that could not be refreshed is its own exit **6** rather than rc 0 with a stale-cache warning Round 5 — every url-bearing message is redacted through the shared helper, and the cloned `bootstrap.yaml` is read as authoritative for `git_url:`/`branch:`/`config_path:` (the same keys `setup_layers.sh` reads), so a manifest repo whose `.repos` do not sit beside its bootstrap is no longer reported as a missing directory |
| `.agent/scripts/field_mode.sh` | Round 2 — factor the URL classification into `is_field_url`; `is_field_mode` calls it. One allowlist for both a checkout and a bare manifest url |
| `.agent/scripts/list_overlay_repos.py`, `.agent/scripts/lib/workspace.py` | Round 2 — `--config-dir` / `extra_config_dirs`, **additive** to the normal search path, for reading a cloned manifest's `.repos` files. Round 3 — `get_optional_layers(..., extra_config_dirs=)` reads `optional_layers.txt` from the same effective config dir, so the optional-layer exclusion is not silently empty on a host with no `layers/` |
| `.agent/scripts/tests/test_field_mode.sh`, `.agent/scripts/tests/test_workspace_lib.py` | Round 2 — `is_field_url` over the same URL table as `is_field_mode`; `extra_config_dirs` additive and tolerant of a stale path |

`.gitignore` already ignores `.agent/scratchpad/*`, which covers both the report directory
and the clone cache — verified, no change.

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Report-only, and in this slice publish-nothing: no PRs, no per-finding issues, no GitHub write at all — one local report file per run, which the operator triages into work. |
| Enforcement over documentation | The sweep is still hand-run this slice — a recorded sequencing choice, not a gap. The one mechanically enforceable piece (repo resolution, including its false-green paths) gets a script and a test. |
| A change includes its consequences | Step 7 lands all four skill-list sites, the script table, and the consequences-map clause in this PR. |
| Test what breaks | The resolver is tested across every exit it can return (2 through 7) and both resolution modes, including the manifest fallback (its origin check, its refused git base, and its sourced-only guard), the `version:` validation, the SHA-pin path and the clone-cache lock, and (round 5) the credential/path redaction, the version-pin ambiguity and the authoritative `bootstrap.yaml` — 45 hermetic cases, plus 7 more for `workspace_root.sh`. The report's degraded behaviour is procedure, not code — stated as explicit report rows rather than claimed as tested. |
| Only what's needed | Chains existing detectors; adds one small script. No scheduler, no rotation state, no new infra. |
| Workspace vs. project separation | Rotation is derived from `.repos` manifests and a remote AGENTS.md probe — no repo names hardcoded (ADR-0003). |
| Improve incrementally | Sweep now, trigger later, as the operator scoped it; the two deferred limitations ([R4], [R8]) are named where the trigger decision will meet them. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0003 (project-agnostic workspace) | Yes | Rotation and checks are data-driven; nothing project-specific is baked into the skill. |
| ADR-0013 (progress.md vocabulary) | Considered, not triggered | `progress.md` is per-issue-keyed; the janitor is not issue-scoped. Its durable record is the local report file, consistent with its three periodic siblings, none of which write `progress.md`. Stated explicitly in the skill, and stated in the consequences map itself ([R9]) — which says only what is true: three of the four siblings persist nothing at all. |
| ADR-0015 / ADR-0019 (dispatch handoff / containment) | Not this slice | Cited as the forward pointer for the deferred trigger decision (step 8). |
| ADR-0017 (AGENTS.md in project repos) | Indirectly | The remote AGENTS.md probe reuses ADR-0017's marker as the onboarding signal; `audit-project`'s currency check is unchanged in substance, only in how it locates the file. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Workflow skill list (add a skill) | Three adapter files + `skill_workflows.md` | Yes — step 7 |
| A script in `.agent/scripts/` | `AGENTS.md` script table | Yes — step 7 |
| A framework skill | That framework's adapter file | Yes — step 7 |
| Add a skill producing durable findings | Consequences-map row says "persist a typed `progress.md` entry" | Yes — [R9] lands the clarifying clause for non-issue-scoped periodic skills |
| Change a chained skill's failure vocabulary | That skill's own SKILL.md, not the caller's wrapper | Yes — `issue-triage`'s empty-list guard landed in `issue-triage` |

## Documentation & Instruction Impact

- **Stale docs** (land in this PR): `.claude/skills/audit-project/SKILL.md` (step 1 becomes
  inaccurate the moment the resolver lands); `AGENTS.md` script table;
  `.agent/knowledge/skill_workflows.md`; the three framework adapter skill lists;
  `.agent/knowledge/principles_review_guide.md` ([R9]).
- All instruction-file edits above are **operator-approved** for this PR (2026-09-11),
  satisfying AGENTS.md § Boundaries "Ask First".

## Open Questions

All three are closed by the 2026-09-11 operator decisions:

- [x] Instruction-file edits (`AGENTS.md`, three adapters, plus
  `principles_review_guide.md`) — **approved** for this PR. The approval was given
  verbally at the 2026-09-11 plan checkpoint and is recorded only in this plan and in
  the `## Plan Authored` / `## Local Review (Pre-Push)` timeline; the review flagged
  that as a thin trace, so **re-confirm it at PR time** before merge. Two details to
  re-confirm with it, both flagged by the round-2 review: the
  `principles_review_guide.md` clause landed as a **four-sentence paragraph**, not the
  "one-line clause" the approval was framed around (it grew to say what is true of all
  four periodic skills, and now also carries the failed-write requirement); and the
  round-2 fixes added `AGENTS.md` script rows for `manifest_fallback.sh` and a rewritten
  row for `resolve_repo_checkout.sh`.
- [x] Consequences-map clause for non-issue-scoped durable-findings skills — **land it**
  ([R9]).
- [x] ~~May the sweep create the rolling issue on first run~~ — **moot for this slice**:
  the operator re-scoped it to a local report only (2026-09-11 pre-push checkpoint). The
  settled answer (create exactly once on zero exact-title matches, plain `gh issue create`
  with the AI signature, two or more is FAILED) carries forward to the deferred
  publishing-and-trigger decision, which must also settle the create-path race the review
  found.

## Open items for PR time

- [ ] **File the follow-up issue for the deferred publishing-and-trigger
  decision.** The skill, this plan and `skill_workflows.md` all cross-reference
  it with no issue number, so nothing tracks it once this PR merges. It needs
  the two known limitations ([R4], [R8]) and the settled-but-unshipped rolling
  issue design (including the create-path race) carried into its body. Left for
  the operator to file — the agent does not open it.
- [ ] **The PR body says "Part of #569", not "Closes #569".** The issue title is
  "*Scheduled* janitor…" and the trigger is explicitly deferred, so this slice
  does not close it (AGENTS.md § Issue-closing keywords — the keyword auto-closes
  even in a negated or sibling mention, so scrub it from any plan text pasted
  into the body, including this line's neighbours).
- [ ] **Re-confirm the instruction-file edits** — see Open Questions above for
  what grew beyond the original framing.

## Estimated Scope

Single PR.
