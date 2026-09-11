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
the `$ROOT` anchoring of every path the sweep touches, six resolver defects,
FAILED criteria for the two sub-skill checks (with the empty-list guard added
to `issue-triage` itself rather than to the janitor's wrapper), and the
consequences-map clause, which claimed a durable output for three skills that
have none.

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
   | 3 | **No repo manifest configured** — `configs/manifest` absent or holding no `.repos`, so *zero* repos are enumerable. `list_overlay_repos.py` prints an empty list at exit 0 in this state (verified in this worktree, where `configs/` holds only `project_bootstrap.url`), which is exactly the #609 false-green the issue forbids. It is a loud FAILED here, never "repo not found". |
   | 4 | Repo not listed in any manifest (manifests *were* read) |
   | 5 | Clone/refresh failed, or a layer checkout exists but cannot be read |
   | 6 | Manifest unreadable, or the repo's entry is malformed (no `url:`, or a url in no recognised form) |
   | 7 | The repo is declared in two manifests with **conflicting urls** — ambiguous, never a silent first-match |

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
   `<main-workspace-root>/.agent/scratchpad/janitor/<YYYYmmddTHHMMSS>-sweep.md`. The
   filename is **timestamped** (generated with `date`, never hand-typed) so a second run
   on the same day cannot overwrite the first — the runs are the record. This write
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
   anchored at the **main workspace root**, not the worktree
   (`dirname $(git rev-parse --path-format=absolute --git-common-dir)`, falling back to
   `pwd`), so every worktree on a host shares one report directory; the residual
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
   janitor, and its three siblings) persists its record to its own rolling report instead
   of a `progress.md` entry, which is keyed by issue. Without the clause the row reads
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
   clone-failure case pass a stale cached clone instead). Eighteen cases, one per resolution
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
| `.claude/skills/janitor-sweep/SKILL.md` | New — the sweep procedure, per-check status contract (including named FAILED evidence for each of the four checks), report format, timestamped local report write, known limitations, deferred publishing-and-trigger note |
| `.agent/scripts/resolve_repo_checkout.sh` | New — layer-checkout-or-shallow-clone resolver; clones the manifest's pinned version, verifies a cached clone's origin, serialises the shared cache with `flock`, validates the repo name before it reaches a path handed to `rm -rf`; prints `path\tmode` and nothing at all on a failure path; fails loud with the distinct exit codes in [R1] |
| `.agent/scripts/tests/test_resolve_repo_checkout.sh` | New — the hermetic cases in [R5] |
| `.claude/skills/audit-project/SKILL.md` | All five `layers/main/...` sites: step 1 uses the resolver; the AGENTS.md currency check and the report `**Location**` header accept a clone path; the optional `colcon test` and step 7's "correct layer" report SKIPPED in `clone` mode |
| `AGENTS.md` | Script-reference row for `resolve_repo_checkout.sh` (instruction file — **operator-approved**) |
| `.github/copilot-instructions.md`, `.agent/instructions/gemini-cli.instructions.md`, `.agent/AGENT_ONBOARDING.md` | Add `janitor-sweep` to the skill enumeration (instruction files — **operator-approved**) |
| `.agent/knowledge/skill_workflows.md` | Add `janitor-sweep` to the Utility-skills table |
| `.agent/knowledge/principles_review_guide.md` | [R9] clarifying clause on the durable-findings consequences-map row — stating only what is true: three of the four periodic skills persist nothing at all |
| `.claude/skills/issue-triage/SKILL.md` | Empty-manifest / unreadable-manifest / failed-per-repo-list guards in step 1 — the janitor chains it, and the guard belongs in the skill every caller shares |

`.gitignore` already ignores `.agent/scratchpad/*`, which covers both the report directory
and the clone cache — verified, no change.

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Report-only. No PRs, no per-finding issues, exactly one rolling issue; the operator decides what becomes work. |
| Enforcement over documentation | The sweep is still hand-run this slice — a recorded sequencing choice, not a gap. The one mechanically enforceable piece (repo resolution, including its false-green paths) gets a script and a test. |
| A change includes its consequences | Step 7 lands all four skill-list sites, the script table, and the consequences-map clause in this PR. |
| Test what breaks | The resolver is tested, including three distinct failure paths. The report's degraded behaviour is procedure, not code — stated as explicit report rows rather than claimed as tested. |
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
  that as a thin trace, so **re-confirm it at PR time** before merge.
- [x] Consequences-map clause for non-issue-scoped durable-findings skills — **land it**
  ([R9]).
- [x] ~~May the sweep create the rolling issue on first run~~ — **moot for this slice**:
  the operator re-scoped it to a local report only (2026-09-11 pre-push checkpoint). The
  settled answer (create exactly once on zero exact-title matches, plain `gh issue create`
  with the AI signature, two or more is FAILED) carries forward to the deferred
  publishing-and-trigger decision, which must also settle the create-path race the review
  found.

## Estimated Scope

Single PR.
