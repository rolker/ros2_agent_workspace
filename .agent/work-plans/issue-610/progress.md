---
issue: 610
---

# Issue #610 — merge_pr.sh reports "CI checks failed" when a repo has no CI configured

## Issue Review
**Status**: complete
**When**: 2026-08-24 13:47 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #610
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: needs-more-detail

### Verification of the issue's premises

- **`gh pr checks` exit-code claim — confirmed, but the fine-grained signal
  is on stderr, not the exit code.** Reproduced live against
  `rolker/mru_transform#40` (merged, zero workflows): `gh pr checks 40`
  exits `1` and prints `no checks reported on the 'feature/issue-34' branch`
  to stderr (stdout is empty; `--json` output is identical — same message,
  same exit code, no JSON emitted). `gh help exit-codes` documents only the
  generic 0/1/2/4 convention plus checks-pending's dedicated exit `8`; there
  is no dedicated exit code or `--json` field for "no checks configured."
  The distinguishing signal that does exist is the literal stderr string
  `no checks reported on`. That makes a fix straightforward (grep the
  captured stderr for that string before deciding what to report) but it is
  string-matching an unversioned CLI message, not a stable exit code or
  `--json bucket` value — worth flagging as a fragility the fix should note
  (e.g. a code comment pointing at this exact repro), not a full
  evaporation of the design question. The three-way distinction proposed in
  the issue (pass / fail-or-pending / none-configured) is achievable this
  way.
- **Prevalence — confirmed widespread, not a papercut.** Counted
  `.github/workflows/*.yml|*.yaml` across all 45 git-repo directories under
  `layers/main/*/src/`: **26 of 45 (58%) have no CI workflow file at all**
  (`manda_coverage`, `marine_ais`, `ben_description`, `ben_project11`,
  `drix_description`, `lr30_project11`, `mobile_lab`, `mru_transform`,
  `edgetech_sonar`, `imagenex_deltat`, `ros2_network_monitor`,
  `starlink_stats_ros`, `unh_marine_radar`, `ben_gazebo`, `drix_gazebo`,
  `unh_marine_simulation`, `ccomjhc_project11`, `image_warper`,
  `rqt_marine_radar`, `rqt_udp_bridge`, `rviz_sonar_image`,
  `detection_visualizer`, `geographic_info`, `nmea_navsat_driver`,
  `ros2launch_gui`, `ros2launch_session`). This reframes the issue per the
  filer's own note: the merge gate is materially weaker than "green CI"
  implies across roughly half the project-repo surface, not just
  `mru_transform`. (Caveat: this count is workflow-file presence, a proxy —
  it doesn't distinguish "genuinely no CI" from "CI defined some other way";
  for all 26 the same `no checks reported` behavior from the reproduction
  above applies whenever `merge_pr.sh` runs `gh pr checks` against them.)
- **ADR-0018 claim — confirmed, and this is the more load-bearing half of
  the issue.** [ADR-0018](../../../docs/decisions/0018-local-first-ci-verification.md)
  §Decision item 1 does make a full-scope `ci-local` attestation
  (`refs/notes/ci-local`, `ci-local: pass`, `scope: full`) an accepted
  substitute for hosted CI on project-repo PRs. Its own Consequences
  section already states the gap plainly: *"`merge_pr.sh` does not yet
  check for the attestation before merging; wiring that check in (warn or
  refuse when the head lacks a full-scope record) is a natural hardening
  follow-up under #572."* Grepping `.agent/scripts/merge_pr.sh` confirms:
  no reference to `ci-local` or `refs/notes/ci-local` anywhere in the
  script — it only ever calls `gh pr checks`. So for a `mru_transform`-style
  repo, the *documented* accepted path (a full-scope `ci_local.sh`
  attestation) is currently unusable through `merge_pr.sh` regardless of
  how the no-checks-vs-failed distinction is resolved, because the script
  never looks at the note. The issue also correctly notes `ci_local.sh`
  presently cannot even run against `mru_transform` (its container can't
  resolve `ros-jazzy-geodesy`) — not re-verified here (out of scope: fixing
  CI in affected project repos, per the issue's own scope line), but it
  means the ADR-0018 path is doubly blocked for this specific repo today.
- **mru_transform#35 cross-check** — confirmed open, title matches the
  issue's characterization ("No verification in this repo: lint_auto
  registers zero linters, no CI workflow, no pre-commit"); no
  `.pre-commit-config.yaml` present in the local checkout.

### Scope Assessment

**Well-scoped?** Partially. The issue explicitly defers the *design*
decision ("Proposed direction (not settled)") to implementation time and
lists three candidate resolutions without picking one. That's reasonable
framing for review-issue → plan-task, but plan-task will need to settle it
before writing a plan, or the plan itself needs to carry the decision
explicitly rather than deferring further. Recommend plan-task resolve the
open question using this review's ADR-0018 finding: a `--no-ci-configured`
flag alone (issue's option 2) treats the *whole space* of no-CI repos the
same as an explicit human override, which reintroduces the exact
distinguishability problem the issue is trying to solve one level up (now
"did the operator mean this repo has no CI" vs "did the operator mean I've
verified some other way"). Consulting `refs/notes/ci-local` first (ADR-0018
option) is the more principled fix and is *already* the documented accepted
verification path — recommend making the ci-local check the primary
resolution for "no checks configured" (proceed automatically when a
full-scope attestation is present, as ADR-0018 already permits) and
reserving a distinct opt-in flag only for the residual case of a repo with
neither hosted checks nor a local attestation.
**Right repo?** Yes. `merge_pr.sh` lives in `.agent/scripts/` in this
(workspace) repo; this is workspace tooling, not project content —
consistent with the workspace-vs-project separation principle.
**Dependencies**: Overlaps directly with the still-open hardening
follow-up named in ADR-0018 (tracked loosely under the #572 umbrella, not
a numbered sub-issue as far as this review found). Implementation should
either close that named gap as part of this issue or explicitly note it's
being deferred again. Also touches `mru_transform#35` (verification gaps
in that specific repo) but this issue's own "Out of scope" line correctly
excludes fixing CI in affected project repos — keep that boundary.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | Watch | Current behavior actively misinforms the operator ("CI checks failed" when nothing failed) — this is the core bug and squarely a transparency violation worth fixing regardless of which design option is chosen. |
| Enforcement over documentation | Action needed | ADR-0018 already documents the accepted ci-local-attestation path but `merge_pr.sh` doesn't enforce/consult it — a rule that exists only in the ADR text, not in the tool. Wiring the check in is exactly what this principle calls for. |
| Capture decisions, not just implementations | OK | This is a review-stage issue; the eventual design choice among the three proposed directions should get its rationale recorded (progress.md plan entry at minimum; an ADR amendment to 0018 if the ci-local-consultation piece lands, since that's already an ADR-0018 decision-column item). |
| A change includes its consequences | Watch | If the fix wires in a `ci-local` check, `AGENTS.md`'s merge-verification section and/or ADR-0018 itself should be updated to reflect that `merge_pr.sh` now performs the check ADR-0018 describes as pending — currently ADR-0018 explicitly says it doesn't. |
| Only what's needed | OK | Issue scope is narrow (one script's error handling) and explicitly excludes fixing CI in downstream repos. |
| Improve incrementally | OK | Fits a single PR; the three-way check/flag/attestation logic is a contained change to one script. |
| Test what breaks | Action needed | Any fix should include a test (or at least a documented manual repro, since `merge_pr.sh` isn't unit-tested elsewhere in this workspace) exercising the "no checks configured" stderr path found in this review — reproduce the `mru_transform`-style case without needing a real no-CI repo to be live (e.g., stub/mock `gh pr checks`), per AGENTS.md's "test what breaks" and "fix it completely" standard. |
| Workspace vs. project separation | OK | Stays within workspace tooling. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| 0018 — Local-first CI verification | Yes | Directly implicated — see ADR-0018 verification findings above. The issue's third proposed direction ("consult ADR-0018's local-CI attestation") is not a new decision; it's finishing an already-accepted decision's stated follow-up. |
| 0004/0005 — Enforcement hierarchy / layered enforcement | Watch | Relevant to *how* the fix should be enforced (script-level check vs. just a warning) but not decision-blocking for this review. |
| 0013 — progress.md entry-type vocabulary | N/A (process, not code) | This review's own entry follows it. |

### Consequences

- If the fix wires in a `refs/notes/ci-local` check, ADR-0018's
  Consequences section ("`merge_pr.sh` does not yet check for the
  attestation...") becomes stale and should be updated or the ADR
  superseded/amended per its own cross-reference-addendum mechanics.
- If `AGENTS.md`'s "Merge verification (ADR-0018)" section describes
  `merge_pr.sh` behavior explicitly, it should be checked/updated to match
  whatever the implementation lands on.
- No `.msg`/`.srv`, parameter, or topic surfaces are touched — standard
  script-consequences checklist otherwise not applicable.

### Recommendations

- Resolve the "proposed direction (not settled)" question during
  plan-task rather than carrying it forward unsettled; this review
  recommends ci-local-attestation-first (see Scope Assessment) as it
  reuses an already-accepted ADR-0018 path instead of adding a fourth
  meaning to CLI flags.
- Keep the stderr-string-match fragility visible in the implementation
  (comment + citation of this review's repro) since it's matching an
  unversioned `gh` CLI message, not a documented stable interface.
- Consider whether the 58%-no-CI prevalence finding belongs as its own
  follow-up note (not this issue's scope) — it's a workspace-wide
  documentation/expectation gap ("green CI" merge gate) beyond what one
  script fix addresses.

### Actions
- [ ] Settle the "proposed direction (not settled)" design choice in plan-task; recommend ci-local-attestation-first per this review's Scope Assessment.
- [ ] Update ADR-0018's Consequences section (or supersede/amend) if merge_pr.sh gains a ci-local check, since it currently states the opposite.
- [ ] Update AGENTS.md's "Merge verification (ADR-0018)" section to match whatever merge_pr.sh behavior lands.
- [ ] Add a test or documented repro for the "no checks configured" stderr path (mock/stub gh pr checks) rather than relying on a live no-CI repo.
- [ ] Note the stderr-string-match fragility (unversioned gh CLI message) in the implementation as a comment.

## Plan Authored
**Status**: complete
**When**: 2026-08-24 14:03 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-610/plan.md` at `5f5efab`
**Branch**: feature/issue-610 at `5f5efab`
**Phases**: single

### Open questions
- [ ] No open questions — plan is review-plan-ready.

## Plan Review
**Status**: complete
**When**: 2026-08-24 14:08 -04:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-610/plan.md` at `5f5efab`
**PR**: PR-less (`--issue` mode; branch `feature/issue-610`, base `main`)
**Verdict**: changes-requested

### Evaluation

| Dimension | Verdict | Notes |
|---|---|---|
| Scope | Good | One script + one sourced helper + docs + two test files is proportionate; the separate helper file earns its keep by making the note logic hermetically testable. |
| Issue alignment | Good | Implements the settled direction (ci-local first, then warn; no second override flag) and closes ADR-0018's named gap. |
| File targeting | Good | Right files. `BRANCH_REPO` (the main project checkout) shares the ref store with linked worktrees, so a note written by `ci_local.sh` in a worktree is already visible — no extra fetch needed in the common case; worth stating so nobody adds one. |
| Consequences | Needs work | Misses ADR-0018 decision 4 (workspace-repo exemption) and decision 5 (push `refs/notes/ci-local` at merge time when the note is what authorized the merge). |
| Documentation & instruction impact | Good | Section present, non-silent, correctly scoped ("None" for instruction candidates with a reason). |
| Principle alignment | Concern | "Test what breaks": the empty-rollup classification converts today's **fail-closed** race into a **fail-open** one on the 42% of repos that do have CI — the opposite of the transparency goal. |
| ADR compliance | Needs work | ADR-0018 decision 4 unaddressed; ADR-0012 instrument is right but for the wrong stated reason, and the addendum must stay navigational. |
| ROS conventions | N/A | Workspace tooling. |

### Findings
- [ ] (must-fix) Empty `statusCheckRollup` is **not** unambiguously "no CI configured" — a head whose workflows have not yet registered check runs, a `paths:`-filtered workflow that does not match, and a queued check suite with no runs all present as `[]`. Today that race fails **closed** (misleading message, no merge); the plan makes it fail **open** (merge on a warning). Add a distinguishing test — presence of `.github/workflows` **at the PR head** (`gh api repos/<repo>/contents/.github/workflows?ref=<head_sha>`; verified: 404 on `rolker/mru_transform`) — plus a short settle/re-poll, and keep the ambiguous case fail-closed with an accurate message. Do **not** use `gh api .../actions/workflows`: it returns `total_count: 2` for `mru_transform` (dynamic Copilot entries). — `plan.md:45`
- [ ] (must-fix) ADR-0018 decision 4 exempts the **workspace repo** — its hosted checks stay required, and a `ci-local` attestation is accepted only for *project-repo* PRs. The plan's three-way logic is repo-agnostic, so on `ros2_agent_workspace` an empty rollup (most likely exactly when merging right after a push) would warn-and-merge. Gate the new path on the repo not being the workspace repo. — `plan.md:38-100`
- [ ] (must-fix) A failed `gh pr view --json statusCheckRollup` (auth, network, rate limit) must not be read as an empty array — that is a second fail-open path. Specify: non-zero `gh` exit or non-JSON output → error out, never "no checks configured". (`jq` on empty input exits 4, so the plan's flow needs the distinction made explicitly.) — `plan.md:38-47`
- [ ] (must-fix) Scratch-ref fetch: the non-clobbering claim is **verified** (fetching `refs/notes/ci-local:refs/notes/ci-local-merge-check` leaves a local unpushed `refs/notes/ci-local` intact). But a leftover scratch ref from an interrupted run makes the next fetch fail non-fast-forward (reproduced, rc=1), yielding a false `no-attestation` → merge on a warning instead of on evidence. Use a forced refspec (`+refs/...`) and delete the scratch ref up front / in a trap. — `plan.md:62-68`
- [ ] (suggestion) The exact-head rule is **correct and workable, and not too strict**: `git notes --ref=<ref> show <sha>` is keyed by the sha path and resolves even when the head commit object is absent locally (verified), so no ancestor can be matched by accident and no commit fetch is needed. Keep it — but spell out the warning/verdict text so the recourse is "re-run `ci_local.sh` on the new head", not `--no-wait`; an unhelpful message here recreates the flag habituation the issue exists to prevent. — `plan.md:69-76`
- [ ] (suggestion) `upstream.repos` completeness: the plan does not say *which* copy is parsed. `BRANCH_REPO` is the main checkout, typically on the default branch, so the working-tree file may not be the PR head's. Read it at the head commit (`git -C <repo> show <head_sha>:upstream.repos`) and state the verdict when the head object is not local (note lookup works without it; `git show` does not). — `plan.md:82-90`
- [ ] (suggestion) ADR-0018 decision 5 pushes `refs/notes/ci-local` at merge time. If `merge_pr.sh` now merges *because of* a local note, it should push that ref at merge time (or record why not) — otherwise the evidence that authorized the merge exists only on the merging machine. — `plan.md:117-131`
- [ ] (suggestion) ADR-0012 instrument: the addendum is the right call, but the plan's reasoning inverts the ADR — substantive changes require **superseding**, not an addendum; addendums are permitted because they are navigational. Keep the `### Addendum (#610)` strictly navigational (gap closed → #610 / AGENTS.md). The new third-state policy ("merge proceeds when nothing verifies it") is a consequence ADR-0018 never recorded and must not be asserted inside it; land it in AGENTS.md. — `plan.md:117-131`
- [ ] (suggestion) Test hermeticity holds — the bare-`origin` fetch fixture works over a local path with no network (verified), and the `upstream.repos` and no-attestation-at-all cases are pure fixtures. But the `merge_pr.sh`-side test only asserts `gh pr checks` is never called; add stubbed-`gh` cases asserting the three-way *outcomes* (warning text on stderr, attested-path message, workspace-repo fail-closed) — the wiring, not the helper, is what will rot. — `plan.md:166-175`

## Implementation
**Status**: complete
**When**: 2026-08-24 14:25 -04:00
**By**: Claude Opus

**Plan**: `.agent/work-plans/issue-610/plan.md` at `a4be918` (revised for the plan review before implementing)
**Branch**: feature/issue-610 at `73470b7`
**Commits**: `a4be918` (plan revision), `e228118` (helper + tests), `3c2bf92` (merge_pr.sh classification + tests), `73470b7` (AGENTS.md + ADR-0018 addendum)

### What landed

- **`.agent/scripts/_ci_verification_helpers.sh` (new)** —
  `ci_local_attestation_status <repo> <head_sha>`: local-first note lookup,
  scratch-ref fetch fallback with a **forced** refspec + up-front delete,
  exact-head matching, all appended records parsed (`pass (partial)` never
  matches), and `upstream.repos` completeness read **at the head commit**
  (fails closed when that commit object is not local).
- **`.agent/scripts/merge_pr.sh`** — the CI-wait block became a four-state
  classification: checks present → unchanged wait/gate; workflows present at
  the head but no checks registered → settle/re-poll then **refuse**; project
  repo with no workflows + full-scope attestation → merge on it and push
  `refs/notes/ci-local` first; project repo with no workflows and no
  attestation → merge with a loud warning naming the recourse. `gh` failures
  and non-404 probe failures are errors. `MERGE_PR_SETTLE_ATTEMPTS` /
  `MERGE_PR_SETTLE_SECONDS` tune (and, in tests, zero out) the settle window.
- **`AGENTS.md`** — the four states, the `gh`-failure rule, and the two
  Script Reference rows. **ADR-0018** — Status-line pointer + a strictly
  navigational `### Addendum`; no Decision/Consequences text edited.
- **Tests** — `test_ci_verification_helpers.sh` (new, 15 assertions) and
  eight new stubbed-`gh` cases in `test_merge_pr.sh`.

### Plan review must-fixes — how each was closed

1. **Empty-rollup race (fail-open risk)**: `[]` is never read as "no CI" on
   its own. `gh api repos/<r>/contents/.github/workflows?ref=<head_sha>` is
   the discriminator — 404 = genuinely no CI; a listing = CI exists, so
   re-poll and then **fail closed** with a message that says what is actually
   happening (Actions not started, or every workflow filtered out for this
   PR's paths). A code comment records why
   `gh api .../actions/workflows` must not be used (`total_count: 2` for
   `mru_transform` from dynamic Copilot entries).
2. **ADR-0018 decision 4**: the substitution path is gated on
   `BRANCH_REPO != ROOT_DIR`. The workspace repo can only settle-and-refuse;
   regression test asserts the ADR-0018-decision-4 message and that no merge
   is attempted.
3. **`gh` failure ≠ empty array**: `check_rollup_count()` treats non-zero
   exit, empty output, and non-JSON as errors (`jq -e` separates "parsed" from
   "input was not JSON"); the workflows probe likewise errors on anything that
   is not a 404. Both have regression tests asserting the output does *not*
   claim "no verification" and that no merge is attempted.
4. **Leftover scratch ref**: forced refspec (`+refs/notes/ci-local:…`) plus an
   up-front delete and a cleanup after use. Verified as a real guard: with the
   `+` and the delete removed, `test_ci_verification_helpers.sh` drops to
   14 passed / 1 failed; restored, 15/0.

Suggestions also applied: `refs/notes/ci-local` is pushed at merge time when
the note authorized the merge (decision 5); the ADR-0018 addendum is
navigational with the third-state policy landing in `AGENTS.md` (ADR-0012
reasoning corrected); the no-attestation message points at re-running
`ci_local.sh` on the new head rather than `--no-wait`; `upstream.repos` is
read at the head commit; the `merge_pr.sh` tests assert the three-way
outcomes; the shared-ref-store point is documented in the helper's comments
so nobody adds a redundant fetch.

### Verification

- `make test-scripts`: **22 shell test files ✅, 73 pytest tests passed, 0
  failures** (includes the new `test_ci_verification_helpers.sh` at 15/15 and
  `test_merge_pr.sh` at 24/24 — 16 pre-existing + 8 new).
- `make validate`: **PASSED**. Its one warning (Check 3, polluted baked parent
  chains in `underlay`/`core`) is a pre-existing build-state condition,
  unrelated to this change.
- Pre-commit hooks (incl. shellcheck) ran clean on every commit; no
  `--no-verify`.
- A green local run is **not** the gate for this PR: the workspace repo's
  hosted checks are required — the very ADR-0018 decision implemented here.

### Notes / follow-ups

- One deliberate scope call: the `gh pr checks` **stderr string** (`no checks
  reported on`) is not parsed anywhere. The classification makes it
  structurally unreachable, which is stronger than matching an unversioned CLI
  message, so the Issue Review's "note the fragility in a comment" action is
  satisfied by removing the dependency instead.
- Not in scope (per the issue): fixing CI in the 26 project repos that lack
  it, and `ci_local.sh`'s inability to run against `mru_transform`.

### Actions
- [ ] Human content review of the `AGENTS.md` change (instruction file) before merge.
- [ ] Hosted checks must be green before merging this PR (workspace repo, ADR-0018 decision 4).

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-08-24 14:36 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-610 at `f5db667`
**Mode**: pre-push
**Depth**: Deep (reason: merge gate for every repo in the workspace; new fail-open surface + an instruction-file change)
**Must-fix**: 7 | **Suggestions**: 14
**Round**: 1 | **Ship**: continue — the four plan-review must-fixes are genuinely closed, but three independent reads found new fail-open paths in the same class (see findings 1-4), plus an instruction-file/behavior mismatch.

Specialists: static analysis (shellcheck --severity=warning clean; `make test-scripts` 22 shell files + 73 pytest, 0 failures; `make validate` PASSED with the pre-existing Check-3 baked-chain warning), governance + plan drift, two disjoint-lens Claude adversarial passes. Copilot and local-model passes not run (per dispatch).

Verified by mutation, not by reading: the scratch-ref guard is load-bearing (14/1 with the `+` and the up-front delete removed, 15/15 restored); `pass (partial)` cannot match as full scope, and a `ci-local: pass` in one appended record cannot combine with a `scope: full` in another (rejected in both orderings).

### Findings
- [x] (must-fix) `upstream.repos` completeness (#577) is skipped entirely when the head commit object is not local and the note carries no `upstream-repo:` lines — the inverse of the guard's stated intent; found independently by all three reads — `.agent/scripts/_ci_verification_helpers.sh:145-162`
- [x] (must-fix) The `.github/workflows` 404 probe cannot distinguish "no CI" from a permission 404 or a bad-ref 404 (`No commit found for the ref`) — a token without Contents read makes every project repo classify as no-CI and merge on a warning; `HEAD_SHA` is also never validated as 40-hex before being used as both a URL ref and a git revision — `.agent/scripts/merge_pr.sh:372-397`
- [x] (must-fix) The scratch ref name is a constant, so a concurrent `merge_pr.sh` in the same repo deletes it mid-fetch and the attested PR falls through to the merge-with-no-verification state (reproduced 10/10); use a per-process name — `.agent/scripts/_ci_verification_helpers.sh:20,125-131`
- [x] (must-fix) The decision-5 note push fails in both normal states — no local ref when the note came from origin (warning then falsely claims the evidence "exists only on this machine" and prescribes a command that fails identically), and non-fast-forward whenever origin's notes ref has records this checkout never fetched (git never fetches `refs/notes` by default); both verified — `.agent/scripts/merge_pr.sh:463-473`
- [x] (must-fix) The state-4 recourse prints `ci_local.sh $BRANCH_REPO` — the main checkout, which sits on the default branch — so following it attests the wrong commit and the re-run still reports no attestation — `.agent/scripts/merge_pr.sh:449-450`
- [x] (must-fix) AGENTS.md does not match the script in three places: the standing paragraph still promises attestation "instead of waiting for hosted Actions" while state 1 now always waits; state 2 omits that checks appearing during the re-poll are waited on and gated, not refused; and the § Merging bullets above still describe an unconditional gate with no back-link to the new merge-on-a-warning state — `AGENTS.md:462-490` vs `merge_pr.sh:405-419`
- [x] (must-fix) The new merge_pr test cases build fixtures inside the real workspace root with no `trap`, no empty-`ROOT_DIR` guard, and the `--repo-slug workspace` case omits the `GIT_SSH_COMMAND=/bin/false` guard the others carry — a regressed gate would run branch-delete and `make sync` against the real repo — `.agent/scripts/tests/test_merge_pr.sh:18,182-334`
- [x] (suggestion) `jq -e '.statusCheckRollup | length'` yields 0 for a missing key as well as an empty array; assert the key is an array so a gh schema change fails closed — `.agent/scripts/merge_pr.sh:347`
- [x] (suggestion) `${entry}` is interpolated unescaped into `grep -qE`; an `upstream.repos` key of `.*` satisfies completeness with one line. Apply ci_local.sh's `^[A-Za-z0-9_-]+$` validation on the consumer side — `.agent/scripts/_ci_verification_helpers.sh:77`
- [x] (suggestion) `MERGE_PR_SETTLE_ATTEMPTS`/`_SECONDS` reach arithmetic contexts unvalidated (command execution demonstrated via `x[$(...)]`); validate as `^[0-9]+$` — `.agent/scripts/merge_pr.sh:330-331,408-410`
- [x] (suggestion) The ADR-0018 decision-4 exemption is a path-string compare; a workspace worktree outside `.workspace-worktrees/` would drop it. Identify the workspace repo by `GH_REPO` slug as well — `.agent/scripts/merge_pr.sh:402-403`
- [x] (suggestion) `.github/workflows` existing is not the same as a runnable workflow; a dir holding only a README or a `workflow_dispatch`-only file wedges the repo in state 2 permanently, with `--no-wait` the only escape the docs tell agents not to take — `.agent/scripts/merge_pr.sh:387-391`
- [x] (suggestion) `gh pr merge` is not passed `--match-head-commit "$HEAD_SHA"`, so a push landing after the attestation check merges a commit nothing attested — `.agent/scripts/merge_pr.sh:476`
- [x] (suggestion) The push warning swallows git's stderr (`2>/dev/null`), so the operator cannot tell non-fast-forward from missing-ref — and the two need different remedies — `.agent/scripts/merge_pr.sh:466`
- [x] (suggestion) "after $SETTLE_ATTEMPTS poll(s)" undercounts by one — the initial read at line 355 also polled — `.agent/scripts/merge_pr.sh:432`
- [x] (suggestion) Add helper tests for the two cases the mutation work exercised but nothing pins: head-absent with a note that omits `upstream-repo:` lines, and a cross-record `pass`/`scope: full` split — `.agent/scripts/tests/test_ci_verification_helpers.sh`
- [x] (suggestion) `Makefile:77` still says "NO_WAIT=1 skips the CI wait"; it now skips the attestation lookup and the note push too — `Makefile:77`
- [x] (suggestion) The test file header still says the CI-wait block is "intentionally NOT exercised here", directly above eight cases that exercise it — `.agent/scripts/tests/test_merge_pr.sh:8-11`
- [x] (suggestion) The ADR-0018 addendum is a `###` under `## Alternatives considered`, so it renders as an alternative; the ADR-0013 precedent puts it under `## References` — `docs/decisions/0018-local-first-ci-verification.md:141`
- [x] (suggestion) AGENTS.md item 4 of a four-state list calls itself "the honest third state" — `AGENTS.md:487`
- [x] (suggestion) Sync the plan text with what shipped: the scratch-ref `trap ... RETURN` became an explicit delete, and `--no-wait`'s meaning did widen — `.agent/work-plans/issue-610/plan.md:172-175,233-235`

### For the operator (decisions, not defects)
- [x] State 4 is a real loosening: a project repo with no CI and no attestation now merges on a stderr warning where it previously refused (26 of 45 project repos have no workflow file). ADR-0018's Consequences named "warn or refuse" as the open choice, so it is authorized — but it is recorded only in AGENTS.md, and there is no prompt or distinct exit code. Yes/no: is the AGENTS.md paragraph enough, or should state 4 require an explicit flag and/or a superseding ADR?
- [x] The attestation is consulted only when a repo has no workflows at all; project repos that do have CI still must wait on hosted Actions. That is narrower than ADR-0018 decision 1 reads. Deliberate scope, or a follow-up?

## Implementation
**Status**: complete
**When**: 2026-08-24 14:59 -04:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-610 at `e4c6004`
**Addressed**: `## Local Review (Pre-Push)` (2026-08-24 14:36 -04:00, branch at `f5db667`) — 7 must-fix, 14 suggestions, plus the two operator-decision items
**Commits**: `4e0b796` (helper fail-open paths), `0beae40` (merge_pr fail-open paths + the ADR-0018 decision-1 widening), `b10e184` (AGENTS.md / ADR addendum / Makefile / plan), `e4c6004` (empty workflows dir is not CI)

### The seven must-fixes

1. **#577 completeness skipped when the head object is absent** — the guard only
   refused when the *note itself* carried `upstream-repo:` lines, so a note that
   omitted them bypassed the rule entirely. Now: one targeted `git fetch origin
   <sha>`, then **fail closed unconditionally** if the commit is still absent —
   without it, `upstream.repos` at the head is unreadable, so completeness is
   not evaluable at all.
2. **The `.github/workflows` 404 could be a permission or bad-ref 404** — the
   `gh` text is byte-identical for all three. A 404 is now believed only after a
   **control probe** of the repository root at the same ref reads back; if that
   also 404s, merge-pr refuses and names both possibilities. `HEAD_SHA` is
   validated as 40-hex before use as a URL ref and a git revision.
3. **Constant scratch ref → concurrent runs sabotage each other** — the name is
   now built per call (prefix + PID + `$RANDOM` + ns). A test runs 10 concurrent
   lookups in one repo; under the constant name it loses the attestation.
4. **Decision-5 note push failed in both normal states** — extracted to
   `ci_local_push_attestation()`, which (a) reports "already published; nothing
   to push" when there is no local ref because the note came from origin —
   instead of falsely claiming the evidence exists only on this machine — and
   (b) reconciles with origin's notes ref (`notes merge -s cat_sort_uniq`,
   correct for append-only records) before pushing, so the routine
   non-fast-forward becomes a fast-forward. Failures now quote git's own stderr
   and prescribe a command that actually works. Three hermetic tests, including
   one asserting both sides' records survive on origin.
5. **State-4 recourse named the main checkout** (on the default branch, so
   following it attests the wrong commit) — it now names a checkout verified to
   be sitting on the head, and otherwise says explicitly that one is needed.
6. **AGENTS.md ↔ script mismatches** — § Merge verification is rewritten as a
   precedence table: no more "instead of waiting" while the code always waited;
   checks that appear during the settle re-poll are stated to be waited on and
   gated; the § Merging bullets no longer describe an unconditional gate and
   link down to it. The ADR-0018 addendum moved under a new `## References`
   heading so it stops rendering as an alternative considered.
7. **Test fixtures inside the real workspace root** — added a `trap ... EXIT INT
   TERM` cleanup, a refusal to run when `ROOT_DIR` does not resolve to a real
   git root, a refusal to run if the fixture layer already exists, and
   `GIT_SSH_COMMAND=/bin/false` on the `--repo-slug workspace` case. A `make`
   stub keeps the one case that runs to completion from syncing the workspace.

### Operator decision A — state 4 stays warn-and-merge, but is visible

Distinct exit status **42** plus a `####`-banner naming the PR, the head, and
what was not checked. The merge and all cleanup still happen first, so the code
reports the outcome without changing it. Recorded in the script's usage header
(a full exit-code table: 0/1/2/42), `AGENTS.md` § Merge verification, the
Script Reference row, and `make help` (which notes `make merge-pr` surfaces it
as `Error 42`). Two tests pin it: unverified → 42 after cleanup completes,
attested → 0.

### Operator decision B — ADR-0018 decision 1 implemented in full

The attestation is now consulted for **every** project-repo PR, not only repos
with no workflows. The rollup is classified per entry (CheckRun
`status`/`conclusion`, StatusContext `state`) instead of merely counted, and the
precedence is explicit in `gate_on_hosted_checks()`:

| At the head | behavior |
|---|---|
| failing | **refuse**, attestation or not |
| pending + full-scope attestation (project repo) | merge on it, no wait |
| pending, no attestation | wait and gate (unchanged) |
| passing | merge (unchanged) |
| unrecognized state | wait — fail closed, never shortcut |

The workspace repo is excluded from every attestation path (decision 4), now
identified by GitHub slug as well as by path so a worktree outside
`.workspace-worktrees/` cannot lose the exemption. Five new tests pin the
precedence; the red-plus-valid-attestation case (which must refuse) is the one
that would turn the widening into a loophole, and a mutation allowing the
attestation to answer a red signal fails it.

**Where ADR-0018 does not settle the case — flagged, not chosen silently.**
Decision 1 says an attestation lets a project-repo PR merge "without waiting for
hosted Actions" and is silent on *failing* checks; decision 3 calls hosted CI on
project repos "a mirror and backstop — never a blocker", which read literally
would permit merging past red. The ADR never addresses attestation-plus-red
directly. AGENTS.md's standing rule ("never merge past a red signal from
whichever verification applies") settles it as **refuse**, which is what the
operator directed and what shipped; `AGENTS.md` § Merge verification states the
tension and the resolution, and confines decision 3's "never a blocker" to
post-merge failures and to waiting. If that reading is wrong, the fix is an ADR
amendment, not a code change.

### Suggestions taken (13 of 14, plus one upgraded)

`statusCheckRollup` must be an array (a missing key no longer reads as an empty
one); `upstream.repos` entry names re-validated consumer-side against
`^[A-Za-z0-9_-]+$` so an entry of `.*` cannot satisfy completeness;
`MERGE_PR_SETTLE_*` validated as integers (a test asserts `1$(touch ...)` cannot
execute); the workspace repo identified by slug as well as path;
`--match-head-commit "$HEAD_SHA"` on the merge; git's stderr surfaced on a note
push failure; the poll count no longer undercounts by one; helper tests added
for head-absent-without-upstream-lines and the cross-record `pass`/`scope: full`
split; `Makefile` help and the script header now say `--no-wait` skips the
attestation lookup and push too; the test-file header no longer claims the CI
block is unexercised; the ADR addendum moved; the "honest third state" wording
gone with the rewrite; the plan synced (scratch-ref trap → per-call name,
recourse path, widened `--no-wait`, and the decision-1 widening).

The fourteenth — a `.github/workflows` directory holding nothing runnable —
was **fixed rather than documented**: the listing must contain a `.yml`/`.yaml`
entry, otherwise the repo is treated as having no CI (an unparseable listing
still counts as CI: fail closed). A `workflow_dispatch`-only file remains
unresolvable from the listing alone and is left as a human call, said so in the
code.

### One fail-open found while implementing, not in the review

`check_rollup_state`'s `exit 1` on a `gh` failure ran inside `$( )` — a
subshell — so it killed only the subshell and the caller carried on with an
empty classification. The status is now propagated with `|| exit 1` and a
non-numeric count fails closed. Caught by the existing gh-failure regression
test, which failed the moment the rewrite landed.

### Verification

- `make test-scripts`, fresh: **22 shell test files ✅, 73 pytest tests passed,
  0 failures** — `test_merge_pr.sh` **36/36** (was 24) and
  `test_ci_verification_helpers.sh` **23/23** (was 15).
- `make validate`, fresh: **PASSED** (all four layer-sourcing checks green,
  including the Check-3 baked-chain check that warned in the previous round).
- `shellcheck --severity=warning` clean on both scripts and both test files;
  pre-commit hooks ran on every commit, no `--no-verify`.
- **Mutation-checked, 9 guards, each failing under the specific bug it
  targets**: constant scratch ref (concurrency test 2/10 lost);
  head-absent fail-open (attests a note it cannot check); dropped entry-name
  validation (`.*` attests); attestation allowed to answer a red signal;
  dropped control probe (permission-404 merges); dropped head-sha validation;
  dropped exit 42; dropped settle validation; gh-failure status not propagated;
  recourse naming the main checkout; and directory-existence taken as CI. Bare
  note push mutation fails all three publication tests.
- Fixtures leave nothing behind: `layers/main/citest_ws` is absent after the
  run, and the workspace was never synced (the `make` stub logged it instead).

### Actions
- [x] All 7 must-fix findings closed — `.agent/scripts/_ci_verification_helpers.sh`, `.agent/scripts/merge_pr.sh`, `AGENTS.md`, `.agent/scripts/tests/test_merge_pr.sh`
- [x] Operator decision A implemented (exit 42 + banner, recorded in the script header, AGENTS.md, the Script Reference row, and `make help`)
- [x] Operator decision B implemented (ADR-0018 decision 1 in full, with red-refuses precedence explicit in code and AGENTS.md)
- [x] All 14 suggestions taken (one upgraded from a documented caveat to a fix)
- [ ] Human content review of the `AGENTS.md` change before merge — it is an instruction file, and the decision-B widening is a policy change riding on a bug-fix branch
- [ ] Hosted checks must be green before merging this PR (workspace repo, ADR-0018 decision 4)

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-08-24 15:23 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-610 at `b779168`
**Mode**: pre-push
**Depth**: Deep (reason: merge gate for every repo in the workspace; a policy widening plus an instruction-file change)
**Must-fix**: 5 | **Suggestions**: 9
**Round**: 2 | **Ship**: continue — round 1's seven are genuinely closed and the precedence is right where it was tested, but an independent read and a fresh repro both landed on the same new correctness bug in `ci_local_push_attestation` (finding 1), and two documented behaviours do not match the code.

Specialists: static analysis (shellcheck --severity=warning clean on both scripts and both test files; `make test-scripts` 22 shell files + 73 pytest, 0 failures, `test_merge_pr.sh` 36/36 and `test_ci_verification_helpers.sh` 23/23 as reported; `make validate` PASSED, all four layer-sourcing checks green including Check 3, which warned last round), governance + plan drift (plan is in sync — the decision-B widening is recorded at plan.md:65-75), two disjoint-lens Claude adversarial passes. Copilot and local-model passes not run (per dispatch).

**Mutation spot-checks, run rather than accepted (6 of the reported 11):** attestation allowed to answer a red signal → fails "attestation cannot override red hosted checks" (35/36); `MERGE_UNVERIFIED_EXIT=0` → fails "unverified merge exits 42" (35/36); control probe removed → fails "unconfirmable 404 fails closed" (35/36); `upstream.repos` entry-name validation removed → fails "regex-metacharacter entry rejected" (22/23); head-absent fail-closed neutered → fails two #577 tests (21/23). **One did not reproduce**: removing `|| exit 1` from both `check_rollup_state` call sites leaves the suite fully green (36/36) — under `set -eo pipefail` the failing assignment already aborts, so the guard is defence-in-depth, not the fix, and the comment at merge_pr.sh:438-441 teaches a rule that does not hold here.

### Findings
- [ ] (must-fix) `ci_local_push_attestation`'s `notes merge -s cat_sort_uniq` is a LINE-level union: it sorts and de-dupes lines, destroying the `---` record framing `_ci_local_note_accepts` parses. Two records that each correctly FAIL the gate (a pre-#577 `pass`+`scope: full` note with no `upstream-repo:` lines, and a current `pass (partial)`+`scope: partial` note that has them) blend into one that PASSES — reproduced end-to-end against the real helper, and independently reproduced by the adversarial pass. The blended note is then pushed to origin, so the forged evidence propagates to every clone; the blend is repo-wide over the whole notes tree, so a legitimate merge of head H1 can forge H2's note. Even without forgery, `date:`/`host:`/`image-id:`/`log-sha256:` become multi-valued and un-pairable — the published artifact ADR-0018 decision 5 exists to create is unauditable. The comment at line 236-238 ("union … never drops a record from either side") is inaccurate, the same defective command is printed to the operator as the manual recourse, and the test that claims to prove it ("both sides' records survive") puts the two notes on DIFFERENT commits, where `cat_sort_uniq` never fires — `.agent/scripts/_ci_verification_helpers.sh:245-250`, `merge_pr.sh:651-653`, `tests/test_ci_verification_helpers.sh:284-312`
- [ ] (must-fix) AGENTS.md contradicts itself and the code on the feature's headline case: the § Merging bullet says the attestation "satisfies the gate without waiting for hosted Actions — whether or not the repo also has workflows" (and merge_pr.sh:42-43 repeats it), but when workflows exist at the head and no checks have registered, the code settles, re-polls and REFUSES without ever calling `attestation_available`. That is precisely the run-ci_local-then-push-then-merge flow the widening was for. AGENTS.md's own table row states the refusal correctly. Either consult the attestation before that refusal or narrow both over-claims; no test combines `STUB_WORKFLOWS=listing` with an attestation, so either behaviour passes today — `AGENTS.md:447-450`, `merge_pr.sh:42-43,596-614`
- [ ] (must-fix) TOCTOU between the two `gh pr view` calls: `statusCheckRollup` is read at line 442, `headRefOid` at line 452. `--match-head-commit "$HEAD_SHA"` therefore pins the merge to the head from the SECOND call while the authorisation came from the FIRST — a push landing in between yields `passing` describing the old head and merges the new one, and gh matches happily. The settle loop re-polls the rollup but never re-reads `HEAD_SHA`, widening the window to `SETTLE_ATTEMPTS × SETTLE_SECONDS`. The comment at line 664 ("pins the merge to the commit that was actually verified") is false for the hosted-check path. Fix: one `--json statusCheckRollup,headRefOid` call, re-read together in the settle loop — `.agent/scripts/merge_pr.sh:442,452,585,664-668`
- [ ] (must-fix) `--no-wait` merges with nothing verified — skipping the RED gate as well as the wait — and exits **0**, while AGENTS.md's exit-code paragraph states "Exit `0` means verified" and the § Merging bullet calls the no-CI state "the one case where it merges on a warning". `make merge-pr NO_WAIT=1` is the easiest route to an unverified merge and it reads as an ordinary success, which is exactly what decision A's exit 42 exists to prevent. Either set `MERGED_UNVERIFIED=true` under `--no-wait` (or a third code) or correct both AGENTS.md claims; no test covers `--no-wait`'s exit status — `.agent/scripts/merge_pr.sh:436`, `AGENTS.md:453-456,498-505`
- [ ] (must-fix) The settle-knob validation error names variables that do not exist: `MERGE_PR_${v#SETTLE_}` renders as `MERGE_PR_ATTEMPTS` / `MERGE_PR_SECONDS`; the real knobs are `MERGE_PR_SETTLE_ATTEMPTS` / `MERGE_PR_SETTLE_SECONDS`. Same class as round 1's must-fix 5 (a recourse the operator cannot follow). The test greps only "must be a non-negative integer", so it sidesteps the name — `.agent/scripts/merge_pr.sh:370`
- [ ] (suggestion) `StatusState.EXPECTED` (a required status not yet reported) is classified as **pass**, contradicting the block's own "anything we do not recognize is waited on" invariant. Not a fail-open today only because `gate_on_hosted_checks` does not actually return early on `passing` — contrary to its comment at line 500 — but it becomes one the moment someone "optimises" that path. Map `EXPECTED` to pending — `.agent/scripts/merge_pr.sh:403,500`
- [ ] (suggestion) A failing `make -C "$ROOT_DIR" sync`, or a failing worktree removal, aborts under `set -e` before the MERGED-WITHOUT-VERIFICATION banner and the 42 exit — the only record of the state is lost in the one case where cleanup went wrong. Emit the banner immediately after the merge succeeds, not only at the end — `.agent/scripts/merge_pr.sh:694-703,717,728-738`
- [ ] (suggestion) The AGENTS.md precedence table's "workflows exist but no checks registered" row says "Short re-poll, then **refuses**" and never states that checks appearing DURING the re-poll are gated on per the rows above. This is the unfinished half of round 1's must-fix 6 (the § Merging bullets were fixed well; the table row was not) — `AGENTS.md:491`
- [ ] (suggestion) The attestation is unauthenticated evidence: `_ci_local_note_accepts` checks two literal lines, and `image-id:`/`log-sha256:`/`host:` and the notes-commit committer (`ci_local <ci-local@localhost>`) are recorded but never verified. `git notes --ref=ci-local add -m $'ci-local: pass\nscope: full' <sha>` satisfies the gate with no container run, and `refs/notes/*` is outside branch protection. The realistic failure is an agent under pressure "fixing" a red gate, the same class AGENTS.md § Never already legislates against. Worth an explicit line in ADR-0018 and a cheap committer/`log-sha256` well-formedness check — `.agent/scripts/_ci_verification_helpers.sh:85-92`
- [ ] (suggestion) A kill between the scratch-ref `fetch` and its `update-ref -d` leaves `refs/notes/ci-local-merge-check-*` behind permanently, in both the read and the push helper; nothing prunes them. Harmless per-ref (the `+` refspec and per-call naming keep them inert) but unbounded. Add a `trap`, or sweep stale `${CI_LOCAL_SCRATCH_REF_PREFIX}-*` refs at entry — `.agent/scripts/_ci_verification_helpers.sh:152-159,245-250`
- [ ] (suggestion) AGENTS.md's "Precedence beyond the ADR" paragraph overstates the tension: decision 3's own second sentence already scopes "never a blocker" to POST-merge triage, so the literal reading it warns about is one the ADR itself rules out. The genuine silence is in decision 1, which permits merging on an attestation "without waiting" and says nothing about failing checks. Name decision 1 as the actual gap — `AGENTS.md:507-515`
- [ ] (suggestion) The manual recourse printed on a note-push failure repeats the defective `cat_sort_uniq` merge (finding 1) and leaves `refs/notes/origin-ci-local` behind — `.agent/scripts/merge_pr.sh:651-653`
- [ ] (suggestion) A note-push failure warns on stderr and the run still exits 0, so a merge whose evidence was never published is indistinguishable from one whose was. Given decision A just established a distinct code for "nothing verified this", consider the same treatment for "verified but unpublished" — `.agent/scripts/merge_pr.sh:646-655`
- [ ] (suggestion) AGENTS.md:470-472 still tells a human to publish with a bare `git push origin refs/notes/ci-local` — the exact command `ci_local_push_attestation`'s own comment says fails in both normal states. Point at the helper instead — `AGENTS.md:470-472`
- [ ] (suggestion) Unpinned coverage: no merge_pr-level case for the `unknown` rollup state (so AGENTS.md's fail-closed row is unenforced); the helper header claims it is safe to source under `set -eo pipefail` but the test harness runs `set -uo pipefail` (verified sound manually, but a future edit can break it silently); merge_pr fixtures never carry `upstream.repos` — `.agent/scripts/tests/`

### For the operator (a decision, not a defect)
- [ ] **Does the decision-B precedence need an ADR, not just an AGENTS.md paragraph?** My judgement: yes, as a follow-up issue — not a blocker on this PR. The behaviour that shipped (red refuses, attestation or not) is right, and AGENTS.md states the tension honestly rather than papering over it. But ADR-0012 permits addendums *because they are navigational*; substantive change requires superseding — which the plan reasoned through correctly (plan.md:284-308) and the addendum honours. The consequence is that the narrowing of ADR-0018 decision 1 now lives ONLY in AGENTS.md, while decision 1 in the ADR still reads unconditional. A future reader of ADR-0018 alone gets the wrong rule. The instrument that fits is a small amending/superseding ADR recording "an attestation never overrides a red hosted signal", filed as its own issue.
