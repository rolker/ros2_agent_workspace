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
