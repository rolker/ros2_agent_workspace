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
