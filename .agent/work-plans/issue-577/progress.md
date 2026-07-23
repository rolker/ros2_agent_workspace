---
issue: 577
---

# Issue #577 — ci_local.sh: support upstream.repos source dependencies (blocks ADR-0018 attestation for repos like cube_bathymetry)

## Issue Review
**Status**: complete
**When**: 2026-07-22 18:57 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #577
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: needs-more-detail

### Findings

**Confirmed against source.** Read `.agent/scripts/ci_local.sh` (295 lines):
it mounts only the target repo into `/ci/src/<repo_name>`, runs
`rosdep install --from-paths src` and `colcon build --packages-up-to` against
that single-repo source tree, with zero handling of `<repo>/upstream.repos`.
The issue's reproduction (rosdep-key resolution failure on
`marine_mbes_backscatter_store`, then a CMake `find_package(marine_autonomy)`
failure) is exactly the failure mode the current script's single-repo mount
would produce. Confirmed `layers/main/sensors_ws/src/cube_bathymetry/upstream.repos`
exists and lists `unh_marine_autonomy`, `mru_transform`, `geographic_info` as
floating-branch (`version: jazzy`) git deps. A repo-wide search of currently
checked-out project repos found no other `upstream.repos` file — cube_bathymetry
is the only affected repo today, but the mechanism is correctly scoped as
generic workspace infra (ADR-0003), not a cube-specific patch.

**cube_bathymetry's actual hosted CI is industrial_ci, not the workspace's
generic `ci_workflow.yml` template.** `.github/workflows/ci.yml` uses
`ros-industrial/industrial_ci@master` with `UPSTREAM_WORKSPACE`,
`ROSDEP_SKIP_KEYS` (prunes `marine_nav_interfaces`/`marine_nav_tasks` from
rosdep resolution even though the packages are never built), and
`AFTER_SETUP_UPSTREAM_WORKSPACE` (touches `COLCON_IGNORE` in three upstream
subpackages to scope the monorepo build to what cube actually needs). This
means the issue's option 2 ("delegate to the repo's own industrial_ci
config") is not a thin wrapper — it requires either parsing/replicating these
env vars generically, or cube_bathymetry-specific logic that would violate
the project-agnostic-workspace principle (ADR-0003). Option 1 (`vcs import`
+ generic build) is simpler and stays generic, but as written would not
reproduce the pruning behavior — an unpruned build may fail or diverge from
what hosted CI actually verifies. The issue text already flags this
("honoring per-repo pruning hooks if feasible") but doesn't resolve it; the
script already has a generic per-repo hook precedent
(`<repo>/.agents/ci_local_extra.sh`) that plan-task should consider extending
to an equivalent upstream-side hook, rather than inventing industrial_ci-env
parsing.

**Attestation semantics gap (ADR-0018).** The `upstream.repos` entries here
pin to floating branches (`version: jazzy`), not SHAs. The current
`ci-local: pass` note format records `repo`/`commit`/`image`/`packages`/`scope`
but nothing about resolved dependency state. An attestation built by cloning
a floating upstream branch is not reproducible from the note alone — a repeat
run on the same target-repo commit could silently build against a different
`unh_marine_autonomy` state. ADR-0018 decision 1 treats a `scope: full`
`ci-local: pass` record as sufficient standalone merge evidence; that
guarantee weakens once "full" build state includes an unpinned external input
the note doesn't capture. Recommend the plan record resolved upstream SHAs in
the note (e.g. an `upstream:` field per repo/commit) so the attestation stays
self-describing, and treat this as a note-format extension under ADR-0018
rather than a silent behavior change to what "scope: full" already means to
existing consumers (`merge_pr.sh` future wiring, human readers of
`git notes --ref=ci-local show`).

**Test coverage precedent exists.** `.agent/scripts/tests/test_ci_local.sh`
already stubs `docker`/`vcs`-adjacent calls and drives the script through
dry-run, package-discovery, and injection-safety cases. The upstream.repos
path should get equivalent stub-based coverage (dry-run plan showing the
upstream clone step; injection-safety on repo names/URLs/versions parsed from
`upstream.repos`, mirroring the existing package-name validation) rather than
being exempted as "only exercised against real cube_bathymetry."

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Workspace vs. project separation | OK | Fix belongs in `.agent/scripts/ci_local.sh` (workspace infra); mechanism (`vcs import` on a well-known filename) is generic, not cube-specific |
| Enforcement over documentation | OK | Extends the existing enforced tool (ci_local.sh + attestation), not a docs-only fix |
| A change includes its consequences | Action needed | ADR-0018's attestation note format (and its "scope: full" guarantee) needs an explicit extension for upstream dependency state — see Findings above |
| Test what breaks | Action needed | New `upstream.repos` code path needs stub-based tests in `test_ci_local.sh` (dry-run + injection-safety), mirroring existing coverage patterns |
| Only what's needed / Improve incrementally | Watch | Issue proposes two alternative implementation strategies (vcs-import vs. industrial_ci delegation) and leaves the full-scope/caching question open by design — plan-task should pick the minimal option (vcs-import, reusing the `ci_local_extra.sh` hook precedent for pruning) rather than replicating industrial_ci's env-var surface |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| 0003 — Project-agnostic workspace | Yes | Fix must stay generic (any repo with `upstream.repos`); option 2 (industrial_ci delegation) risks cube-specific logic leaking into workspace infra unless generalized |
| 0018 — Local-first CI verification | Yes | This closes a real gap in ADR-0018's stated Phase-1 tool; attestation note format likely needs a documented extension (see Findings) — plan-task should decide whether that's a note change under this ADR or needs a superseding addendum |
| 0009 — Python package management | Watch | `vcs import` may need `vcs`/`vcstool` available in the container image; confirm it's already present in the agent sandbox / rosdep-baked image rather than requiring an ad hoc pip install |

### Consequences

- ADR-0018 doc itself may need a Consequences-section note once the note format is extended (per Findings).
- `AGENTS.md`'s `ci_local.sh` script-reference line (currently silent on upstream.repos) should be updated to mention the new capability once implemented.
- `test_ci_local.sh` needs new cases (see Findings).

### Actions
- [ ] Decide implementation strategy: vcs-import (generic, reuses `ci_local_extra.sh`-style hook precedent) vs. delegating to per-repo industrial_ci config (risks project-specific logic in workspace infra) — recommend vcs-import as the minimal, ADR-0003-aligned choice.
- [ ] Design how per-repo pruning (cube's `ROSDEP_SKIP_KEYS`/`AFTER_SETUP_UPSTREAM_WORKSPACE`-equivalent) is expressed generically, likely via an upstream-side hook analogous to `ci_local_extra.sh`.
- [ ] Extend the `ci-local` git-note format to record resolved upstream dependency state (repo/commit per `upstream.repos` entry) so `scope: full` attestations stay self-describing per ADR-0018's merge-evidence guarantee.
- [ ] Add stub-based tests to `test_ci_local.sh` for the new upstream.repos path (dry-run plan output, injection-safety on parsed repo name/URL/version fields).
- [ ] Confirm `vcs`/`vcstool` availability in the agent sandbox / clean-room images (ADR-0009 dev-tool provisioning, not bare pip).
- [ ] Update `AGENTS.md`'s ci_local.sh reference line once the capability lands.

## Plan Authored
**Status**: complete
**When**: 2026-07-22 20:15 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-577/plan.md` at `1dc9627`
**Branch**: feature/issue-577 at `1dc9627`
**Phases**: single

### Open questions
- [ ] Upstream/target workspace layout inside the container: separate underlay build (recommended, mirrors industrial_ci) vs. single merged src/ tree.
- [ ] Whether to cache the built upstream workspace across repeated runs (recommend deferring to a follow-up issue; this PR lands correctness first).

## Plan Review
**Status**: complete
**When**: 2026-07-22 17:04 -04:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-577/plan.md` at `1dc9627`
**PR**: PR-less (--issue mode; feature/issue-577)
**Verdict**: approve-with-suggestions

### Evaluation
| Dimension | Verdict | Notes |
|---|---|---|
| Scope | Good | 4 files (ci_local.sh + its help text, test_ci_local.sh, ADR-0018, AGENTS.md); additive and backward-compatible. Well under split threshold. |
| Issue alignment | Good | Implements vcs-import strategy (settled), pruning via hooks, and resolved-SHA note extension (settled in-scope). Both operator checkpoint decisions honored. |
| review-issue alignment | Good | All six review-issue action items addressed: strategy choice, pruning-hook design, note-format extension, stub tests, vcstool availability, AGENTS.md line. |
| File targeting | Good | Correct files. cube_bathymetry confirmed the only repo with `upstream.repos`; no cube-specific change bundled (correct — follow-up). Verified against source: single-repo mount + `ci_local_extra.sh` hook precedent both exist as the plan describes. |
| Consequences | Good | Consequences table covers ADR-0018 doc, AGENTS.md, tests, cube follow-up, and wall-clock tradeoff. Nothing material missing. |
| Principle alignment | Good | Filename-keyed generic mechanism (ADR-0003), enforcement-over-docs, tests included, incremental. Two-hook design justified by demonstrated industrial_ci parity (not speculative). |
| ADR compliance | Good | 0003, 0018, 0009 all triggered and addressed. Note-format change framed as an extension, not a redefinition — preserves ADR-0018 Decision 1 merge-evidence guarantee. |
| ROS conventions | N/A | Workspace-infra plan (shell + docker + vcstool); no ROS package code. |

### Findings
- [ ] (suggestion) Attestation integrity — make explicit that for attestable runs the `vcs import` consumes `upstream.repos` **from the pristine snapshot** (it is committed content, already in the `git archive HEAD` tree), not a host-rewritten copy. Host-side parse stays validation/dry-run only. The plan floats both options and the step-4→step-7 cross-reference is muddled (`plan.md:68-69` points at "step 7" which is dry-run output, not the snapshot boundary). Reading from the snapshot keeps the "tests exactly the commit content" guarantee intact. — `plan.md:65-79`
- [ ] (suggestion) Consider capturing resolved upstream SHAs via a parseable stdout marker in the inner script (extracted by the host from the tee'd `$LOG`) instead of a new read-write bind mount. This preserves the current all-mounts-read-only invariant, and the SHAs then fall under the existing `log-sha256` integrity hash for free. If the rw mount is kept, the plan's hard-fail-on-missing handling is the right call. — `plan.md:81-89`
- [ ] (suggestion) The underlay-vs-merged-src layout open question is the highest-impact implementation decision (it determines whether `--packages-up-to` target semantics stay unchanged). Recommend committing to the plan's own recommended underlay approach before implementation rather than deferring, since it shapes the rosdep/build/source wiring in steps 4. — `plan.md:204-211`
- [ ] (suggestion) Per ADR-0018 Decision 6, `--clean-room` is the recommended image when a change touches dependency declarations/build environment — which `upstream.repos` repos inherently do, and clean-room's `ros-dev-tools` install guarantees `vcstool`. Worth a one-line note in the ADR-0018 doc update (step 8) that clean-room is preferred for `upstream.repos` repos, resolving the ADR-0009 vcstool-availability watch in the same stroke. — `plan.md:190`
- [ ] (nit) Files-to-Change table lists `.agent/scripts/ci_local.sh` twice (logic row + help-text row). Intentional (separates the header-comment doc change) but reads as a duplicate; fold or annotate. — `plan.md:160,164`

### Summary
Strong, source-verified plan that addresses every review-issue finding and both settled operator decisions. It is ready for implementation. The suggestions are refinements — two of them (read `upstream.repos` from the snapshot; capture SHAs via a log marker rather than a rw mount) touch the ADR-0018 attestation-integrity guarantee and are worth resolving as the implementer wires up steps 4–5, but none block starting.

### Recommended Actions
- [ ] During implementation, resolve step-4/step-5 so upstream.repos is read from the pristine snapshot and resolved-SHA capture preserves attestation integrity (log marker preferred over rw mount).
- [ ] Commit to the underlay layout (plan's own recommendation) before writing the inner-script changes.
- [ ] Add the `--clean-room`-preferred-for-upstream.repos note to the ADR-0018 update.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-22 17:22 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-577 at `1345aec`
**Mode**: pre-push
**Depth**: Deep (reason: command-injection surface parsing upstream.repos + shelling out to a generated container script; substantive ADR-0018 edit)
**Must-fix**: 0 | **Suggestions**: 3
**Round**: 1 | **Ship**: recommended — no must-fix; static analysis clean, 75/75 tests pass, attestation-integrity design sound

### Findings
- [ ] (suggestion) Re-validate the `git ls-remote`-resolved SHA against `^[0-9a-f]{40}$` before interpolating into the container `git checkout` and the `upstream-repo:` note line (defense-in-depth; also handles annotated-tag object-vs-peeled-commit) — `.agent/scripts/ci_local.sh:299-301`
- [ ] (suggestion) Record `ci_local_rosdep_skip_keys.txt` usage in the persistent `steps:` note field (only dry-run surfaces it today), so a skip-keys-pruned rosdep resolution is reflected in the merge evidence — `.agent/scripts/ci_local.sh:248-263`
- [ ] (suggestion) Header doc comment frames skip-keys as upstream-only pruning, but the code applies it unconditionally (unlike the UPSTREAM_PRESENT-gated hook); reconcile comment vs. gating — `.agent/scripts/ci_local.sh:44-46,248`

## Integrated Review
**Status**: complete
**When**: 2026-07-23 08:46 -04:00
**By**: Claude Code Agent (Claude Fable 5)

**PR**: #578 at `6f747b6`
**Sources**: 3 (Copilot R1 @ `2c7d53b`, Copilot R2 @ `6f747b6`, Local Review (Pre-Push) @ `1345aec`, CI rollup)
**Cross-source confirmations**: 0
**CI**: all-pass (after `6f747b6` added pyyaml to the script-tests job — setup-python lacks the yaml module the upstream.repos parser imports)

### Findings
- [ ] (minor, Copilot R1) `upstream.repos` parser defaults a missing `type` to `git` (`ent.get("type", "git")`), contradicting the documented "requires `type: git`" contract and weakening fail-loud validation — require an explicit `type: git`, error when absent; add a rejection test — `.agent/scripts/ci_local.sh:226`
- [ ] (minor, Copilot R2) host-side `git ls-remote` lacks the `--` separator, so a charset-valid URL beginning with `-` would parse as an option; the generated container-side `git clone --` is already guarded — add `--` for consistency of the injection-hardening model — `.agent/scripts/ci_local.sh:315`

### False positives
- none — both bot comments verified valid against current head

### Notes
- The 3 Local Review (Pre-Push) suggestions @ `1345aec` were all applied in `2c7d53b` (SHA re-validation + peeled-tag preference; persistent rosdep-skip-keys note line + steps token; header comment reconciliation) — closed, not re-raised by any GitHub-side source.
