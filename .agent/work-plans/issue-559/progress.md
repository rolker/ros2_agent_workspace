---
issue: 559
---

# Issue #559 — Fix O(N²) layer sourcing and inverted overlay precedence in setup.bash

## Issue Review
**Status**: complete
**When**: 2026-07-14 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #559
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Summary

The issue identifies two correctness bugs and one performance problem in
`.agent/scripts/setup.bash` and `.agent/scripts/build.sh`, with benchmarks,
root-cause analysis, and a concrete five-step fix. The root causes are verified
by reading the actual scripts:

- **`setup.bash:86`**: sources `install/setup.bash` (baked chained version)
  instead of `install/local_setup.bash`, causing O(N²) Python invocations (one
  full chain expansion per layer). Confirmed in code.
- **`build.sh:78`**: `source "$SCRIPT_DIR/setup.bash" > /dev/null` before
  building any layer loads the full ROS 2 prefix path, so when colcon builds
  `underlay_ws` first, every other layer is already present as a parent — baked
  into `underlay_ws/install/setup.sh` permanently. Confirmed in code.
- **Inverted precedence**: result of sourcing in ascending order with chained
  setup; the fast alternative produces correct canonical order.

### Scope Assessment

**Well-scoped?** Yes — change is bounded to `.agent/scripts/` (setup.bash,
build.sh, worktree_create.sh templates) plus a one-time clean rebuild. Single PR.
**Right repo?** Yes — workspace infrastructure, not project-repo content.
**Dependencies?** None identified. A one-time clean rebuild is a manual step (not
a script dependency), documented in the issue.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | OK | Issue documents root cause, benchmarks, and expected outcomes clearly |
| Enforcement over documentation | OK | Fix is behavioral, not documentation-only |
| Capture decisions, not just implementations | Watch | Switching from `install/setup.bash` to `install/local_setup.bash` (runtime vs baked chaining) is a design choice worth an ADR; it isn't currently documented in `docs/decisions/` |
| A change includes its consequences | Watch | Issue mentions sweeping `.agent/` for other `install/setup.bash` references and updating generated worktree scripts; existing worktrees age out. Implementer should track the sweep explicitly — grep result should appear in the PR. `WORKTREE_GUIDE.md` may need a note about the generated-script change |
| Only what's needed | OK | Solves concrete, measured pain (4.5s → 0.5s per agent command); no speculative scope |
| Improve incrementally | OK | Five-step plan is bounded; step 3 (worktree generators) is a natural scope inclusion to prevent regressions on new worktrees |
| Test what breaks | Watch | Verification section lists manual timing/diff steps, not automated tests. The timing regression would be easy to catch with a timed `source` in CI; the precedence correctness could be checked with a script. Consider adding at least one automated check |
| Workspace vs. project separation | OK | All changes in workspace infra scripts |
| Workspace improvements cascade to projects | OK | Generated worktree scripts get the fix; existing worktrees noted as aging out naturally |
| Primary framework first, portability where free | OK | No framework coupling in these shell scripts |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| 0001 — Adopt ADRs | Yes | The switch from baked-chain (`setup.bash`) to runtime-chain (`local_setup.bash`) sourcing is a design decision that future agents need to understand — should be recorded |
| 0002 — Worktree isolation | OK | Standard; use worktree for implementation |
| 0007 — Retain Make with Dependency Tracking | Watch | `build.sh` change should not break Make's stamp-file dependency semantics; verify `make build` still works correctly end-to-end |
| 0013 — progress.md entry-type vocabulary | OK | This entry follows the Issue Review type per ADR-0013 |

### Consequences

Per the consequences map, changing `.agent/scripts/` (setup.bash, build.sh,
worktree_create.sh) requires:
- **AGENTS.md script reference table** — descriptions reference script purpose
  only, not implementation; minor note may still be warranted if the semantics
  change visibly (e.g., "sources `local_setup.bash` per layer").
- **`.agent/WORKTREE_GUIDE.md`** — documents generated worktree scripts; if the
  generated preambles change, the guide should reflect it.
- **Sweep result** — grep for direct `install/setup.bash` sourcing across
  `.agent/` (container/dispatch flows, other hooks) should be documented in the
  PR; this is listed in the issue but not tracked as an explicit checklist item.

### Actions
- [ ] Record the runtime-vs-baked chaining design decision in an ADR (ADR-0001 trigger — strongly recommended before or alongside implementation to prevent accidental reversion by a future agent).
- [ ] Sweep `.agent/` for all direct `install/setup.bash` sourcing; document findings in PR description.
- [ ] Update `.agent/WORKTREE_GUIDE.md` if generated worktree script preambles change.
- [ ] Consider adding an automated timing/correctness check (e.g., timed `source` assertion in CI or a Makefile validate target) to prevent O(N²) regression.

## Plan Authored
**Status**: complete
**When**: 2026-07-14 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-559/plan.md` at `057f99a`
**Branch**: feature/issue-559 at `057f99a`
**Phases**: single

### Open questions
- [ ] No open questions — plan is review-plan-ready.

## Plan Review
**Status**: complete
**When**: 2026-07-14 13:02 +00:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-559/plan.md` at `057f99a`
**PR**: PR-less (`--issue 559`, host-dispatched sub-agent, no GitHub auth)
**Verdict**: changes-requested

### Findings
- [ ] (must-fix) `build.sh` step 2 must retain the ROS 2 base source. Line 78's `source "$SCRIPT_DIR/setup.bash"` currently supplies both the `LAYERS` array *and* `/opt/ros/jazzy/setup.bash`. The plan replaces it with "a targeted parse of `layers.txt` to populate the `LAYERS` array" only — a literal implementation drops the jazzy base, so the first `colcon build` (underlay) has no ROS environment and `make build` breaks. The replacement must still source `/opt/ros/jazzy/setup.bash` (the base only, NOT the layer overlays — sourcing overlays pre-build is the pollution bug being fixed). — `plan.md:33-37`
- [ ] (suggestion) Sweep-completeness claim is overstated. Step 10 says the grep "finds all remaining `install/setup.bash` references in `.agent/`" and "No other callers need updating," but the sweep also hits `.agent/templates/ci_workflow.yml:76` and `.agent/knowledge/ros2_development_patterns.md:154-155`. Both are legitimately out of scope (ci_workflow sources a single built `ws`, not layered chaining; the knowledge doc is illustrative), but the plan should explicitly triage-and-exclude them with a reason rather than assert they don't exist. — `plan.md:66-69`
- [ ] (suggestion) `ros2_development_patterns.md:154-155` documents sourcing each layer's `install/setup.bash` in ascending order — the exact O(N²)/inverted-precedence anti-pattern being fixed. Consider updating it to `local_setup.bash` (with an explicit `source /opt/ros/jazzy/setup.bash` first) or noting it's illustrative-only. Minor; acceptable as a follow-up.
- [ ] (suggestion) review-issue Action #4 (automated timing/correctness regression check) is unaddressed. The plan's "Test what breaks" row lists only manual verification. Adopt a lightweight check (timed `source` assertion or a `validate` target) or explicitly defer it with a reason — right now it is silently dropped.

### Notes on verified strengths
- All plan line numbers confirmed against source: `setup.bash:86`, `build.sh:78`/`126-127`, `worktree_create.sh:130`/`132`/`279-281`, `test.sh:97-98`, `verify_change.sh:63-64`.
- ADR-0016 is the correct next number; ADR-0001 trigger is valid.
- Generated worktree `setup.bash` (worktree_create.sh:91) and the per-layer build preamble (worktree_create.sh:269) already source the jazzy base explicitly, so switching *their* layer sourcing to `local_setup.bash` is safe — no ROS-base regression there.
- `dashboard.sh:197` correctly excluded (existence check only).
- review-issue Actions #1 (ADR → step 7) and #3 (WORKTREE_GUIDE → step 8) are addressed.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-14 14:21 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-559 at `080bd75`
**Mode**: pre-push
**Depth**: Deep (reason: ADR addition + 585 lines >200 + 12 files >10)
**Must-fix**: 1 | **Suggestions**: 3
**Round**: 1 | **Ship**: continue — one mechanical consequence fix (AGENTS.md table); all else are low-impact robustness suggestions

### Findings
- [x] (must-fix) New script `test_layer_sourcing.sh` not added to AGENTS.md Script Reference table — consequences map mandates it (`principles_review_guide.md:45`); precedent at `AGENTS.md:489` — `AGENTS.md`
- [x] (suggestion) Guard `LAYERS_BASE` not worktree-aware (asymmetric with worktree-aware `LAYERS_CONFIG`); Checks 2-3 silently skip in a layer worktree; sed at :92 is `layers/main`-specific — `.agent/scripts/test_layer_sourcing.sh:30,92` [cross-confirmed: Lens A + Lens B]
- [x] (suggestion) Trailing-whitespace strip diverges: guard strips, `setup.bash`/`build.sh` don't — align all three — `.agent/scripts/setup.bash:76`, `.agent/scripts/build.sh:105`, `.agent/scripts/test_layer_sourcing.sh:74` [cross-confirmed: Lens A + Lens B]
- [x] (suggestion) Check 1 static allowlist is a hardcoded 5-file list; a future baked-chain source escapes the guard — prefer a repo-wide `.agent/scripts/*.sh` glob — `.agent/scripts/test_layer_sourcing.sh:37`

### Notes on verified strengths
- shellcheck `--severity=warning` clean on all 6 changed shell scripts.
- Guard runs in CI: `run_script_tests.sh` auto-discovers `test_*.sh` (glob) → `make test-scripts` (validate.yml:71) runs Check 1; Checks 2-3 skip cleanly with no built layers. Also wired into `make validate`.
- Overlay precedence logic verified against colcon `local_setup` prepend semantics — ADR-0016 claim, Check 2 `EXPECTED_ORDER`, and actual behavior agree.
- Full plan adherence: every planned file changed, no scope creep; `verify_change.sh` deviation (source workspace `setup.bash`, not a naive `local_setup.bash` swap) transparently documented and correct.
- `build.sh` retains the jazzy base pre-build (plan-review must-fix addressed) and sources lower layers progressively post-build; no chain pollution.
- Sweep complete: only other baked-chain source is `ci_workflow.yml:76` (flat project CI template, out of scope); `dashboard.sh:197` is `-f` existence check only.

## Implementation
**Status**: complete
**When**: 2026-07-14 14:33 +00:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-559 at `7902bf3`
**Addressed**: Local Review (Pre-Push) — 2026-07-14 14:21 +00:00, branch at `080bd75`
**Commits**: `d7d0376`, `5eb17f4`, `fb7ebd0`, `7902bf3`

### Actions
- [x] (must-fix) Added `test_layer_sourcing.sh` row to the AGENTS.md Script Reference table (next to the other regression-test scripts) — `AGENTS.md:490` (`d7d0376`)
- [x] (suggestion) Trailing-whitespace strip aligned across all three layers.txt readers: added `sed 's/[[:space:]]*$//'` to `setup.bash` and `build.sh` so a stray trailing space can't yield a bad `<layer> _ws` path — `.agent/scripts/setup.bash:78`, `.agent/scripts/build.sh:105` (`5eb17f4`)
- [x] (suggestion) Made the guard's `LAYERS_BASE` worktree-aware: derive one `MAIN_ROOT` and set both `LAYERS_CONFIG` and `LAYERS_BASE` from it, and pass `ROS2_LAYERS_BASE` into Check 2's scrubbed env so `setup.bash` sources the same built layers (keeps the `/layers/main/` sed correct from a layer worktree instead of silently skipping) — `.agent/scripts/test_layer_sourcing.sh:28-30,61-83,85-100` (`fb7ebd0`)
- [x] (suggestion) Replaced Check 1's hardcoded 5-file allowlist with a repo-wide `.agent/scripts/*.sh`/`*.bash` glob (self-excluded via `-ef`, loose pattern kept so heredoc-generated regressions are still caught) — `.agent/scripts/test_layer_sourcing.sh:36-53` (`7902bf3`)

### Verification
- `shellcheck --severity=warning` clean on `setup.bash`, `build.sh`, `test_layer_sourcing.sh`.
- Guard runs green here (Check 1 passes, Checks 2-3 skip — no built layers, expected in CI); positive-tested that a fresh script with a direct *and* a heredoc-builder baked-chain `source` is flagged (exit 1), confirming self-exclusion doesn't blind the glob.

### Next step
Lifecycle: **Implementation** → **review-code** (re-review the fixes). Hand off to a fresh-context sub-agent:

    .agent/scripts/dispatch_subagent.sh --mode in-process --issue 559 --skill review-code

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-14 14:43 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-559 at `e41b4b6`
**Mode**: pre-push
**Depth**: Deep (reason: new ADR + cross-cutting environment-sourcing change across 6 scripts)
**Must-fix**: 1 | **Suggestions**: 3
**Round**: 2 | **Ship**: recommended — one mechanical one-line env scrub (not rising, obvious correction); a full re-review round isn't warranted for it

### Findings
- [ ] (must-fix) `build.sh` doesn't scrub an inherited `COLCON_PREFIX_PATH`; sourcing jazzy doesn't clear it and `make build` passes it through, so `source setup.bash; make build` (and the ADR's own heal command in a sourced shell) re-bakes higher layers into lower chains — reintroducing ADR problem #3. Add `unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH AMENT_CURRENT_PREFIX` before sourcing jazzy — `.agent/scripts/build.sh:76-90`
- [ ] (suggestion) Fallback `LAYERS` array omits `site` (present in real layers.txt); introduces a second stale copy alongside `setup.bash:82` — sync both — `.agent/scripts/build.sh:110`
- [ ] (suggestion) Check 1 comment says "repo-wide glob" but scan is top-level `.agent/scripts/*.sh` only (misses `tests/`, `.agent/hooks/`); make recursive or narrow the comment — `.agent/scripts/test_layer_sourcing.sh:37-43`
- [ ] (suggestion) Check 2 `uniq` collapses only adjacent duplicates; fragile if a layer ever contributes non-adjacent AMENT prefixes — note or first-occurrence dedup — `.agent/scripts/test_layer_sourcing.sh:105-107`

### Notes on verified strengths
- shellcheck `--severity=warning` clean on all 6 changed shell scripts; guard runs green here (Check 1 pass, Checks 2-3 skip — no built layers).
- All 4 round-1 findings confirmed addressed in the branch diff.
- Two independent adversarial passes (Lens A logic, Lens B systemic) both confirmed runtime-chaining equivalence to a clean baked chain and the real-world pollution it fixes; empirically verified jazzy does not clear `COLCON_PREFIX_PATH` and `make build` inherits it.
- Full plan adherence; `verify_change.sh` deviation documented and correct. Consequences map (AGENTS.md table, WORKTREE_GUIDE, ros2_development_patterns) all addressed. Sweep complete (`ci_workflow.yml:76` out of scope; `dashboard.sh:197` existence check only).

## Implementation
**Status**: complete
**When**: 2026-07-14 10:49 -04:00
**By**: Claude Code Agent (Claude Fable 5)

**Branch**: feature/issue-559 at `9073b0e`
**Addressed**: Local Review (Pre-Push) — 2026-07-14 14:43 +00:00, Round 2 (Ship: recommended — remaining findings host-applied instead of a full round 3)
**Commits**: `9073b0e`

### Actions
- [x] (must-fix) build.sh scrubs inherited COLCON/AMENT/CMAKE prefix paths before sourcing jazzy; verified scrub reproduces pristine-shell env (`9073b0e`)
- [x] (suggestion) fallback LAYERS lists include site, synced build.sh + setup.bash (`9073b0e`)
- [x] (suggestion) guard Check 1 recursive over .agent/scripts + .agent/hooks (`9073b0e`)
- [x] (suggestion) guard Check 2 first-occurrence dedup replaces adjacent-only uniq (`9073b0e`)

## Integrated Review
**Status**: complete
**When**: 2026-07-14 11:05 -04:00
**By**: Claude Code Agent (Claude Fable 5)

**PR**: #565 at `d19d3eb`
**Sources**: 3 (Copilot R1 @ `d19d3eb`, Local Review (Pre-Push) R1 @ `080bd75` + R2 @ `7902bf3`, CI rollup)
**Cross-source confirmations**: 0
**CI**: all-pass (commit identity, script tests, docs validation, lint)

### Findings
(none — Copilot reviewed 13/13 files, generated no comments; both local
pre-push rounds' findings were addressed pre-publish and verified at
`d19d3eb`)

### False positives
(none)
