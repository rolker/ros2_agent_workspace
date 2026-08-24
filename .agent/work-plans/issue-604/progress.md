---
issue: 604
---

# Issue #604 — docker_run_agent.sh: worktree build/install/log anonymous volumes come up root-owned (entrypoint chown missing for #602's new mounts)

## Issue Review
**Status**: complete
**When**: 2026-08-23 23:18 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #604
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Verification against source

Confirmed all three claims in the issue body directly against current code
on this branch (`fdfddbb`, i.e. #602/#603 merged):

- `.agent/scripts/docker_run_agent.sh` section 4b (lines ~391-421) mounts
  anonymous volumes at `"$WORKTREE_PATH"/*_ws/{build,install,log}` — outside
  `layers/main`.
- `.devcontainer/agent/agent-entrypoint.sh`'s ownership loop (lines 29-41)
  iterates only `"$WORKSPACE_ROOT"/layers/main/*_ws`. The entrypoint's only
  other use of `$WORKTREE_ROOT` is the `cd` at line 141-142 — no ownership
  handling for it exists.
- `.agent/scripts/test_layer_sourcing.sh` Check 4 (line 82: `LAYERS_BASE="$MAIN_ROOT/layers/main"`)
  is scoped to `layers/main` only and stats host-side paths, so it structurally
  cannot observe this failure (container-side anonymous-volume ownership,
  worktree-scoped path) — matches the issue's two-reasons analysis.

Root cause and proposed fix (mirror section 4b's mount loop with a second
entrypoint `chown` loop over `$WORKTREE_ROOT/*_ws`, same `[ -d ] && [ ! -L ]`
guard) are correct and consistent with the existing `layers/main` pattern.
The issue's diagnosis of *why* #602 missed this — pre-creating a host
mountpoint via `mkdir` has no bearing on an anonymous volume's ownership,
which is set by the entrypoint `chown`, not the host-side directory beneath
it — is accurate and worth preserving verbatim in the eventual commit
message; it's a subtle Docker mechanic that's easy to re-break.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Enforcement over documentation | Watch | The fix itself is a straightforward code change, but the issue's own "invariant" (every anonymous volume the launcher adds must have a corresponding entrypoint chown) is stated only in prose. Nothing enforces it going forward — see Recommendation below. |
| A change includes its consequences | OK | Issue correctly scopes itself to the ownership half of #602 and explicitly declines to fold in the separate "should a dispatched agent see host-built layer installs" question — good scope discipline. |
| Test what breaks | Action needed | This is a container-only failure mode; `test_layer_sourcing.sh` runs host-side and cannot catch it by construction (confirmed above). The issue proposes two options (container-side dispatch smoke test, or a `--print-mounts`-driven mount↔chown pairing check) but doesn't commit to implementing either. A fix with zero regression coverage for the second time in this exact area (#602 → #604) should not ship without at least the cheaper of the two options. |
| Improve incrementally | OK | Minimal, targeted second loop mirroring existing code; no scope creep. |
| Capture decisions, not just implementations | OK | The "reasoning error worth recording" section is exactly the kind of rationale that should land in the fix commit message, not just the issue. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| 0002 — Worktree isolation | Yes | Bug is specific to worktree-dispatched containers; fix must preserve the `[ -d ] && [ ! -L ]` symlink guard so a layer worktree's untouched `*_ws` siblings (symlinks into `layers/main`) aren't double-chowned or reached-through. |
| 0016 — Runtime vs. baked layer chaining | Watch | Not directly triggered (no layer-chaining semantics change), but `test_layer_sourcing.sh` is the ADR-0016 regression guard and this issue documents a real gap in its coverage — worth a follow-up note in that ADR's guard script's header/scope comment once #604 lands, so the next reader doesn't assume Check 4 is exhaustive. |
| 0004/0005 — Enforcement hierarchy / layered enforcement | Watch | Same point as the principle row: the proposed invariant has no enforcement layer yet (no hook, no CI, no test). Fine to defer as a separate follow-up issue, but should be flagged rather than silently dropped. |

### Consequences

- `AGENTS.md`'s script reference table already describes `test_layer_sourcing.sh` as covering "mountpoint ownership" — if this fix leaves Check 4's blind spot un-widened, that table entry stays slightly overstated. Not blocking, but worth a one-line caveat if the regression-coverage recommendation below isn't picked up in the same PR.
- No package interfaces, params, topics, or docs are affected.

### Recommendations

- Implement the fix as scoped (second entrypoint loop over `$WORKTREE_ROOT/*_ws`), preserving the section-4b guard semantics.
- Fold in *some* regression coverage in the same PR rather than as a pure follow-up — the cheapest option per the issue's own analysis is the `--print-mounts`-driven "every anonymous volume has a matching chown" invariant check, since it doesn't require an actual container run. A container-side smoke test (write into `build/` as the dropped user) is the stronger check but can be a documented follow-up if a live dispatch harness isn't readily available in this PR's scope.
- Preserve the "mkdir-as-invoking-user precaution and fail-loud handling" comment style from section 4b in the new entrypoint loop so future readers get the same rationale that #602's comment gave for the mount side.

### Actions
- [ ] Add the second `chown` loop to `.devcontainer/agent/agent-entrypoint.sh` for `$WORKTREE_ROOT/*_ws/{build,install,log}`, mirroring section 4b's `[ -d ] && [ ! -L ]` guard.
- [ ] Add regression coverage for the mount↔chown pairing (prefer the `--print-mounts`-driven invariant check as the low-cost option in this PR; container-side smoke test may follow as a separate issue if out of scope here).
- [ ] Consider a one-line scope caveat on `test_layer_sourcing.sh`'s header/AGENTS.md table entry noting it does not cover worktree-scoped or container-side ownership, so the next regression in this area isn't assumed-caught again.

## Plan Authored
**Status**: complete
**When**: 2026-08-23 23:20 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-604/plan.md` at `59e8244`
**Branch**: feature/issue-604 at `59e8244`
**Phases**: single

### Open questions
- [ ] Should the container-side smoke test (write into `build/` as the dropped `ros` user, requires a live Docker dispatch harness) be added in this PR instead of deferred as a follow-up, given #602 -> #604 is the second time this area shipped without regression coverage?

## Plan Review
**Status**: complete
**When**: 2026-08-23 23:26 -04:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-604/plan.md` at `59e8244`
**PR**: PR-less (`--issue` mode, fresh-context sub-agent dispatch — not the plan author's context)
**Verdict**: approve-with-suggestions

### Evaluation

| Dimension | Verdict | Notes |
|---|---|---|
| Scope | Good | One entrypoint loop, one new test, two doc touch-ups. Single PR. Correctly declines the separate "should a dispatch see host-built layer installs" question. |
| Issue alignment | Good | Covers all three review-issue actions (fix, regression coverage, Check-4 scope caveat). |
| File targeting | Needs work | Omits `docker_run_agent.sh` section 4b, whose comment is the exact sentence the issue names as the reasoning error (finding 1). Every other path/line ref verified accurate (`AGENTS.md:582`, `test_layer_sourcing.sh` Check 4 header, entrypoint 29-41, 4b ~391-421). |
| Consequences | Good | Table is complete for what the plan does change; gains one row with finding 1. |
| Documentation & instruction impact | Good | Present and non-silent; instruction item correctly framed as a candidate. One inaccuracy (finding 7). |
| Principle alignment | Good | Test-what-breaks and enforcement-over-documentation both answered with an executable check rather than prose. |
| ADR compliance | Good | 0002 symlink guard honoured; 0016 guard's coverage claim corrected rather than left overstated; 0004/0005 satisfied by the new script-layer check. |
| ROS conventions | N/A | Workspace infra plan — no packages, params, topics, or ROS build changes. |

### Findings
- [ ] (must-fix) `docker_run_agent.sh` section 4b's "Mirror section 4's mkdir-as-invoking-user precaution" comment — the sentence the issue calls out as the reasoning error — is not in Files to Change; a future editor reads 4b, not the entrypoint, so the corrective rationale must be cross-referenced there too — `plan.md:90` (Files to Change table)
- [ ] (must-fix) The Open Question's deferral premise is false on this host: `docker info` succeeds and `ros2-agent-workspace-agent:latest` is already built, so the container-side smoke test IS readily available. Re-decide with that fact, and if the test lands, gate it to skip cleanly when no daemon is present — `run_script_tests.sh` advertises its suite as hermetic/CI-safe — `plan.md:139`
- [ ] (suggestion) In the new test, the `--print-mounts` prefix assertion is near-tautological (the launcher only emits paths under `layers/main` and `$WORKTREE_PATH`); the static grep for the two entrypoint loop headers is the half that would have failed #602. Say so in the test header, loosen the grep patterns so a reformat doesn't cause a mystery failure, and assert the collected mount set is non-empty so a broken fixture can't pass with nothing to check — `plan.md:56`
- [ ] (suggestion) Resolve the "reuse or factor out a shared fixture helper" fork before implementing — duplicating the ~10-line fabricated tree keeps #602's passing test untouched and the diff smaller; refactoring it is scope creep unless the helper is trivial. (No runner registration needed: `run_script_tests.sh` auto-discovers `tests/test_*.sh`.) — `plan.md:56`
- [ ] (suggestion) Accuracy: the existing entrypoint `layers/main` loop has only `[ -d ]`, no `[ ! -L ]`, so the two entrypoint loops will not be "visibly the same mechanism". Make the asymmetry explicit in the new comment (why 4b/worktree needs `! -L` and section 4 does not) so a later reader does not "harmonize" the guard away — `plan.md:42`
- [ ] (suggestion) Step 5's verification is "confirm by reading", which is weak for a failure mode that only exists container-side. At minimum exercise the loop body's dir-vs-mkdir and symlink branches over a fabricated tree; with Docker up (above), a real container run of the entrypoint over that tree is the definitive check — `plan.md:84`
- [ ] (suggestion) If the smoke test is deferred, commit to filing the follow-up issue as a named deliverable — "documented follow-up" without an issue evaporates, which is how #602 became #604 — `plan.md:139`
- [ ] (suggestion) `.agent/knowledge/` has no Docker/devcontainer gotchas doc (verified — nearest are `vscode_setup.md`, `deployment_mode.md`), so the conditional "if one exists" resolves to "create one". State that plainly; it stays an operator-decided candidate either way — `plan.md:134`

### Summary
The plan is sound, correctly scoped, and covers every action the issue review
flagged; its factual references check out against source. Two things should be
settled before implementation: cross-reference the corrective rationale at the
misleading section-4b comment itself, and re-take the deferred container smoke
test decision now that Docker and the agent image are confirmed available here.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-08-24 00:02 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-604 at `96ad78b`
**Mode**: pre-push
**Depth**: Deep (reason: 660 lines / 11 files; privilege-drop + permissions surface; AGENTS.md governance trigger)
**Must-fix**: 6 | **Suggestions**: 12
**Round**: 1 | **Ship**: continue — a mutation-proven gap in the new regression test plus three defects inside the unreviewed stale-image guard warrant another read

### Findings
- [x] (must-fix) Regression test cannot detect #604 being reintroduced: deleting the `"${WORKTREE_ROOT:-}"` argument still yields 5 passed / 0 failed (mutation-verified by the lead) — the entrypoint call site is never exercised by either layer — `.devcontainer/agent/agent-entrypoint.sh:40`
- [x] (must-fix) Absent label reads as "unknown, stay quiet", but the no-label population is exactly the pre-#604 images with the broken entrypoint; verified the local 2-month-old image has an empty label, so it will never warn — `.agent/scripts/docker_run_agent.sh:322-328`
- [x] (must-fix) The two build paths hash different roots (`make agent-build` cwd-relative vs launcher `$ROOT_DIR` rewound out of worktrees); verified digests differ, so a worktree build causes a permanent false "stale" warning. Formula also duplicated with nothing asserting agreement — `Makefile:285` / `.agent/scripts/docker_run_agent.sh:317`
- [x] (must-fix) `startup_scripts_sha` contradicts its "warn (never block)" contract: no `|| true` on `current=` aborts the launcher under `set -euo pipefail` (reproduced); one missing script stamps a well-formed but wrong digest instead of the empty-means-quiet path — `.agent/scripts/docker_run_agent.sh:316-319,329`
- [x] (must-fix) `docker_run_agent.sh` script-table row not updated for the new staleness warning, though the consequences map requires it — `AGENTS.md:561`
- [x] (must-fix) ADR still carries the same overstated Check-4 coverage claim this PR narrows elsewhere; ADR-0012 permits a cross-reference addendum — `docs/decisions/0016-runtime-vs-baked-layer-chaining.md:80-82`
- [x] (suggestion) Hardcoded uid/gid 1000 while the image's `ros` user comes from `--build-arg USER_UID`; derive via `id -u ros` — `.agent/scripts/tests/test_entrypoint_chown_coverage.sh:177,179`
- [x] (suggestion) Layer B's real `docker run` breaks the suite's advertised hermeticity and fails spuriously on a remote/rootless daemon (host bind mount resolves on the daemon's filesystem) — `.agent/scripts/tests/run_script_tests.sh:9-10`, `.github/workflows/validate.yml:66-68`
- [x] (suggestion) Empty `mounted` set prints a reassuring PASS alongside the FAIL; exit early instead — `.agent/scripts/tests/test_entrypoint_chown_coverage.sh:85-87,119-126`
- [x] (suggestion) Contradictory comments: "chown/mkdir are stubbed" vs the next comment saying mkdir runs for real — `.agent/scripts/tests/test_entrypoint_chown_coverage.sh:90,96-98`
- [x] (suggestion) Neither layer exercises the baked image copies; add a label-vs-baked-scripts digest assertion — `.agent/scripts/tests/test_entrypoint_chown_coverage.sh:27-34,163-166`
- [x] (suggestion) `IMAGE` string duplicated from the launcher; a rename silently degrades layer B to permanent skip rather than failure — `.agent/scripts/tests/test_entrypoint_chown_coverage.sh:25`
- [x] (suggestion) Fixture models only a layer worktree; the workspace-worktree "clean no-op" claim is asserted by comment only — `.agent/scripts/tests/test_entrypoint_chown_coverage.sh:45-52`
- [x] (suggestion) Neither root argument validated: a stale/typo'd `WORKTREE_ROOT` makes loop 2 a silent no-op that reproduces #604 with no diagnostic — `.devcontainer/agent/fix-volume-ownership.sh:58,70`
- [x] (suggestion) Unguarded call under `set -e` dies with an opaque exit 127 on an image predating the script; a one-line existence check makes it actionable — `.devcontainer/agent/agent-entrypoint.sh:39-40`
- [x] (suggestion) Volume-ownership troubleshooting section is pre-#604 (no mention of the worktree's own `*_ws` or `fix-volume-ownership.sh`) — `.devcontainer/agent/README.md:276-285`
- [x] (suggestion) "after Dockerfile/startup-script changes" overstates the marker, which hashes only the two scripts — `.devcontainer/agent/README.md:13`
- [x] (suggestion) Plan step 3 still describes the superseded grep-for-loop-headers test design (contradicted later in the same section), and no `## Implementation` entry exists in the timeline — `.agent/work-plans/issue-604/plan.md:56`

### Notes
- Static analysis clean (shellcheck --severity=warning via pre-commit). Full `run_script_tests.sh` suite green (20 shell + 73 pytest), including the container smoke layer.
- Local Adversarial (qwen3.5:35b) returned 2 uncorroborated findings; both discarded on spot-check — bash resolves EXIT traps by name at execution time, and macOS/BSD `sha256sum` portability is not a target environment for a Linux/Docker toolchain.
- Process: `plan.md:171-176` flags the stale-image guard as scope beyond the reviewed plan and requests sign-off. That sign-off is still outstanding, and must-fix 2/3/4 all live inside that scope — dropping the guard to its own issue is a legitimate alternative to fixing it here.
