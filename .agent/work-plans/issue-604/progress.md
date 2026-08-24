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

## Implementation
**Status**: complete
**When**: 2026-08-24 00:22 -04:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-604 at `7971b64`
**Addressed**: `## Local Review (Pre-Push)` (2026-08-24 00:02 -04:00, branch at `96ad78b`) — 6 must-fix, 12 suggestions
**Commits**: 10ff8f8 c75e435 f3efa68 0b80779 9d8fb9b 16b4ac6 445a5ef ac11621 37b4747 f2a55ca bfd7a03 15d28f7

Two operator decisions taken at the review checkpoint framed this pass: the
stale-image guard **stays in this PR** with all three of its defects repaired
here rather than split out, and the test-coverage must-fix is answered by
**covering the call site**, not by a static text assertion.

### Actions
- [x] (must-fix) Regression test could not detect #604 being reintroduced — `.agent/scripts/tests/test_entrypoint_chown_coverage.sh` layer B now runs the working tree's `agent-entrypoint.sh` as the image ENTRYPOINT over real anonymous volumes (both startup scripts mounted over their baked copies, both launcher env vars set) and probes each mountpoint as the user the entrypoint hands off to. Mutation-verified both ways: deleting `"${WORKTREE_ROOT:-}"` at `agent-entrypoint.sh:40` fails the suite, and so does deleting loop 2 of `fix-volume-ownership.sh` (which fails both layers). A new assertion also fails if the probe runs as uid 0 — a writable check under root proves nothing.
- [x] (must-fix) Absent label now warns with its own message — an unmarked image predates the marker and is exactly the pre-#604 population — `.agent/scripts/docker_run_agent.sh`
- [x] (must-fix) The two build paths are now one: `make agent-build` delegates to `docker_run_agent.sh --build-only`, so the digest has a single implementation over a single directory. `tests/test_agent_image_build_paths.sh` asserts the delegation, the absence of a second `docker build`/digest formula, one `sha256sum` site in the launcher, and that the Dockerfile ARG/LABEL name matches what the launcher reads back — `Makefile:276`, `.agent/scripts/docker_run_agent.sh`
- [x] (must-fix) `startup_scripts_sha` can no longer abort the launcher: it returns empty (never a partial digest) when a script is missing, every failable substitution carries `|| true`, and the warn function ends `return 0`. Reproduced the abort before the fix and the clean warn after — `.agent/scripts/docker_run_agent.sh`
- [x] (must-fix) `docker_run_agent.sh` script-table row updated for the single build path and the staleness warning — `AGENTS.md:561`
- [x] (must-fix) ADR-0016 Check-4 coverage claim scoped via a References cross-reference addendum (ADR-0012), leaving the accepted Consequences text intact — `docs/decisions/0016-runtime-vs-baked-layer-chaining.md`
- [x] (suggestion) Hardcoded uid/gid 1000 gone — the entrypoint resolves the target user itself now that layer B goes through it
- [x] (suggestion) Non-local Docker daemon (remote/rootless) now SKIPs instead of failing spuriously; the hermeticity claims in `run_script_tests.sh` and `.github/workflows/validate.yml` say so explicitly
- [x] (suggestion) Empty `mounted` set now stops the run via `summarize_and_exit` instead of printing a reassuring PASS beside the FAIL
- [x] (suggestion) Contradictory stub comment corrected — only `chown` is stubbed; `mkdir` runs for real, deliberately
- [x] (suggestion) Baked image copies now covered: the image's label is compared against a digest of the scripts actually baked into it (skips when unlabelled). Verified on a throwaway derived image, which was removed afterwards rather than overwriting the shared `:latest` tag
- [x] (suggestion) `IMAGE` is read from the launcher, so a rename fails rather than degrading layer B to a permanent silent SKIP
- [x] (suggestion) Workspace-worktree case is now asserted (loop 2 touches nothing inside it), not claimed in a comment
- [x] (suggestion) Both roots validated in `fix-volume-ownership.sh` — a stale/typo'd root fails loud instead of no-oping into #604's failure mode, with test cases for each
- [x] (suggestion) Unguarded entrypoint call replaced with an existence check naming the cause and the fix, instead of exit 127 — `.devcontainer/agent/agent-entrypoint.sh`
- [x] (suggestion) Volume-ownership troubleshooting section rewritten around the two mount sets, `fix-volume-ownership.sh`, and why the host dir underneath an anonymous volume is irrelevant to its ownership — `.devcontainer/agent/README.md`
- [x] (suggestion) Marker scope corrected: only the two startup scripts are hashed; Dockerfile/rosdep changes are not detected — `.devcontainer/agent/README.md`
- [x] (suggestion) Plan step 3 re-synced (grep-for-loop-headers design and "smoke test" framing both superseded), new files and the Makefile delegation recorded, and the guard's sign-off question closed with the checkpoint decision — `.agent/work-plans/issue-604/plan.md`

### Notes
- Found while fixing must-fix 3: moving the build section above the worktree lookup also moved it above the credential check, so a `--build` launch with no auth would have paid the ten-minute rosdep bake before failing. The build now sits between validation and the worktree lookup; `--build-only` is exempt from the auth check since it never launches a container.
- Behaviour note for review: `make agent-build` now resolves the **main workspace root** even when run from a worktree (the launcher's existing rewind), so a worktree's edits to the startup scripts are not what gets baked. That was already true of every container the launcher ran; it is now also true of the build, and is documented in the devcontainer README.
- Full `run_script_tests.sh` green: 21 shell + 73 pytest, container layer included. Pre-commit (incl. shellcheck) ran on every commit.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-08-24 00:43 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-604 at `26270ac`
**Mode**: pre-push
**Depth**: Deep (reason: 1132 added lines / 15 files; privilege-drop + file-ownership surface; AGENTS.md + ADR governance triggers)
**Must-fix**: 3 | **Suggestions**: 12
**Round**: 2 | **Ship**: recommended — must-fix fell 6 -> 3 and all three are precise mechanical edits (a contradicted comment, a recovery command that cannot run, a three-line hermetic assertion); no design question remains open

### Findings
- [x] (must-fix) Marker's defining comment states the launcher treats an absent label as "unknown" and does not warn — it warns, deliberately (round-1 must-fix 2); a maintainer trusting the comment would re-hide the pre-#604 population — `.devcontainer/agent/Dockerfile:129-131`
- [x] (must-fix) Documented recovery cannot run: the `--shell` session it follows is the `ros` user (entrypoint setpriv), the image has no sudo, and no-new-privileges blocks escalation, so the chown silently no-ops through `2>/dev/null || true`; needs `docker exec -u 0` — `.devcontainer/agent/README.md:303-312`
- [x] (must-fix) The #604 defect (dropped `WORKTREE_ROOT` argument) is caught only by layer B, which always SKIPs in CI, and `run_script_tests.sh` suppresses passing output so the SKIP is invisible — a re-deletion merges green; add a hermetic both-roots assertion ALONGSIDE layer B, not replacing it (the call-site-coverage decision stands) — `.agent/scripts/tests/test_entrypoint_chown_coverage.sh:22-26,226-232`
- [x] (suggestion) Reorder lost fail-fast: `--issue <bad> --build` now reaches the build before the worktree lookup (verified against main, which errors in under a second); move the lookup above the build when not `--build-only` — `.agent/scripts/docker_run_agent.sh:272-393`
- [x] (suggestion) `--build-only --print-mounts` prints nothing and exits 0; the new test's last case relies on that, and would overwrite the real `:latest` tag if it ever dropped `--print-mounts` — `.agent/scripts/docker_run_agent.sh:352,391`
- [x] (suggestion) `baked_sha=$(docker run ... 2>&1)` folds daemon chatter into the compared digest, and the label name is hardcoded while `IMAGE` is read from the launcher — `.agent/scripts/tests/test_entrypoint_chown_coverage.sh:291,295-296`
- [x] (suggestion) OAuth token is read and exported before the `--build-only` short-circuit, so a credential-free build still pulls the subscription token into the `docker build` child's environment — `.agent/scripts/docker_run_agent.sh:220-231`
- [x] (suggestion) `chown -R ... 2>/dev/null || true` swallows every failure; under rootless/userns-remap Docker that leaves #604's exact signature with no diagnostic — `.devcontainer/agent/fix-volume-ownership.sh:63`
- [x] (suggestion) Baking from the main root is the right call, but it is silent: a worktree edit to the startup scripts is neither baked nor warned about, in the workflow this PR exists to protect; README documents it, the `--build-only` help / Makefile comment / AGENTS.md row do not, and there is no runtime signal — `.agent/scripts/docker_run_agent.sh:33-39`
- [x] (suggestion) `STAGE_DIR` is one fixed path under an `rm -rf` EXIT trap shared by every entry point — a dispatch auto-build racing `make agent-build` deletes the other's build context; a stale `.rosdep-manifests/` is present in this worktree now — `.agent/scripts/docker_run_agent.sh:368`
- [x] (suggestion) Three places still say the stager is "called by both build entry points"; and README's bare `docker build` now yields a permanently unmarked image — `.agent/scripts/stage_rosdep_manifests.sh:16-17`, `AGENTS.md:562`, `.agent/scripts/docker_run_agent.sh:367`, `.devcontainer/agent/README.md:285`
- [x] (suggestion) `grep -c 'sha256sum'` counts comment lines; the sibling Makefile check strips comments first — `.agent/scripts/tests/test_agent_image_build_paths.sh:60`
- [x] (suggestion) Layer B's `docker run` omits `--security-opt no-new-privileges:true` and `-w`, both of which the launcher passes — a setpriv-to-setuid swap would pass this test and break every real launch — `.agent/scripts/tests/test_entrypoint_chown_coverage.sh:254-262`
- [x] (suggestion) Layer B chowns the `/tmp` fixture to the image's `ros` uid; cleanup cannot remove it when that differs from the runner's — `.agent/scripts/tests/test_entrypoint_chown_coverage.sh:99,233-270`
- [x] (suggestion) Plan's Files to Change omits `run_script_tests.sh` and `.github/workflows/validate.yml`, both changed — `.agent/work-plans/issue-604/plan.md`

### Notes
- Round-1 fixes verified, not trusted. Mutation-proved both directions here: deleting `"${WORKTREE_ROOT:-}"` at `agent-entrypoint.sh:48` fails the suite; deleting loop 2 of `fix-volume-ownership.sh` fails 3 assertions across both layers. `warn_if_startup_scripts_stale` exercised in isolation for both-missing / one-missing / unlabelled-image — all warn and return 0. Full `run_script_tests.sh` green (21 shell + 73 pytest, container layer included). Pre-commit clean over the whole range.
- On the handoff's first question (the reorder): it broke fail-fast, verified empirically against `main`. Nothing else — the STAGE_DIR EXIT trap is the script's only one, so no trap collision, and the `--build-only` credential exemption is sound in itself (no container is launched); the only residue is the token still being read (suggestion 4).
- On the handoff's second question (`make agent-build` resolving the main root): the behaviour is correct — it matches the tree the launcher mounts and the digest it compares, and baking a worktree's copy is what produced the permanent false "stale". The documentation is adequate in the README and nowhere else, and there is no runtime signal; see suggestion 6.
- Local Adversarial (qwen3.5:35b) needed `LOCAL_REVIEW_NUM_CTX=44000` — the 32k default rejected this diff. Its single finding (TOCTOU between the `[ -f ]` loop and the `cat` in `startup_scripts_sha`) was spot-checked and dropped: `pipefail` already fails the build path loud, and the residual window needs a mid-invocation deletion by the invoking user.


## Implementation
**Status**: complete
**When**: 2026-08-24 09:11 -04:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-604 at `25cd47b`
**Addressed**: `## Local Review (Pre-Push)` (Round 2) — 2026-08-24 00:43 -04:00, branch at `26270ac`
**Commits**: f0d078f, f427244, 67c5424, d2d5024, e9bcd54, f5fa0e2, b86ba4e, 18ba230, 0fe5be0, 8d59529, e4878a1, 8fbb31a, 25cd47b

All 15 open findings (3 must-fix, 12 suggestions) actioned; none deferred.

### Actions
- [x] (must-fix) Marker comment inverted the implemented contract — rewritten to state that an absent/empty label WARNS, and why — `.devcontainer/agent/Dockerfile:125-131` (f0d078f)
- [x] (must-fix) Recovery block could not run from the dropped-privilege `--shell` session — split into a host-side `docker exec -u 0` invocation with the reason stated — `.devcontainer/agent/README.md:303-325` (f427244)
- [x] (must-fix) Added an always-runs hermetic assertion (section A4) that the entrypoint's call site passes BOTH roots, ALONGSIDE layer B — `.agent/scripts/tests/test_entrypoint_chown_coverage.sh` (67c5424). Mutation-proved: deleting `"${WORKTREE_ROOT:-}"` fails it; intact it passes. Layer B retained unchanged.
- [x] (suggestion) Worktree resolution moved ahead of the credential read and the image build, guarded by `BUILD_ONLY=false` — a bad `--issue` now errors in ~4 ms again — `.agent/scripts/docker_run_agent.sh` (d2d5024)
- [x] (suggestion) `--print-mounts` with `--build`/`--build-only` now prints a `[dry run] … would build …` line instead of a wordless rc-0 no-op; documented in `--build-only` help; the build-path test asserts the line, so dropping the flag fails loudly — (e9bcd54)
- [x] (suggestion) `baked_sha` uses `2>/dev/null` (no daemon chatter folded into the digest) and the label name is read from the launcher's `STARTUP_SCRIPTS_LABEL` — mutation-proved: renaming it in the launcher fails the test instead of degrading to a silent SKIP — (f5fa0e2)
- [x] (suggestion) OAuth token read/export now skipped for `--build-only`, so a credential-free build stays credential-free — `.agent/scripts/docker_run_agent.sh:261-285` (b86ba4e)
- [x] (suggestion) Failed recursive chown now names the path and the usual cause (rootless / userns-remapped daemon) instead of vanishing into `2>/dev/null || true`; still non-fatal (verified: 6 warnings, rc=0) — `.devcontainer/agent/fix-volume-ownership.sh:57-80` (18ba230)
- [x] (suggestion) `make agent-build` resolving the main root is now documented in the `--build-only` help, the Makefile recipe comment and the AGENTS.md row, and carries a runtime NOTE on every build path (dry run included) when launched from a worktree — verified it fires from a worktree and not outside one — (e4878a1)
- [x] (suggestion) Concurrent builds serialized on a `flock` in TMPDIR keyed by uid + context path, so a dispatch auto-build and `make agent-build` cannot delete each other's staging dir. `STAGE_DIR` itself stays fixed — the Dockerfile's `COPY` is context-relative and cannot be randomized; the reason is recorded inline. No stale `.rosdep-manifests/` in either tree — (0fe5be0)
- [x] (suggestion) Stale "called by both build entry points" wording corrected in the stager header, the launcher comment and the AGENTS.md row; README's bare `docker build` alternative retired outright (it stamps an empty marker that warns at every launch) — (8fbb31a)
- [x] (suggestion) Digest-site count now strips comments first, matching the sibling Makefile check — mutation-proved both ways: a second real `sha256sum` call fails it, a comment mentioning `sha256sum` does not — (8d59529)
- [x] (suggestion) Layer B's `docker run` now passes `--security-opt no-new-privileges:true` and `-w`, matching the real launch, so a setpriv→setuid swap can no longer pass — (f5fa0e2)
- [x] (suggestion) Layer B's fixture cleanup hands the container-chowned tree back via a root container before `rm -rf`, and reports rather than fails if it still cannot — verified no `/tmp/chown_e2e.*` residue — (f5fa0e2)
- [x] (suggestion) Plan's Files to Change now lists `run_script_tests.sh`, `.github/workflows/validate.yml` and `stage_rosdep_manifests.sh` — `.agent/work-plans/issue-604/plan.md` (25cd47b)

### Verification
- Full `run_script_tests.sh` green: 18 shell scripts + 73 pytest, container layer included (local Docker present, so layer B ran rather than skipped).
- `pre-commit run --from-ref main --to-ref HEAD`: clean over the whole range.
- Every regression-test change mutation-checked in both directions (details per action above).
- No image was built: the shared `ros2-agent-workspace-agent:latest` tag is untouched. The local image predates the marker, so the baked-digest check still SKIPs — that path remains unexercised locally and is exercised on the first real `make agent-build`.

## Integrated Review
**Status**: complete
**When**: 2026-08-24 09:31 -04:00
**By**: Claude Code Agent (Claude Opus)

**PR**: #606 at `7831658`
**Sources**: 3 (Copilot R1 @ `7831658`, `## Local Review (Pre-Push)` R1 @ `96ad78b` + R2 @ `26270ac`, CI rollup @ `7831658`)
**Cross-source confirmations**: 0 strict (no two sources at the same head SHA); **2 incomplete-fix lineages** — see below
**CI**: all-pass (Lint (pre-commit), Script tests, Validate Documentation, Validate commit identity (Mechanism C), Copilot reviewer — 9 check-runs, 0 failures)

Copilot's review is against the current head, so nothing here is stale. It
raised 3 inline comments, no conversation comments, no human reviewer. All 3
are new — none repeats a finding already open in the timeline, and none
re-opens either settled operator decision (container smoke test folded into
this PR; stale-image guard stays here).

The notable signal is not a same-SHA cross-confirmation but a **pattern**: two
of Copilot's three findings land on code the round-2 fixes touched, and each is
the half that fix did not reach. Round 2's suggestion 5 warned that
`chown -R … 2>/dev/null || true` swallowed failures; the fix added the warning
at that one call site (18ba230) and left the sibling `chown` in the same
function's other branch unguarded — which Copilot now flags. Round 2's must-fix
2 said the documented recovery block could not run from the `--shell` session;
the fix rewrote it as a host-side `docker exec -u 0` (f427244) and left the
arguments unrunnable on the host — which Copilot now flags. Both fixes were
verified for the mechanism they changed and not for the block they left behind.
That is the class of residue to watch when addressing these.

### Findings
- [ ] (must-fix, Copilot; lineage: R2 suggestion 5, partially fixed at 18ba230) The mkdir branch's `chown` is unguarded under `set -e`, so it aborts where the sibling recursive `chown` warns — contradicting the function's own "non-fatal by design … under a rootless or userns-remapped daemon every chown fails, that is the case this warning exists for" comment eight lines above. The entrypoint calls this script unguarded under `set -euo pipefail`, so the abort is a container that refuses to start with a bare `chown: Operation not permitted` and no context; in the documented `docker exec -u 0` recovery path it strands every workspace after the failing one — the exact harm the recursive branch's comment says it is avoiding. Reachability is narrow and should be stated honestly: the launcher mkdir -p's and mounts all three subdirs (sections 4/4b), so under a normal launch `[ -d "$target" ]` is always true and this branch is not taken. It is taken in the standalone recovery invocation, in layer A of the coverage test, and for any `*_ws` whose subdir appeared after the launcher's scan. Not impossible, so not dismissible. Fix: mirror lines 71-75 — `if ! mkdir -p "$target" || ! chown …; then` warn with the path and the rootless/userns cause, and continue. Guard the `mkdir` too, which Copilot did not mention but has the same abort behaviour on a read-only mount — `.devcontainer/agent/fix-volume-ownership.sh:77-78`
- [ ] (suggestion, Copilot; lineage: R2 must-fix 2, partially fixed at f427244) The recovery block's `"$ROS2_AGENT_WORKSPACE_ROOT"` / `"$WORKTREE_ROOT"` expand in the **host** shell, where the launcher never sets them — the launcher passes them into the container (`docker_run_agent.sh:696-697`), not out to the caller. Copy-paste therefore passes two empty strings. Rated suggestion, not must-fix, on a deliberate distinction from its round-2 predecessor: that one failed **silently** (the chown no-oped through `2>/dev/null || true`), whereas this one fails **loud** — the root validation added this round catches it with `ERROR: workspace root is not a directory:` — and the next sentence already tells the operator to substitute the real paths. Still worth fixing in this round, since the roots are readable from the container the command already targets: `"$(docker exec <container> printenv ROS2_AGENT_WORKSPACE_ROOT)"` and the same for `WORKTREE_ROOT`. Adjust the following sentence, which would then be describing a substitution the command no longer needs — `.devcontainer/agent/README.md:330-338`
- [ ] (suggestion, Copilot) Inverted failure message: the branch fires when `--print-mounts` exits **non-zero**, but prints `FAIL: --print-mounts exits 0`. Confirmed against the file's own conventions — every other failure-only message here is a problem statement (`could not read IMAGE_NAME…`, `fixture produced no *_ws anonymous volumes…`), and the one assertion-phrased message (line 166) has a paired `pass` at 168 that makes it read correctly. Line 129 has no `pass` twin, so the assertion phrasing has nothing to disambiguate it. Fix: `fail "--print-mounts exited non-zero (rc=$?, out=$mount_out)"`, capturing `rc` before the `[` overwrites `$?` — `.agent/scripts/tests/test_entrypoint_chown_coverage.sh:129`

### False positives
- None. All three Copilot comments were verified against the local files and hold.

### Carried forward (not a finding)
- The baked-digest assertion (`test_entrypoint_chown_coverage.sh:341-350`) still SKIPs locally because the `:latest` image predates the staleness marker, so that path stays unexercised until the next real `make agent-build`. Recorded in the PR body rather than hidden; unchanged by this round and not actionable here.

### Next step
Three open findings, one must-fix → `address-findings`. All three are localized
mechanical edits (one shell branch, one docs command block, one message string)
with no design question outstanding; a re-review should be cheap.
