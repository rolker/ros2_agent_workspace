---
issue: 654
---

# Issue #654 — Local rosdep keys: workspace-owned ROSDEP_SOURCE_PATH aggregating per-repo rosdep.yaml, upstream PRs owed at merge

## Issue Review
**Status**: complete
**When**: 2026-09-22 10:32 -04:00
**By**: Claude Code Agent (Claude Sonnet 5)

**Issue**: #654
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Actions
- [ ] Add the new `.agent/scripts/rosdep_local_sources.sh` to the AGENTS.md Script Reference table (consequences-map row "A script in `.agent/scripts/`" — table entry not mentioned in issue scope).
- [ ] Clarify how the agent-container-image path (scope item 3) interacts with the existing bake-time `stage_rosdep_manifests.sh` / `docker_run_agent.sh --build` mechanism (#520) — does the new local `rosdep.yaml` set need its own build-context staging step, or is `ROSDEP_SOURCE_PATH` resolved only at container runtime against a mounted workspace?
- [ ] Note the implied `Makefile` touch-point for wiring the new staleness/upstream-PR-missing check into the `validate` target's recipe (consequences-map row "A script in `.agent/scripts/` → ... Makefile if it has a target" — not explicit in scope item 4).

### Findings (not blocking)

**Scope Assessment**
- Well-scoped, but large: 5 scope items spanning declaration format, an aggregation script, three separate CI integration points (`ci_local.sh` inner script, agent container image, hosted-workflow doc), an enforcement check, and a knowledge note. The pieces are genuinely interdependent (the enforcement check needs the aggregation mechanism; the CI paths need the same source list) so a single PR is defensible, but it's worth watching for scope creep during implementation — items 3's three integration points could be split into a fast-follow if the PR grows unwieldy.
- Right repo: yes. The aggregation script glob-scans `layers/main/*/src/*/rosdep.yaml` generically — no project-specific coupling — consistent with ADR-0003 (workspace infra is project-agnostic). The per-repo `rosdep.yaml` declaration itself lives in the *consuming* project repo (unh_marine_autonomy), correctly left out of this issue's scope.
- Dependencies: soft dependency on rolker/unh_marine_autonomy#397 (the consumer that needs `python3-pystac`/`snakemake`), but the test plan uses synthetic fixture repos, so this issue is independently testable and not blocked on #397 landing first. Parallel-tracked with rolker/agent_workspace#310 (writing up the general non-ROS-package dependency policy) — correctly deferred there per the issue's own text ("tried here before it is written down elsewhere").
- Verified against current code: `bootstrap.sh` and `ci_local.sh`'s inner script today only `rosdep init`/`rosdep update` against the system default `/etc/ros/rosdep/sources.list.d/20-default.list` — no existing `ROSDEP_SOURCE_PATH` override exists anywhere in `.agent/scripts/` or the `Makefile`, confirming this is genuinely new plumbing at the three touch points the issue names, not a duplicate of existing logic.

**Principle Alignment**

| Principle | Status | Notes |
|---|---|---|
| Enforcement over documentation | OK | Item 4 (make-validate staleness/missing-PR check) pairs enforcement with item 5's knowledge note, rather than documenting the policy alone. |
| Capture decisions, not just implementations | OK | ADR deliberately deferred until the mechanism survives a second instance — matches the workspace's recent ADR-fatigue lesson (no premature ADRs while a design is still evolving); the decision itself is recorded in the issue text and will land in the knowledge note. |
| Only what's needed | OK | Explicitly scoped to the Ubuntu-shipped-but-unkeyed case; the harder "Ubuntu doesn't ship it at all" case is named and deferred rather than solved speculatively. |
| A change includes its consequences | Action needed | See Actions above (AGENTS.md script table, Makefile touch-point). |
| Improve incrementally | Watch | Large but cohesive; see Scope Assessment. |
| Workspace vs. project separation | OK | Generic aggregation mechanism; no BizzyBoat/unh_marine_autonomy specifics baked into workspace scripts. |
| Workspace improvements cascade to projects | OK | Any project repo can opt in by adding its own `rosdep.yaml`; item 3's "documented one-step equivalent" for hosted CI keeps project repos independently buildable per ADR-0003. |
| Test what breaks | OK | Test plan targets real failure modes: idempotent regeneration, resolve success/failure gated on the env being sourced, in-container install, and staleness detection with a fixture that resolves upstream. |

**ADR Applicability**

| ADR | Triggered | Notes |
|---|---|---|
| 0003 — Project-agnostic workspace | Yes | Satisfied — see Scope Assessment. |
| 0004/0005 — Enforcement hierarchy / layered enforcement | Yes | The new check runs via `make validate`, which today is *not* invoked by hosted CI (`.github/workflows/validate.yml` runs `make lint` and `make test-scripts`, not `make validate`) — this mirrors `validate_workspace.py`'s existing local-only role, so it's consistent with current practice, not a new gap introduced by this issue. Worth keeping in mind during plan-task if the staleness check is meant to be load-bearing rather than advisory. |
| 0009 — Python package management | Yes (Tier 1) | `python3-pystac`/`snakemake` are Ubuntu 24.04 apt packages needed at runtime by a ROS-adjacent tool, matching Tier 1 ("ROS 2 dependency... available via apt/rosdep"). The issue's own "open case" callout (non-Ubuntu-shipped libraries → venv, out of scope) correctly avoids conflicting with ADR-0009's Tier 2/3, which govern the workspace `.venv` only. Note ADR-0009's own status is still "Proposed," not "Accepted" — not a blocker for this issue, just a standing gap unrelated to #654. |
| 0018 — Local-first CI verification | Yes | Touches `ci_local.sh`'s inner script (rosdep source install before `rosdep install`). Should verify during implementation that the new source-list step doesn't interact badly with the existing `upstream-repo:`/`rosdep-skip-keys:` attestation note lines for repos using `upstream.repos` — not called out in the issue, likely orthogonal, but worth a look. |

**Consequences**
- AGENTS.md Script Reference table needs a new row for `.agent/scripts/rosdep_local_sources.sh` (not currently in scope).
- Makefile needs a wiring point for the new validate-time check (implied by item 4, not explicit).
- `.agent/knowledge/dependency_policy.md` is a net-new knowledge doc — no existing doc to reconcile against.

**Recommendations**
- Add the AGENTS.md script-table entry and the Makefile wiring explicitly to the plan-task scope (see Actions).
- During plan-task, confirm the agent-container-image bake path (item 3) either reuses `stage_rosdep_manifests.sh`'s existing manifest-staging pattern or explains why runtime-only `ROSDEP_SOURCE_PATH` resolution is sufficient for the container case.

## Plan Authored
**Status**: complete
**When**: 2026-09-22 10:35 -04:00
**By**: Claude Code Agent (Claude Sonnet 5)

**Plan**: `.agent/work-plans/issue-654/plan.md` at `b3f39a6`
**Branch**: feature/issue-654 at `b3f39a6`
**Phases**: single

### Open questions
- [ ] Confirm no second default rosdep sources file exists beyond `20-default.list` before hard-coding the filename in `rosdep_local_sources.sh`.
- [ ] Confirm whether `run_script_tests.sh` auto-discovers new `test_*.sh` files or needs explicit registration.
- [ ] Staleness check's "upstream PR owed" comment heuristic is advisory/best-effort, not authoritative — call this out explicitly in the knowledge note.

## Plan Review
**Status**: complete
**When**: 2026-09-22 10:38 -04:00
**By**: Claude Code Agent (Claude Sonnet 5)

**Plan**: `.agent/work-plans/issue-654/plan.md` at `b3f39a6`
**PR**: PR-less
**Verdict**: changes-requested

### Findings
- [ ] (must-fix) Step 5's container-image sub-part claims the bake "picks up local keys automatically... via the same ROSDEP_SOURCE_PATH mechanism" — false as written. Neither the Dockerfile's bake-time `rosdep install` nor `agent-entrypoint.sh`'s launch-time `rosdep install` ever sets/stages `ROSDEP_SOURCE_PATH`, and the Dockerfile already computes `skip_keys` for any unresolvable key and continues (`Dockerfile:100-110`, graceful-degradation by design). Once unh_marine_autonomy#397 adds an `<exec_depend>` on `python3-pystac`/`snakemake`, that key lands on `skip_keys` and is silently omitted from the image with only a build-log warning — never a failure, never re-surfaced later. This is the "silent failure" pattern AGENTS.md's Quality Standard says not to wave off. Fix the claim and either (a) wire minimal `ROSDEP_SOURCE_PATH`-equivalent staging into `stage_rosdep_manifests.sh`/Dockerfile now (the container image is already a named in-scope CI path per issue scope item 3, so this isn't "unused plumbing"), or (b) keep it deferred but correct the prose and file a tracking issue now (not just a knowledge-note aside) so the gap is visible before #397 lands, since the current skip_keys design would otherwise mask it. — `plan.md` step 5, "Agent container image" bullet
- [ ] (must-fix) Step 6's staleness check resolves each key with `ROSDEP_SOURCE_PATH` unset to detect "now resolves upstream, delete the local entry" — but never runs `rosdep update` in that unset-env subshell and has no offline/stale-cache handling. This workspace already treats offline/stale rosdep state as expected and non-fatal elsewhere (`ci_local.sh`'s inner script: "Tolerate rosdep update failure (offline field host with baked deps, #520)"). Without a fresh update in the unset-env probe, a stale or unrefreshed default cache can produce either a false "resolves upstream → delete it" (if the cache happens to be contaminated by a prior `ROSDEP_SOURCE_PATH`-inclusive run or stale in the wrong direction) or mask a genuine upstream merge. Add: run `rosdep update` inside the unset-env subshell before resolving, and if that update fails, report the check result for that key as SKIPPED — never silently answer "resolves" or "doesn't resolve" from a cache state the check can't vouch for. — `plan.md` step 6
- [ ] (suggestion) Step 5's hosted-CI sub-part commits only to "a short recipe" in the knowledge note, not to a concrete artifact. For it to be adoptable by unh_marine_autonomy's workflow "in one step" (the issue's own bar), the knowledge note should include an actual copy-pasteable GitHub Actions step (set `ROSDEP_SOURCE_PATH`, write `30-workspace-local.list` from the repo's own `rosdep.yaml`, `rosdep update` before `rosdep install`), not prose describing the shape. — `plan.md` step 5, "Hosted CI" bullet
- [ ] (suggestion) Step 4's Makefile stamp invokes `.SECONDEXPANSION` "same pattern as the existing `layer-%.done` rule," but the new `$(STAMP)/rosdep-local.done` target has no `%` — it's a static target, so a plain `$(wildcard ...)` prerequisite list works without secondary expansion. Not a correctness problem (mtime-based staleness is otherwise sound for both "content changed" and "file newly appears," verified against the existing `layer-%.done` precedent at `Makefile:209`), just avoidable complexity — worth simplifying during implementation. — `plan.md` step 4

### Verified during review
- Worktree-aware `ROSDEP_SOURCE_PATH` export (step 2): the plan's "resolve the same way `_VENV_ROOT` does" claim checks out against `setup.bash:130-144` — a concrete, correct pattern (main-root hop for both workspace and layer worktrees, unset after use).
- Makefile stamp content-change detection (step 4): confirmed `$(STAMP)/layer-%.done`'s existing `$$(wildcard ...)` prerequisite pattern re-runs on file edits (mtime bump) and on newly-appearing files (Makefiles are re-parsed and `$(wildcard)` re-evaluated every invocation) — the analogous claim for `rosdep-local.done` is correct in substance, independent of the `.SECONDEXPANSION` question above.
- `ci_local.sh`'s `repo_file_exists`/`repo_file_content` helpers (step 5) exist and are already used for `upstream.repos` and `.agents/ci_local_rosdep_skip_keys.txt` — the plan's proposed reuse for `rosdep.yaml` is consistent with the existing pattern.
