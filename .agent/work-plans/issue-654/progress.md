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

## Implementation
**Status**: complete
**When**: 2026-09-22 11:04 -04:00
**By**: Claude Opus

**Plan**: `.agent/work-plans/issue-654/plan.md` at `836fd3e` (amended)
**Branch**: feature/issue-654 at `836fd3e`
**PR**: PR-less (not pushed — dispatch contract)

### Commits

| SHA | Subject |
|---|---|
| `0b99a93` | plan: apply plan-review amendments |
| `180f4f8` | Add rosdep_local_sources.sh: workspace-owned rosdep source list |
| `6d336bc` | Export ROSDEP_SOURCE_PATH from setup.bash; generate it in bootstrap.sh |
| `708a0c3` | Makefile: regenerate the local rosdep source list in the build chain |
| `9524285` | ci_local: overlay the tested repo's own rosdep.yaml via ROSDEP_SOURCE_PATH |
| `23b4a4f` | Agent image: bake workspace-local rosdep keys, WARN on every skipped key |
| `5c94d91` | Add rosdep_local_staleness_check.sh, wired into make validate |
| `34bc560` | Add test_rosdep_local_sources.sh: 39 checks over the local-key mechanism |
| `2c7df49` | Document the dependency policy and the two new scripts |
| `c0ddd25` | docker_run_agent: note the rosdep.yaml staging in the build block |
| `836fd3e` | plan: sync Files to Change with the implemented diff |

### Plan-review findings: all four applied before implementation

Per the operator's "amend plan, then implement" choice, `0b99a93` applied all
four findings inline in plan.md; the implementation then followed the amended
plan.

1. **(must-fix) Container image** — the false "picks up local keys
   automatically" claim is gone and the path is wired: the staging script
   stages each repo's `rosdep.yaml` into `<stage_dir>/rosdep-local/`, the
   Dockerfile builds `/opt/rosdep-sources` from every system `*.list` plus a
   generated local list and sets `ENV ROSDEP_SOURCE_PATH`, and unresolvable
   keys now end the build step in a labelled `WARNING` block naming each one
   and how to fix it. `agent-entrypoint.sh` refreshes the cache once when the
   bind-mounted workspace carries a non-empty local list (rosdep's cache is
   keyed by source URL, so the baked cache has no entry for the workspace's
   `file://` URLs; the workspace is mounted at its host absolute path, so
   those URLs resolve).
2. **(must-fix) Staleness probe** — `ROSDEP_SOURCE_PATH` unset, `HOME` on a
   throwaway dir, its own `rosdep update` run first; a failed update yields
   SKIPPED for every key and **no** resolution verdict. Exit codes 0/1/2/3 are
   distinct, and `make validate` prints a 3 and clears it.
3. **(suggestion) Hosted CI** — the knowledge note carries a concrete,
   copy-pasteable GitHub Actions step that exports through `$GITHUB_ENV`, so a
   project repo's existing `rosdep install` step needs no change.
4. **(suggestion) `.SECONDEXPANSION`** — dropped; the new stamp uses a plain
   `$(wildcard …)` prerequisite list and is placed **before** the
   `.SECONDEXPANSION` directive so that stays scoped to the one rule needing it.

### Departures from the plan (synced inline)

- **Aggregation copies every system `*.list`, not just `20-default.list`**, and
  the uninitialized-rosdep refusal is **exit 3**, not 2. This host carries a
  `10-local.list` alongside the default; naming one file would silently drop a
  source the caller already had. Open Question 1 resolved in plan.md.
- **`bootstrap.sh` anchors via `workspace_root.sh`** rather than deriving the
  root from `BASH_SOURCE`: bootstrapping from a worktree would otherwise create
  a second, divergent generated directory there.
- **`ci_local.sh` gained a `+rosdep-local` steps token** (a steps token only, no
  new note field). The plan said no attestation change was needed; an
  environment that resolved a key from a repo-carried source is not the same
  verified environment as one that did not, so the note should say so.
- **`test_make_validate.sh` and `test_ci_local.sh` were extended**, which the
  plan's table did not list. The former had to be: it runs the real recipe
  against stubs, so a third unstubbed invocation would have failed it.
- Also touched: `.agent/knowledge/README.md` (index), `docker_run_agent.sh`
  (comment), and the `validate_workspace.py` AGENTS.md row, which said `make
  validate` runs two checks.

### Tests run

| What | Result |
|---|---|
| `make test-scripts` (worktree, no ROS env) | ✅ 29 shell tests + 220 pytest passed |
| `.agent/scripts/tests/test_rosdep_local_sources.sh` (new) | ✅ 39 passed, 0 failed |
| `.agent/scripts/tests/test_ci_local.sh` | ✅ 88 passed (was 83; +5 for the `+rosdep-local` path) |
| `.agent/scripts/tests/test_make_validate.sh` | ✅ 7 passed (was 4; +3 for the exit-3-is-a-notice split) |
| `.agent/scripts/tests/test_agent_image_build_paths.sh` | ✅ 7 passed — the single-build-path and one-sha256sum-site invariants still hold |
| `make validate` (worktree) | `validate_workspace.py` exits 3 (no `configs/manifest` in a workspace worktree — the pre-existing, expected state, pinned by `test_make_validate.sh`); layer-sourcing guard PASSED; `rosdep-local: no rosdep.yaml files found — nothing to check` |
| `bootstrap.sh --dry-run` | ✅ byte-identical to pre-change output on an already-bootstrapped host |
| Manual: aggregation, staging, staleness check against temp fixtures | ✅ (all folded into the new test file) |
| pre-commit | ✅ on every commit; never `--no-verify` |

### Not done / owed

- [ ] **The agent image must be rebuilt after merge** — `make agent-build`. Per
      #604 the launcher only builds when the image is *missing*, and the startup
      scripts bake from the MAIN checkout, so this worktree's
      `agent-entrypoint.sh` change cannot be baked until it lands. Until then
      the startup-scripts staleness marker warns at every launch, which is the
      intended signal.
- [ ] **`rosdep install` was never run** — it needs sudo; the operator runs it.
      The generated source list and `rosdep update`/`rosdep resolve` against it
      were verified directly; the install itself was not.
- [ ] **No `rosdep.yaml` was added to any project repo** — deliberately out of
      scope here; that is `rolker/unh_marine_autonomy#397`'s job. Every path in
      this PR is exercised by synthetic fixtures, so nothing waits on #397.
- [ ] **The Dockerfile bake was not executed** — no image build was run (no
      Docker build in this dispatch). The staging half is covered by tests; the
      `RUN` block was reviewed for POSIX-`sh` compatibility (`$(( ))`,
      `${var:+…}`, no bashisms) but not built. First `make agent-build` after
      merge is the real check.
- [ ] Open Question 3 stands by design: the upstream-PR-marker test is an
      advisory text heuristic, called out as such in the knowledge note, the
      script header, and the AGENTS.md row.

### Notes for review

- `ROSDEP_SOURCE_PATH` **replaces** `sources.list.d`, it does not add to it.
  Every generator here copies the system lists in; `setup.bash` exports the
  variable **only** when the directory exists, because pointing it at a path
  that was never generated leaves rosdep with no sources at all rather than
  falling back.
- rosdep's cache is per-user and keyed by source URL. That is why the probe
  uses a throwaway `HOME`, why `ci_local.sh` bypasses the baked-cache guard on
  the local-key path, and why the entrypoint refreshes once.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-09-22 11:13 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-654 at `a5158d1`
**Mode**: pre-push
**Depth**: Deep (reason: 1469 lines / 19 files, plus governance-file and CI/container-path triggers)
**Must-fix**: 4 | **Suggestions**: 5
**Round**: 1 | **Ship**: continue — one of the must-fixes is a genuine correctness gap in the container (the launch-time cache refresh lands in the wrong user's cache) and one is a missing-validation/trust-boundary concern, not mechanical fixes

**Specialists**: Static Analysis, Governance, Plan Drift, Claude Adversarial Lens A + Lens B (Copilot and Local Adversarial off by default). Static analysis clean at shellcheck `--severity=warning` on all 11 changed shell files; `test_rosdep_local_sources.sh` 39/39, `test_make_validate.sh` 7/7, `test_ci_local.sh` 88/88 all pass.

### Findings
- [x] (must-fix) `make build` now hard-fails when the system rosdep sources are uninitialized: the stamp recipe calls `rosdep_local_sources.sh` (exit 3) with no `||` guard, unlike bootstrap.sh's recoverable call of the same script; no test covers the path — `Makefile:223-227`
- [x] (must-fix) The launch-time `rosdep update` runs as root (setpriv drop is later, at :187), so the bind-mounted workspace's `file://` sources land in `/root`'s cache; the agent runs as `ros`, inherits the same `ROSDEP_SOURCE_PATH`, and its baked cache is keyed to `/opt/rosdep-local/...` URLs — in-session `rosdep resolve`/`install` of a workspace-local key still fails. The new Dockerfile comment asserts the opposite — `.devcontainer/agent/agent-entrypoint.sh:103-108`, `.devcontainer/agent/Dockerfile:146-149`
- [x] (must-fix) Nothing validates a project repo's `rosdep.yaml` shape before it drives `rosdep install -y` as root (image bake, ci_local container, container launch). rosdep's format also accepts `pip`, `npm`, `gem` and `source` (download-and-run an rdmanifest) rules, so a repo-merged file can install outside the documented `{distro: [pkg]}` form — and a `pip:` rule routes around ADR-0009. Enforce the list form (the staleness check already `yaml.safe_load`s) and state the constraint as a rule — `.agent/scripts/rosdep_local_sources.sh:61-83`, `.agent/scripts/stage_rosdep_manifests.sh:99-113`, `.agent/knowledge/dependency_policy.md:22-30`
- [x] (must-fix) Plan still contradicts the code: "no new note fields needed" vs. the `+rosdep-local` steps token the implementation deliberately adds — `.agent/work-plans/issue-654/plan.md:284`
- [x] (suggestion) Staged local yamls are keyed by repo basename, so two same-named repos under different `*_ws/src/` trees silently overwrite each other while `local_count` counts both — `.agent/scripts/stage_rosdep_manifests.sh:105-110`
- [x] (suggestion) ADR-0018 gets no "format extension" paragraph for the `+rosdep-local` steps token, unlike #577's `upstream-repo:`/`rosdep-skip-keys:` precedent — `docs/decisions/0018-local-first-ci-verification.md`
- [x] (suggestion) The hosted-CI recipe lives only in prose, so it does not cascade: neither the project-repo CI template nor the onboarding step that handles non-rosdep deps mentions it — `.agent/templates/ci_workflow.yml:35-42`, `.claude/skills/onboard-project/SKILL.md:180`
- [x] (suggestion) `dependency_policy.md` never states the trust implication — a `rosdep.yaml` merged in any project repo now has root-level install influence on the dev host, CI containers and the agent image — `.agent/knowledge/dependency_policy.md`
- [x] (suggestion) `make validate`'s one-line description was updated in `make help` but not here — `README.md:220`

## Implementation
**Status**: complete
**When**: 2026-09-22 11:29 -04:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-654 at `94871af`
**Addressed**: ## Local Review (Pre-Push), 2026-09-22 11:13 -04:00, branch at `a5158d1` (round 1, changes-requested — 4 must-fix, 5 suggestions)
**Commits**: 2949ad0 94871af 63dcacb 74a9314 53e93cd 6cabe0d 8e2afab 7b6c7e5 0f0b6c2 e1ac541 

### Actions

- [x] (must-fix) `make build` hard-failed on an uninitialized rosdep — `Makefile:223-227`
      The stamp recipe now branches on the generator's status: exit 3 (no `*.list`
      under the system sources dir — a clone that has not run `sudo rosdep init`)
      is a note, and the stamp is deliberately **not** touched so the next build
      retries once rosdep is initialized; any other non-zero status fails the
      build. Tests drive the real recipe in a sandbox against a stub generator
      (exit 0 / 3 / 4) and assert rc, stamp presence, and whether the cache
      refresh ran. `2949ad0`
- [x] (must-fix) The launch-time refresh landed in root's cache, not the agent's — `.devcontainer/agent/agent-entrypoint.sh:103-108`, `.devcontainer/agent/Dockerfile:146-149`
      Both caches are refreshed now: root's (the entrypoint's own `rosdep check`/
      `rosdep install` run as root) and `$TARGET_USER`'s, via `setpriv` with
      `HOME=$TARGET_HOME` so the cache files land user-owned. `TARGET_HOME` moved
      to the top of the script, where step 4 can use it under `set -u`. The
      Dockerfile comment that asserted the baked non-root cache covers a mounted
      workspace's sources is corrected. The test slices the real refresh block out
      of the entrypoint and executes it against stub `rosdep`/`setpriv`, asserting
      two refreshes, one of them dropped to the target user with its HOME and
      ROSDEP_SOURCE_PATH, and none at all for a comment-only local list. `e1ac541`
- [x] (must-fix) Nothing validated a project `rosdep.yaml` before it drove a root-level install — `.agent/scripts/rosdep_local_sources.sh`, `.agent/scripts/stage_rosdep_manifests.sh`, `.agent/knowledge/dependency_policy.md`
      New `.agent/scripts/rosdep_yaml_validate.sh` accepts exactly
      `<key>: {<os>: [<package>, ...]}` — list form only, plain package names —
      and names what it rejected. Nested mappings (where `pip`/`npm`/`gem`/
      `source` rules live, and the codename-keyed form) are rejected; the
      installer keys are named explicitly in the message. Every path that can
      carry a key to a root install runs it first and **fails closed** when it
      cannot run at all (exit 3): `rosdep_local_sources.sh` and
      `stage_rosdep_manifests.sh` exclude the file and exit 4, `ci_local.sh`
      refuses the run (validating the **attested** content, not the working
      tree), and `rosdep_local_staleness_check.sh` reports it as a finding so
      `make validate` says so. `bootstrap.sh` distinguishes exit 4 (directory
      written, one file dropped) from exit 3. The rule is stated in the policy
      note with a table of what a rejection does on each path, and in the
      AGENTS.md script reference. 15 new validator tests + cross-script tests +
      a ci_local refusal test. `0f0b6c2`
- [x] (must-fix) plan.md still said "no new note fields needed" — `.agent/work-plans/issue-654/plan.md:284`
      The ADR-0018 row now records the `+rosdep-local` steps token and the
      `rosdep-local:` line, why they exist, and that notes on repos without a
      `rosdep.yaml` stay byte-identical. Files to Change and the approach gained
      the shape gate and the three cascade targets. `7b6c7e5`, `94871af`
- [x] (suggestion) Staged yamls keyed by repo basename collided — `.agent/scripts/stage_rosdep_manifests.sh:105-110`
      Staged as `<layer_ws>__<repo>.yaml`; the Dockerfile globs the directory so
      it is unaffected. A test stages two same-named repos from different layers
      and asserts both survive with their own keys, plus one pinning that the
      aggregation path (keyed by absolute path) never collided. `8e2afab`
- [x] (suggestion) ADR-0018 had no format-extension paragraph — `docs/decisions/0018-local-first-ci-verification.md`
      Added, in the shape of #577's: what the note now carries, why an environment
      that resolved a key from a repo-carried source is a different verified
      environment, that a file failing the shape gate is refused rather than
      attested, and that notes on repos without a `rosdep.yaml` are unchanged.
      `6cabe0d`
- [x] (suggestion) The hosted-CI recipe did not cascade — `.agent/templates/ci_workflow.yml:35-42`, `.claude/skills/onboard-project/SKILL.md:180`
      The template carries the step, before `rosdep update`, without `sudo` (the
      job runs in a `ros:jazzy-*` container as root) and guarded by
      `hashFiles('rosdep.yaml')` so it is a no-op in repos with no local keys.
      `onboard-project` says to keep it and where the policy lives; the note says
      which variant is which. `53e93cd`
- [x] (suggestion) The policy note never stated the trust implication — `.agent/knowledge/dependency_policy.md`
      New section: a `rosdep.yaml` merged in any project repo is fed to a
      root-level `rosdep install -y` on the dev host, in the `ci_local` container
      and in the agent image, so reviewing a one-line yaml is reviewing what
      installs as root across three environments. `74a9314`
- [x] (suggestion) README's `make validate` line — `README.md:220`
      Updated, and the same stale line in AGENTS.md § Build & Test. `63dcacb`

### Deferred

None — every finding was actioned.

### Tests run

| What | Result |
|---|---|
| `make test-scripts` (worktree, no ROS env) | ✅ all shell tests + 220 pytest passed |
| `.agent/scripts/tests/test_rosdep_local_sources.sh` | ✅ 81 passed (was 39; +42 for the shape gate, the Makefile stamp, the entrypoint refresh and the staging collision) |
| `.agent/scripts/tests/test_ci_local.sh` | ✅ 91 passed (was 88; +3 for the shape refusal) |
| `.agent/scripts/tests/test_make_validate.sh` | ✅ 7 passed |
| `.agent/scripts/tests/test_agent_image_build_paths.sh` | ✅ 7 passed |
| `make validate` (worktree) | `validate_workspace.py` exits 3 (no `configs/manifest` in a workspace worktree — the pre-existing, expected state); layer-sourcing guard PASSED; `rosdep-local: no rosdep.yaml files found` |
| Aggregation smoke-run against the real workspace root (out\_dir in scratch) | ✅ 2 system lists copied, 0 local yamls, exit 0 |
| pre-commit | ✅ on every commit; never `--no-verify` |

### Still owed (carried forward from the implementation entry)

- The agent image must be rebuilt after merge (`make agent-build`) — this round
  changed `agent-entrypoint.sh` again, and the startup scripts bake from the
  MAIN checkout.
- The Dockerfile bake itself was not executed (no Docker build in this
  dispatch); the staging half is covered by tests.
- `rosdep install` was never run (needs sudo).
