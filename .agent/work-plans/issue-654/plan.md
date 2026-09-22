# Plan: Local rosdep keys: workspace-owned ROSDEP_SOURCE_PATH aggregating per-repo rosdep.yaml, upstream PRs owed at merge

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/654

## Context

`python3-pystac` and `snakemake` (needed by rolker/unh_marine_autonomy#397)
are Ubuntu 24.04 packages with no rosdep key upstream. The owner's decision
(2026-09-22): local rosdep entries first, upstream `ros/rosdistro` PRs when
the consuming PR is ready to merge. Verified on this host: rosdep honors
`ROSDEP_SOURCE_PATH` (its `sources.list.d` override) — a directory holding a
copy of the system `20-default.list` plus a `30-workspace-local.list` with
`yaml file://<repo>/rosdep.yaml` lines, followed by `rosdep update`, resolves
both keys with no edit under `/etc`.

Today there is no `ROSDEP_SOURCE_PATH` override anywhere in `.agent/scripts/`
or the `Makefile`. `bootstrap.sh` and `ci_local.sh`'s inner script both
`rosdep init`/`rosdep update` against the system default list only. The
agent container image bakes `package.xml`-derived rosdep deps at build time
via `stage_rosdep_manifests.sh` (#520) — that mechanism is orthogonal (it
stages manifests into the Docker build context; it never touches
`sources.list.d`) and needs its own, separate hookup for the local list.

The per-repo `rosdep.yaml` declaration itself (upstream rosdistro format,
`ubuntu:`/`debian:` lists, a comment per key naming the package and that an
upstream PR is owed) is out of scope here — it lives in the consuming
project repo (unh_marine_autonomy#397) and this issue's mechanism must stay
generic, with no repo names baked into workspace scripts.

## Approach

1. **`.agent/scripts/rosdep_local_sources.sh`** — new execute-only script,
   modeled on `stage_rosdep_manifests.sh`'s structure and header-comment
   style. `Usage: rosdep_local_sources.sh <workspace_root> [<out_dir>]`
   (`out_dir` defaults to `<workspace_root>/.rosdep/sources.list.d`).
   - Refuses to run if sourced (same guard pattern).
   - Rebuilds `<out_dir>` idempotently (`rm -rf` + `mkdir -p`, matching
     `stage_rosdep_manifests.sh`).
   - Copies **every** `*.list` file from the system
     `/etc/ros/rosdep/sources.list.d/` into `<out_dir>/` — not just
     `20-default.list`. Verified during implementation: this host also
     carries a `10-local.list`, and copying only the default would
     silently drop a source the caller already had (resolving Open
     Question 1 in the direction of "copy them all"). Errors clearly
     (exit 3) when the system directory holds no `.list` file at all —
     rosdep isn't initialized yet; tell the caller to run `bootstrap.sh`
     first rather than silently producing a half source list.
   - Globs `<workspace_root>/layers/main/*_ws/src/*/rosdep.yaml`
     (non-recursive — one per project-repo root, per the issue's
     declaration format) and writes one `yaml file://<abs-path>` line per
     match into `<out_dir>/30-workspace-local.list`, sorted for
     determinism.
   - Prints the count of `rosdep.yaml` files aggregated (stdout), for the
     Makefile stamp / staleness check to key off, mirroring
     `stage_rosdep_manifests.sh`'s "Staged N manifest(s)" print.
   - Exits 0 even with zero `rosdep.yaml` files found (empty local list is
     a valid, common state) — same "degrade gracefully" posture as
     `stage_rosdep_manifests.sh`.
   - `.rosdep/` added to the root `.gitignore` (generated, workspace-root
     only — mirrors `.devcontainer/agent/.rosdep-manifests/`'s existing
     gitignore entry for the sibling mechanism).

2. **`setup.bash` export** — after the existing layer-sourcing block, add:
   resolve the workspace root the same way the existing `_VENV_ROOT`
   worktree-fallback block does (worktree → main root hop), and
   `export ROSDEP_SOURCE_PATH="<resolved-root>/.rosdep/sources.list.d"`
   whenever that directory exists (don't error if it's never been
   generated — e.g. a fresh clone before `make build`'s first run; rosdep
   just falls back to its own default when the env var points at nothing
   usable... actually rosdep does NOT fall back gracefully if the path is
   set but empty/missing, so only export when the directory exists,
   otherwise leave `ROSDEP_SOURCE_PATH` unset so rosdep uses its real
   default). This must run in every shell that sources `setup.bash`,
   worktree or main.

3. **`bootstrap.sh` step 4 rewrite** — replace the bare
   `rosdep init`/`rosdep update` block with: keep the existing system
   `rosdep init` guard (unchanged — `ROSDEP_SOURCE_PATH` overrides
   `sources.list.d`, it doesn't replace `rosdep init`, which still writes
   `/etc/ros/rosdep/sources.list.d/20-default.list` — the file
   `rosdep_local_sources.sh` copies from), then call
   `rosdep_local_sources.sh "$ROOT_DIR"` (workspace root, resolved the
   same way the rest of `bootstrap.sh` already does) before the final
   `rosdep update`, and export `ROSDEP_SOURCE_PATH` for that `rosdep
   update` call so it picks up the freshly generated local list
   immediately (a fresh shell wouldn't have `setup.bash`'s export yet).

4. **Makefile stamp** — new `$(STAMP)/rosdep-local.done` target,
   depending on `$(STAMP)/manifest.done` (needs `layers/` checked out)
   plus a **plain** `$(wildcard $(MAIN_ROOT)/layers/main/*_ws/src/*/rosdep.yaml)`
   prerequisite list. No `.SECONDEXPANSION` (Plan Review finding 4):
   unlike `$(STAMP)/layer-%.done`, this target has no `%`, so nothing in
   its prerequisites needs deferring to a second expansion pass — the
   wildcard expands correctly at parse time. Makefiles are re-parsed on
   every invocation, so the wildcard re-evaluates and the stamp goes
   stale when any project repo's `rosdep.yaml` changes (added, edited,
   or a repo with one gets checked out for the first time). Recipe: run
   `rosdep_local_sources.sh $(MAIN_ROOT)` then
   `ROSDEP_SOURCE_PATH=$(MAIN_ROOT)/.rosdep/sources.list.d rosdep update`.
   Add `$(STAMP)/rosdep-local.done` to `_build-layers`'s prerequisites
   (alongside `$(LAYER_STAMPS)`) so `make build` keeps the local source
   list current — this is the "make build's setup chain regenerate it"
   line from the issue, even though no layer's `build.sh` runs `rosdep
   install` today (rosdep install only happens in the CI paths below);
   the stamp's job is to keep the generated list fresh for whichever
   command consumes it next (`rosdep update`/`resolve`/`install` run
   ad hoc, or by the CI scripts in step 5).

5. **CI paths** (issue scope item 3, three sub-parts):
   - `ci_local.sh` inner script (~line 380-388): before the existing
     `rosdep init`/`rosdep update`/`rosdep install` block, copy in the
     tested repo's own `rosdep.yaml` (if `repo_file_exists
     "rosdep.yaml"`, using the same `repo_file_content` helper the
     `upstream.repos` and skip-keys parsing already use — works for both
     the attestable snapshot and the dirty-tree path) into a
     container-local `30-workspace-local.list`, and set
     `ROSDEP_SOURCE_PATH` for that container's `rosdep update`/`rosdep
     install` invocations. No dependency on the aggregation script inside
     the container — a single repo's `rosdep.yaml`, not the whole
     workspace glob, since `ci_local.sh` tests one repo in isolation.
   - Agent container image (`docker_run_agent.sh --build` /
     `stage_rosdep_manifests.sh` / `.devcontainer/agent/Dockerfile` /
     `agent-entrypoint.sh`) — **wire it** (Plan Review finding 1). The
     earlier "picks up local keys automatically" claim was false:
     nothing in the image ever sets `ROSDEP_SOURCE_PATH`, and the
     Dockerfile's `skip_keys` computation would drop an unresolvable
     local key into a single build-log line and carry on — the silent
     failure the Quality Standard says not to wave off. Four concrete
     pieces:
     - `stage_rosdep_manifests.sh` additionally stages every
       `layers/main/*_ws/src/*/rosdep.yaml` into
       `<stage_dir>/rosdep-local/<repo>.yaml` — a sibling directory
       inside the same staged tree the Dockerfile already `COPY`s, so no
       second build-context path, no second lock and no second cleanup
       trap in `docker_run_agent.sh`. It holds no `package.xml`, so
       `rosdep install --from-paths` ignores it. The script's summary
       line gains the local-yaml count.
     - The Dockerfile bake step copies those yamls to a persistent
       `/opt/rosdep-local/`, builds `/opt/rosdep-sources/` from **all**
       of `/etc/ros/rosdep/sources.list.d/*.list` plus a generated
       `30-workspace-local.list`, and exports `ROSDEP_SOURCE_PATH` for
       its own `rosdep update` / `rosdep check` / `rosdep install`. An
       `ENV ROSDEP_SOURCE_PATH=/opt/rosdep-sources` after that step makes
       the later non-root `rosdep update` and every container process
       inherit it.
     - Unresolvable keys stop being a one-line aside: the bake prints a
       clearly labelled `WARNING: rosdep bake: N key(s) NOT installed`
       block naming each skipped key **at the end** of the build step,
       after the install, so it is the last thing that layer's log says.
     - `agent-entrypoint.sh` sources `setup.bash` (step 2) before its
       `rosdep check`/`rosdep install` loop, so it inherits
       `ROSDEP_SOURCE_PATH` pointing at the **bind-mounted workspace's**
       generated dir (`docker_run_agent.sh` mounts the workspace at the
       same absolute path — `-v "$ROOT_DIR:$ROOT_DIR"` — so the generated
       `file://` URLs resolve inside the container). rosdep's cache is
       keyed by source URL and the baked cache was built from
       `/opt/rosdep-local/…` URLs, so the entrypoint runs one
       best-effort `rosdep update` when — and only when — a non-empty
       `30-workspace-local.list` exists at that path.
     - **#604 rebuild caveat**: `agent-entrypoint.sh` and
       `fix-volume-ownership.sh` are baked from the MAIN checkout and the
       launcher only builds when the image is *missing*. The agent image
       must be rebuilt after this PR merges (`make agent-build`);
       until then the startup-scripts staleness marker warns on every
       launch, which is the intended signal.
   - Hosted CI (project-repo workflow): the new knowledge note (step 7)
     carries a **concrete, copy-pasteable GitHub Actions step** (Plan
     Review finding 3 — prose describing the shape is not adoptable "in
     one step", which is the issue's own bar): `sudo mkdir -p` a sources
     dir, copy in every `*.list` from the system default, copy the repo's
     own `rosdep.yaml`, write the `yaml file://…` line, `rosdep update`,
     and export `ROSDEP_SOURCE_PATH` into `$GITHUB_ENV` so the workflow's
     existing `rosdep install` step picks it up unchanged. Not a code
     change here (each project repo owns its `.github/workflows/`), but
     the note is the durable reference `rosdep_local_sources.sh`'s header
     and future project-repo CI templates point to.

6. **Enforcement check** — new
   `.agent/scripts/rosdep_local_staleness_check.sh` (execute-only),
   invoked from the `validate` Makefile target (wired explicitly per the
   Issue Review gap). For each `rosdep.yaml` found by the same glob as
   step 1: parse its keys (reuse the `yaml.safe_load` pattern already used
   in `ci_local.sh`'s `upstream.repos` parsing), and for each key run a
   probe that can vouch for its own answer (Plan Review finding 2): a
   subshell with `ROSDEP_SOURCE_PATH` **unset** and `HOME` pointed at a
   throwaway temp dir (rosdep's cache is per-user at
   `$HOME/.ros/rosdep/sources.cache`, so probing under the caller's HOME
   would rewrite the very cache the local override populates), one
   `rosdep update` **inside that subshell first**, then `rosdep resolve
   <key>` per key. If that `rosdep update` fails — offline, no network in
   CI, `rosdep` not installed — every key is reported **SKIPPED**, never
   a resolves/does-not-resolve verdict read off a cache state the probe
   cannot vouch for (same posture as `ci_local.sh`'s "tolerate rosdep
   update failure (offline field host)"). A key that resolves against the
   default sources alone is flagged: the local entry is stale and ready
   to delete once the noted upstream PR has actually merged. Also
   flag any key in a `rosdep.yaml` whose comment doesn't mention "PR" or a
   `ros/rosdistro` URL (heuristic for "upstream PR owed" bookkeeping,
   matching the per-repo declaration format the issue specifies). Wire
   into the `validate` target: add a third `rc=$$?` branch alongside
   `validate_workspace.py` and `test_layer_sourcing.sh`, same
   accumulate-and-report-worst-exit-code pattern already in that recipe.
   Exit codes, deliberately distinct so that accumulation can tell them
   apart: **0** clean (including "no `rosdep.yaml` files found" — this
   check must never fail a fresh workspace with zero local keys); **1** a
   local key now resolves upstream, or a key lacks the upstream-PR
   marker; **2** usage error; **3** SKIPPED (offline / no rosdep). The
   `validate` recipe reports 3 as a printed notice and does **not** fail
   on it, exactly as the workspace already treats offline rosdep state
   elsewhere; only 1 and 2 fail.

7. **`.agent/knowledge/dependency_policy.md`** — new knowledge doc: the
   rule as tested here (ROS packages depend only on what rosdep resolves;
   Ubuntu-shipped-but-unkeyed libraries get a local `rosdep.yaml` key then
   an upstream `ros/rosdistro` PR at the consuming PR's merge time;
   libraries Ubuntu does not ship at all are the explicitly open case —
   venv with system site-packages, never pip-installed native geospatial
   libs — deferred until one actually appears, tracked in
   rolker/agent_workspace#310). Includes the hosted-CI recipe from step 5
   and a pointer to `rosdep_local_sources.sh` / the `validate` staleness
   check.

8. **AGENTS.md Script Reference row** (Issue Review gap #1) — add a row
   for `.agent/scripts/rosdep_local_sources.sh` (and
   `rosdep_local_staleness_check.sh`) in the same table, following the
   existing entries' style: usage, exit codes, what it reads/writes, who
   calls it (`bootstrap.sh`, the new Makefile stamp, `ci_local.sh`).

9. **Tests** — `.agent/scripts/tests/test_rosdep_local_sources.sh`: temp
   workspace fixture with two fake repos under
   `layers/main/fake_ws/src/`, one with a `rosdep.yaml`, one without;
   assert the generated `30-workspace-local.list` has exactly one line
   pointing at the right absolute path; assert idempotent regeneration
   (run twice, same output); assert the missing-`20-default.list` error
   path. A second test (or a second section of the same file) covers the
   staleness check: a fixture `rosdep.yaml` with a key that resolves
   upstream (`python3-numpy`) is flagged, one that doesn't resolve
   (`snakemake`) is not. `Makefile`'s `run_script_tests.sh` picks up new
   `test_*.sh` files automatically (verify against its glob before
   assuming this — check during implementation).

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/rosdep_local_sources.sh` | New — aggregation script (step 1) |
| `.agent/scripts/rosdep_local_staleness_check.sh` | New — enforcement check (step 6) |
| `.agent/scripts/setup.bash` | Export `ROSDEP_SOURCE_PATH` when `.rosdep/sources.list.d` exists (step 2) |
| `.agent/scripts/bootstrap.sh` | Call aggregation script before final `rosdep update` (step 3) |
| `.agent/scripts/ci_local.sh` | Inner script: install tested repo's own `rosdep.yaml` via `ROSDEP_SOURCE_PATH` (step 5) |
| `.agent/scripts/stage_rosdep_manifests.sh` | Stage per-repo `rosdep.yaml` into `<stage_dir>/rosdep-local/` for the image bake (step 5) |
| `.devcontainer/agent/Dockerfile` | Build `/opt/rosdep-sources`, export `ROSDEP_SOURCE_PATH`, WARN block naming unresolved keys (step 5) |
| `.devcontainer/agent/agent-entrypoint.sh` | Best-effort `rosdep update` when the mounted workspace has a non-empty local list (step 5) |
| `Makefile` | New `$(STAMP)/rosdep-local.done` stamp + `_build-layers` prereq + `validate` target third branch (steps 4, 6) |
| `.gitignore` | Add `.rosdep/` |
| `.agent/knowledge/dependency_policy.md` | New knowledge note (step 7) |
| `AGENTS.md` | Script Reference table rows for both new scripts (step 8) |
| `.agent/scripts/tests/test_rosdep_local_sources.sh` | New test file (step 9) |
| `.agent/scripts/tests/test_make_validate.sh` | Stub the new check so the `validate`-recipe regression test still exercises the real recipe (step 6 consequence) |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Enforcement over documentation | Step 6's `validate`-wired staleness check pairs with step 7's knowledge note — the policy isn't just written down, a check nags when a local key should be deleted. |
| Only what's needed | Amended after Plan Review: the container image is a *named in-scope CI path* in the issue, and its `skip_keys` design would silently mask a missing local key, so step 5 wires it rather than documenting a claim that was false. Still no speculative plumbing — nothing is added for the "Ubuntu doesn't ship it at all" case. |
| A change includes its consequences | AGENTS.md script table + Makefile `validate` wiring both included explicitly (Issue Review gaps 1 and 3), plus `.gitignore`. |
| Workspace vs. project separation | `rosdep_local_sources.sh` glob-scans generically (`layers/main/*_ws/src/*/rosdep.yaml`); no repo names anywhere in the new scripts or knowledge note. |
| Test what breaks | Step 9 covers idempotent regen, the missing-`20-default.list` failure path, and both staleness-check outcomes (flagged vs. not). |
| Improve incrementally | Declared 5 scope items map to 9 approach steps but stay one cohesive PR — the pieces share the same glob and the same generated directory, splitting them would leave the aggregation script half-wired. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0003 — Project-agnostic workspace | Yes | All new scripts operate on the generic `layers/main/*_ws/src/*/rosdep.yaml` glob; no BizzyBoat/unh_marine_autonomy names anywhere in workspace code. |
| 0004/0005 — Enforcement hierarchy | Yes | Staleness check runs via `make validate`, consistent with `validate_workspace.py`'s existing local-only role (not wired into hosted CI, matching current practice — see Issue Review's note that `validate.yml` doesn't call `make validate` today; not a gap this PR needs to close). |
| 0009 — Python package management | Yes (Tier 1 reference only) | `python3-pystac`/`snakemake` are the consumer's Tier-1 apt/rosdep case; this PR builds the generic mechanism, doesn't touch `.venv` (Tier 2/3). No conflict. |
| 0018 — Local-first CI verification | Yes | `ci_local.sh`'s inner script gets the local-source install step (step 5); the existing `upstream-repo:`/`rosdep-skip-keys:` attestation note lines are untouched — the local-source install happens before the existing `rosdep install --from-paths ... --skip-keys` line, same combined install, no new note fields needed since this doesn't change what's skipped or what upstream SHA was used. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `.agent/scripts/` gains two new scripts | AGENTS.md Script Reference table | Yes — step 8 |
| `Makefile` gains a new stamp + validate branch | `make clean`'s stamp-reset already globs `$(STAMP)/*` generically (verify during implementation — no plan change expected) | Yes — verify, no separate step needed |
| New `.rosdep/` generated directory | `.gitignore` | Yes — listed in Files to Change |
| New enforcement check can produce false positives on a key still mid-transition | Knowledge note explains the intended lifecycle (flag → confirm upstream PR merged → delete local entry) so a flag isn't read as an immediate build break | Yes — step 7 |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): None — this is entirely new
  plumbing; `AGENTS.md`'s Script Reference table gets new rows (additive,
  not a correction of stale content) and `stage_rosdep_manifests.sh`'s
  header comment gets a clarifying note about the local-key interaction
  (step 5).
- **Agent-instruction candidates** (proposals only): the "Ubuntu doesn't
  ship it at all → venv, deferred" open case from the knowledge note
  could eventually warrant its own ADR once a real library forces the
  decision — explicitly not now, per the workspace's no-premature-ADRs
  lesson and the issue's own "tried here before written down elsewhere"
  framing (rolker/agent_workspace#310 tracks the broader write-up).

## Open Questions

- [x] **Resolved during implementation (2026-09-22)**: a second system
      source file *does* exist on this host (`10-local.list` alongside
      `20-default.list`), so `rosdep_local_sources.sh` copies **every**
      `*.list` in `/etc/ros/rosdep/sources.list.d/` rather than naming
      one file. Step 1 amended.
- [x] **Resolved during implementation (2026-09-22)**:
      `run_script_tests.sh` discovers tests by glob
      (`"$SCRIPTS_DIR"/test_*.sh` and `"$TESTS_DIR"/test_*.sh`), so a new
      `test_rosdep_local_sources.sh` is picked up with no registration.
- [ ] The staleness check's "upstream PR owed" comment heuristic (looking
      for "PR" or a `ros/rosdistro` URL in the yaml comment) is a
      best-effort text match, not a structured field — acceptable for a
      first pass per the issue's own scope, but worth a one-line callout
      in the knowledge note that it's advisory, not authoritative.

## Estimated Scope

Single PR — the pieces share one generated directory and one glob
pattern; splitting further would leave partial, non-functional plumbing
at each intermediate step (matches the Issue Review's "interdependent
enough that one PR is defensible" verdict).
