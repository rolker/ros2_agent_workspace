# Plan: ci_local.sh: support upstream.repos source dependencies (blocks ADR-0018 attestation for repos like cube_bathymetry)

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/577

## Context

`ci_local.sh` (#573, ADR-0018) mounts only the target repo into `/ci/src/<repo>`
and never reads an `upstream.repos` file, so any repo with unreleased source
dependencies (today: `cube_bathymetry`, depending on `unh_marine_autonomy`,
`mru_transform`, `geographic_info`) fails immediately at rosdep/CMake
resolution and can never produce a `ci-local: pass` attestation. Hosted CI for
these repos uses `industrial_ci`'s `UPSTREAM_WORKSPACE` mechanism, plus
repo-specific `ROSDEP_SKIP_KEYS` and `AFTER_SETUP_UPSTREAM_WORKSPACE` env vars
to prune the upstream monorepo down to what's actually needed.

**Operator checkpoint decisions (settled, not reopened by this plan):**
1. Strategy = `vcs import`, not industrial_ci delegation. When
   `<repo>/upstream.repos` exists, `ci_local.sh` vcs-imports it into an
   upstream workspace inside the container, builds it, and sources it before
   building the target — with pruning expressed via a hook generalized from
   the existing `<repo>/.agents/ci_local_extra.sh` precedent.
2. The `ci-local` attestation note format is extended in this PR to record
   resolved upstream commit SHAs per `upstream.repos` entry, so `scope: full`
   attestations stay self-describing merge evidence per ADR-0018 even though
   `upstream.repos` entries pin to floating branches, not SHAs.

## Approach

1. **Detect `upstream.repos`.** After repo-root/mount validation, check for
   `$REPO/upstream.repos`. If absent, behavior is unchanged (today's
   single-repo path). If present, parse it with `python3 -c` + `yaml` (already
   a rosdep/vcstool runtime dependency, avoids a new hand-rolled YAML parser)
   into a list of `(local_dir, url, version)` tuples — this happens on the
   **host**, before the container runs, so injection-unsafe values are caught
   before they reach any shell.

2. **Validate parsed fields before interpolation.** Mirror the existing
   package-name validation: each `local_dir` must match
   `^[A-Za-z0-9_-]+$`; each `version` must match a conservative git-ref
   charset (`^[A-Za-z0-9_./-]+$`); each `url` must not contain whitespace or
   shell metacharacters that would break out of a single-quoted argument.
   Reject with a clear error (naming the offending repo) rather than passing
   anything unsanitized into the inner container script — same threat model
   as the existing package-name check (attestation forgery via injection).

3. **Generalize the per-repo hook.** Add
   `<repo>/.agents/ci_local_upstream_extra.sh` (executable, optional) as the
   upstream-side analog of `ci_local_extra.sh`: run inside the container
   after `vcs import` of the upstream workspace but before `rosdep install`/
   `colcon build` of it, with the upstream workspace as cwd. This is where a
   repo like cube_bathymetry expresses its
   `AFTER_SETUP_UPSTREAM_WORKSPACE`-equivalent pruning (`touch
   COLCON_IGNORE` in subpackages it doesn't need) — imperative, per-repo,
   never baked into the generic script (ADR-0003).

   Add `<repo>/.agents/ci_local_rosdep_skip_keys.txt` (optional, one key per
   line, comments with `#`) as the declarative analog of `ROSDEP_SKIP_KEYS`:
   read on the host, validated against `^[A-Za-z0-9_-]+$` per line (same
   injection threat as package names), and passed to the single combined
   `rosdep install --from-paths src --ignore-src -r -y --skip-keys "..."`
   call that covers both the upstream and target workspaces.

4. **Extend the inner container script.** When upstream is present:
   - `vcs import upstream_ws/src < upstream.repos` (host writes the validated
     repos file into the snapshot/live tree copy that's mounted, or the
     parsed+re-validated tuples are passed as env/args — see step 7 on the
     snapshot boundary).
   - run `ci_local_upstream_extra.sh` if present.
   - `rosdep install --from-paths src --ignore-src -r -y` across
     `/ci/upstream_ws/src` and `/ci/src` together (single call, matching
     industrial_ci's combined resolution and avoiding order-dependent
     partial installs), with `--skip-keys` from step 3.
   - `colcon build` the upstream workspace first (`--base-paths upstream_ws`
     or a merged workspace layout — pick whichever keeps target-repo
     `--packages-up-to` semantics intact; see Open Questions), `source
     upstream_ws/install/local_setup.bash`, then proceed with the existing
     target-repo build/test steps unchanged.

5. **Capture resolved upstream SHAs for the attestation.** After `vcs
   import`, the inner script writes one line per upstream repo (`<local_dir>
   <resolved-sha>`) to a small file in a directory the host bind-mounts
   read-write for this purpose only (e.g. `$LOG_DIR/.upstream-<ts>/`,
   cleaned up after the run) — the rest of the container's mounts stay
   read-only, unchanged. The host reads this file after a successful run and
   folds it into the note (step 6). If the file is missing/malformed after a
   run that used `upstream.repos`, treat it as a hard failure (never attest
   silently-unresolved upstream state).

6. **Extend the attestation note format.** Add one `upstream-repo:` line per
   entry, appended after `packages:`:

   ```
   ci-local: pass
   repo: cube_bathymetry
   commit: <sha>
   image: ...
   packages: ...
   scope: full
   upstream-repo: unh_marine_autonomy@<resolved-sha>
   upstream-repo: mru_transform@<resolved-sha>
   upstream-repo: geographic_info@<resolved-sha>
   steps: template+upstream+extra-hook
   date: ...
   host: ...
   log-sha256: ...
   ```

   `steps:` gains an `upstream` token (and keeps `+extra-hook` when
   `ci_local_extra.sh` — the *target*-repo hook — also ran) so a note reader
   can tell at a glance whether upstream sources were part of the attested
   build. No `upstream.repos` in the repo → note format is byte-identical to
   today (no regression for existing attested repos).

7. **`--dry-run` plan output.** Add a section showing the parsed upstream
   repos (dir/url/version), whether `ci_local_upstream_extra.sh` and
   `ci_local_rosdep_skip_keys.txt` were found, and the `steps` value —
   mirroring the existing dry-run detail level for packages/image/scope.

8. **ADR-0018 doc update.** Add a Consequences-section note (or a short
   "Decision 1a" style addendum — whichever reads better once drafted)
   stating that `scope: full` attestations on repos with `upstream.repos`
   also require `upstream-repo:` lines resolving every entry, and that a
   note lacking them for a repo that has `upstream.repos` is not valid merge
   evidence even if it says `ci-local: pass`/`scope: full`. This is a note
   **format extension**, not a redefinition of what already-recorded notes
   (repos without `upstream.repos`) mean.

9. **AGENTS.md reference-line update.** Extend the `ci_local.sh` row in the
   Script Reference table to mention `upstream.repos` support in one clause.

10. **Tests.** Extend `.agent/scripts/tests/test_ci_local.sh` (docker still
    fully stubbed — no real container/network/vcs execution in the test
    harness, consistent with existing coverage):
    - Fixture repo gains a sibling fixture with an `upstream.repos` file;
      dry-run shows the parsed upstream entries and `steps` includes
      `upstream`.
    - Injection-safety: malformed `local_dir` (shell metacharacters),
      malformed `version`, and a `url` with embedded whitespace/newline are
      each rejected before any container run, with a message naming the bad
      field — mirroring the existing malicious-package-name case.
    - `ci_local_upstream_extra.sh` presence is detected and reflected in
      `steps`; absence leaves `steps` unchanged from today's format (no
      `upstream.repos` case stays byte-identical — regression guard).
    - `ci_local_rosdep_skip_keys.txt` parsing: valid file accepted; a line
      failing the key-name charset is rejected.
    - Note-format case: docker stub is extended to fake-write the
      resolved-upstream-SHA output file (mirroring how it already fakes
      `docker image inspect`), so the full success path can assert the note
      gains the expected `upstream-repo:` lines and `steps: ...+upstream`.
    - Regression case: a repo *without* `upstream.repos` produces a note
      with no `upstream-repo:` lines and unchanged `steps` value — pins
      backward compatibility.

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/ci_local.sh` | Detect/parse/validate `upstream.repos`; extend inner container script (vcs import, upstream hook, combined rosdep with skip-keys, upstream build+source); capture resolved upstream SHAs; extend dry-run output and the attestation note format |
| `.agent/scripts/tests/test_ci_local.sh` | New fixture + cases per Approach step 10 |
| `docs/decisions/0018-local-first-ci-verification.md` | Consequences note: `upstream-repo:` lines required for `scope: full` validity on repos carrying `upstream.repos` |
| `AGENTS.md` | Extend the `ci_local.sh` Script Reference row to mention upstream.repos support |
| `.agent/scripts/ci_local.sh` (help text / header comment) | Document the new `<repo>/.agents/ci_local_upstream_extra.sh` and `<repo>/.agents/ci_local_rosdep_skip_keys.txt` hook files, same style as the existing `ci_local_extra.sh` doc comment |

No changes needed in `cube_bathymetry` itself for this PR — adopting the new
hooks there (splitting its industrial_ci `ROSDEP_SKIP_KEYS`/
`AFTER_SETUP_UPSTREAM_WORKSPACE` into the new files) is a natural follow-up
once the mechanism lands, tracked separately (see Consequences), not required
to close this issue (the issue only requires that `ci_local.sh` *can*
support `upstream.repos`, verified against the test fixtures).

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Workspace vs. project separation | Mechanism keyed off a well-known filename (`upstream.repos`) and optional hook files under `.agents/`, not a cube_bathymetry special-case; no repo name appears in `ci_local.sh` |
| Enforcement over documentation | Extends the existing enforced attestation tool rather than documenting a manual workaround |
| A change includes its consequences | ADR-0018 doc, AGENTS.md reference line, and test coverage are in this same plan, not deferred |
| Test what breaks | New parsing/validation/note-format logic gets stub-based tests before/alongside implementation, including a backward-compatibility regression case |
| Only what's needed | Two hook files (imperative prune + declarative skip-keys) mirror industrial_ci's two real mechanisms exactly — no speculative generality beyond what cube_bathymetry's actual CI config demonstrates is needed |
| Improve incrementally | Single PR; cube_bathymetry's own adoption of the new hooks is left as a follow-up, not bundled in |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0003 — Project-agnostic workspace | Yes | Generic filename-keyed detection; per-repo specifics (pruning, skip-keys) live in the project repo's own `.agents/` files, never in workspace script logic |
| 0018 — Local-first CI verification | Yes | Closes the Phase-1 tool gap for `upstream.repos` repos; note format extended (step 6) with a doc addendum (step 8) so the `scope: full` merge-evidence guarantee (Decision 1) is preserved rather than silently weakened |
| 0009 — Python package management | Watch | `vcs` (python3-vcstool) confirmed present on this dev host via `ros-dev-tools`; before implementation, confirm it's baked into `ros2-agent-workspace-agent:latest` (fast path) — if missing, the inner script's existing `ros-dev-tools` apt-install fallback (already present for `colcon`) covers `ros:jazzy-ros-core`, so no new dependency-provisioning step is needed either way |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `ci_local.sh` note format | ADR-0018 doc (Consequences section) | Yes — step 8 |
| `ci_local.sh` capability surface | `AGENTS.md` Script Reference | Yes — step 9 |
| `ci_local.sh` parsing/build logic | `test_ci_local.sh` | Yes — step 10 |
| New hook file convention (`ci_local_upstream_extra.sh`, `ci_local_rosdep_skip_keys.txt`) | cube_bathymetry adopts them, replacing its industrial_ci env vars | No — follow-up issue in `rolker/cube_bathymetry`, filed after this PR merges (adoption is optional; industrial_ci config keeps working independently as the hosted mirror per ADR-0018 Decision 3) |
| Upstream workspace build added to the local run | Wall-clock cost of `ci_local.sh` runs on `upstream.repos` repos | No code change — flagged as an accepted tradeoff in Open Questions; caching (ccache-equivalent) is out of scope for this PR |

## Open Questions

- [ ] Upstream + target workspace layout inside the container: build upstream
      as a separate underlay sourced before the target build (simpler,
      mirrors industrial_ci's two-workspace model), vs. a single merged
      `src/` tree built together (lets one `colcon build --packages-up-to`
      call cover both, but changes what "target repo only" build output
      means). Recommend the underlay approach — closer to industrial_ci
      semantics the hosted CI already validates, and keeps the existing
      target-repo build/test invocation unchanged.
- [ ] Whether to cache the built upstream workspace across repeated
      `ci_local.sh` runs on the same `upstream.repos` content (the issue
      flags this as dominating wall-clock, mirroring hosted CI's ccache
      use). Recommend deferring to a follow-up issue — this PR should land
      the correctness fix (attestable runs succeed at all) before optimizing
      repeat-run latency.

## Estimated Scope

Single PR. The change is additive and backward-compatible (no
`upstream.repos` → identical behavior/note format to today), concentrated in
one script plus its test file and two doc updates.
