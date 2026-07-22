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

1. **Detect `upstream.repos`.** After attestation eligibility is computed,
   check for `upstream.repos` — read from the **pristine HEAD commit content**
   (`git show HEAD:...`) for attestable runs, from the live tree for
   dirty/`--no-attest` runs, so an attested run can never consume inputs that
   differ from the recorded commit (same rule for the two hook files in step
   3). If absent, behavior is unchanged (today's single-repo path). If
   present, parse it with `python3 -c` + `yaml` (already a rosdep runtime
   dependency, avoids a new hand-rolled YAML parser) into a list of
   `(local_dir, url, version)` tuples — on the **host**, before the container
   runs, so injection-unsafe values are caught before they reach any shell.
   Entries must be `type: git` and carry an explicit `version` (attestation
   must resolve a concrete ref).

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

4. **Resolve upstream refs host-side, then extend the inner container
   script.** Before the container runs, the host resolves each entry's
   floating `version` to a commit SHA via `git ls-remote <url>
   refs/heads/<v> refs/tags/<v>` (a 40-hex `version` is used as-is);
   resolution failure is fatal. The inner script then, when upstream is
   present:
   - `git clone` each validated url into `/ci/upstream_ws/src/<dir>` and
     `git checkout --detach <resolved-sha>` — generated from the validated
     tuples, so no repos file and no `vcs`/vcstool are needed in the
     container at all (see Implementation Notes).
   - run `ci_local_upstream_extra.sh` (via bash, cwd `/ci/upstream_ws`) if
     present.
   - `rosdep install --from-paths src upstream_ws/src --ignore-src -r -y`
     (single combined call, matching industrial_ci's resolution and avoiding
     order-dependent partial installs), with `--skip-keys` from step 3.
   - `colcon build` the upstream workspace as a **separate underlay**
     (`--base-paths upstream_ws/src`, its own build/install bases — the
     resolved Open Question), `source upstream_ws/install/local_setup.bash`,
     then run the existing target-repo build/test steps pinned with
     `--base-paths src` so cwd discovery never re-includes the underlay.

5. **Attestation carries the host-resolved SHAs.** Because resolution happens
   on the host before the run (step 4), the note is written from values the
   host chose and the container was *told* to check out — no container
   output is trusted for attestation content, and every mount stays
   read-only, unchanged. Unresolvable upstream state fails the run before
   the container starts (never attest silently-unresolved upstream state).

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

8. **ADR-0018 doc update.** Add a Consequences-section note stating that
   `scope: full` attestations on repos with `upstream.repos` also require
   `upstream-repo:` lines resolving every entry, and that a note lacking
   them for a repo that has `upstream.repos` is not valid merge evidence
   even if it says `ci-local: pass`/`scope: full`. This is a note **format
   extension**, not a redefinition of what already-recorded notes (repos
   without `upstream.repos`) mean. Also note `--clean-room` as the natural
   mode for `upstream.repos` repos (clones need network anyway; agent-image
   deps for upstream sources aren't baked).

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
    - Note-format case: the fixture's `upstream.repos` points at a **local
      fixture git repo** (file-path url), so the host-side `ls-remote`
      resolution runs for real with no network and no docker-stub changes;
      the full success path asserts the note gains the expected
      `upstream-repo: <dir>@<real-sha>` line and `steps: ...+upstream`.
      An unresolvable-version case asserts failure before any container run.
    - Regression case: a repo *without* `upstream.repos` produces a note
      with no `upstream-repo:` lines and unchanged `steps` value — pins
      backward compatibility.

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/ci_local.sh` | Detect/parse/validate `upstream.repos` (pristine-at-HEAD for attestable runs); host-side ref→SHA resolution; extend inner container script (generated clones, upstream hook, combined rosdep with skip-keys, underlay build+source, `--base-paths src` pinning); extend dry-run output, the attestation note format, and the header doc comment (hook files) |
| `.agent/scripts/tests/test_ci_local.sh` | New fixtures + cases per Approach step 10 |
| `docs/decisions/0018-local-first-ci-verification.md` | Consequences note: `upstream-repo:` lines required for `scope: full` validity on repos carrying `upstream.repos`; `--clean-room` preference |
| `AGENTS.md` | Extend the `ci_local.sh` Script Reference row to mention upstream.repos support |

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
| 0009 — Python package management | Resolved | The implementation generates plain `git clone`/`git checkout` commands from host-validated tuples, so `vcs`/vcstool is **not needed at all** (host or container); the inner script gains only a `git` apt fallback beside the existing `ros-dev-tools` one |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `ci_local.sh` note format | ADR-0018 doc (Consequences section) | Yes — step 8 |
| `ci_local.sh` capability surface | `AGENTS.md` Script Reference | Yes — step 9 |
| `ci_local.sh` parsing/build logic | `test_ci_local.sh` | Yes — step 10 |
| New hook file convention (`ci_local_upstream_extra.sh`, `ci_local_rosdep_skip_keys.txt`) | cube_bathymetry adopts them, replacing its industrial_ci env vars | No — follow-up issue in `rolker/cube_bathymetry`, filed after this PR merges (adoption is optional; industrial_ci config keeps working independently as the hosted mirror per ADR-0018 Decision 3) |
| Upstream workspace build added to the local run | Wall-clock cost of `ci_local.sh` runs on `upstream.repos` repos | No code change — flagged as an accepted tradeoff in Open Questions; caching (ccache-equivalent) is out of scope for this PR |

## Open Questions

- [x] Upstream + target workspace layout: **resolved — separate underlay**
      (`upstream_ws` with its own build/install bases, sourced before the
      target build; target build/test pinned to `--base-paths src`). Mirrors
      industrial_ci's two-workspace model the hosted CI already validates.
- [ ] Whether to cache the built upstream workspace across repeated
      `ci_local.sh` runs on the same `upstream.repos` content (the issue
      flags this as dominating wall-clock, mirroring hosted CI's ccache
      use). Deferred to a follow-up issue — this PR lands the correctness
      fix (attestable runs succeed at all) before optimizing repeat-run
      latency.

## Estimated Scope

Single PR. The change is additive and backward-compatible (no
`upstream.repos` → identical behavior/note format to today), concentrated in
one script plus its test file and two doc updates.

## Implementation Notes

- **`vcs import` replaced by generated `git clone` + `checkout --detach`**:
  since every `(dir, url, version)` tuple is already host-validated, emitting
  explicit git commands removes the vcstool dependency entirely (host and
  container) and shrinks the container's parsing surface to zero — the
  container never reads `upstream.repos` at all. Same strategy (the
  workspace imports the upstream workspace itself), simpler mechanism.
- **Container log-marker SHA capture replaced by host-side pre-run
  `ls-remote` resolution**: the plan-review suggestion (parseable log marker,
  no RW mount) still left the note's upstream SHAs derived from container
  *output*, which the repo-under-test's own build/tests could forge. Resolving
  on the host *before* the run and telling the container which SHA to check
  out makes the note tamper-proof, keeps every mount read-only, and turns
  "unresolvable upstream" into a pre-container hard failure.
