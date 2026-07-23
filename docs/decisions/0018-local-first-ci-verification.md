# ADR-0018: Local-first CI verification for project-repo merges

## Status

Accepted. Phase 1 governance of the local-first quality gates umbrella
([#572](https://github.com/rolker/ros2_agent_workspace/issues/572));
tool delivered in [#573](https://github.com/rolker/ros2_agent_workspace/issues/573)
(`.agent/scripts/ci_local.sh`).

## Context

Project-repo merges were gated on green GitHub Actions CI. That gate has two
structural problems and one strategic mismatch:

1. **Latency and fragility we don't control.** Hosted runners rebuild a ROS
   container from bare `apt-get` on every run. On 2026-07-22 a routine
   ~4-minute run took 2h21m to pass, and a sibling run failed outright on
   Ubuntu-mirror connection errors — before ever reaching checkout. The
   verdicts, when they arrived, matched what local verification had
   established hours earlier in seconds.
2. **The gate measures the wrong thing.** What the merge decision needs is
   "this commit builds and its tests pass in a clean environment" — not
   "GitHub's runner fleet and Canonical's mirrors were healthy today."
3. **Strategic mismatch.** Field hosts have no GitHub credentials by design
   (ADR-0011 field mode), and upcoming operations are fully
   over-the-horizon. The workspace is converging on offline-capable
   workflows; a hosted-only merge gate points the other way.

`ci_local.sh` (#573) closes the verification gap locally: it runs the CI
template steps (rosdep → colcon build → colcon test → test-result, plus
per-repo extras) in a throwaway container, builds attestable runs from a
pristine `git archive HEAD` snapshot (the local equivalent of
`actions/checkout`), and appends an auditable record — image, packages,
scope, host, log sha256 — to a git note at `refs/notes/ci-local` on the
tested commit. Three project-repo PRs merged on such attestations on
2026-07-22 (unh_echoboats_project11 #353, #379, #383).

The same day also demonstrated the counter-case: the **workspace repo's**
hosted checks are container-free, complete in seconds, and caught a real
`ci_local.sh` bug (git-notes identity missing on clean runners) that a
locally-configured machine could not have caught. Hosted CI's value is not
zero; it is environment diversity.

## Decision

1. **A full-scope ci_local attestation is an accepted merge verification
   for project-repo PRs.** A PR whose head commit carries a
   `refs/notes/ci-local` record with `ci-local: pass` and `scope: full`
   may be merged without waiting for hosted Actions. Verify with:

   ```bash
   git notes --ref=ci-local show <head-sha>
   ```

2. **Partial or unattested runs do not satisfy the gate.** `pass (partial)`
   records (a `--packages`-narrowed run), dirty-tree runs, and `--no-attest`
   runs are iteration aids, not merge evidence.

3. **Hosted CI on project repos remains configured, as a mirror and
   backstop — never a blocker.** Post-merge hosted failures are triaged:
   infrastructure failures (e.g. mirror outages) are noted and re-run at
   convenience; genuine code failures are treated as any post-merge
   regression — the local and hosted environments disagreed, and the
   disagreement itself is a bug to chase.

4. **The workspace repo (`ros2_agent_workspace`) is exempt: its hosted
   checks stay required.** They are fast (no ROS container), and they
   provide the clean-environment diversity that local runs cannot
   (proven the day this ADR was drafted).

5. **Attestation notes are pushed at merge time** for auditability beyond
   the merging machine:

   ```bash
   git push origin refs/notes/ci-local
   ```

6. **Preference order for the attestation image**: the agent sandbox image
   (deps baked, #520) for routine merges; `--clean-room` (stock
   `ros:jazzy-ros-core`, exact hosted mirror) when the change touches
   dependency declarations or the build environment itself. Records
   append, so a clean-room record is never destroyed by a later fast-path
   re-run.

## Consequences

- Merge latency for project repos drops from runner-weather-dependent to
  seconds-to-minutes, and merging works offline (field mode, OTH ops) —
  the original #572 motivation.
- The merge operator's machine becomes part of the trust chain. The
  attestation records host and log hash, snapshots are built from commit
  content only, and the note format is injection-guarded — but this is
  attestation, not proof; the honest-operator assumption is explicit and
  matches the rest of the workspace's local-first tooling. Server-side
  enforcement (pre-receive verification of attestations) is Phase 3 of
  #572, not this ADR.
- Hosted CI failures on project repos lose their blocking teeth; the
  discipline of triaging post-merge hosted failures (decision 3) is what
  keeps the mirror honest. If that triage lapses, environment drift
  between local images and hosted CI will accumulate silently.
- `merge_pr.sh` does not yet check for the attestation before merging;
  wiring that check in (warn or refuse when the head lacks a full-scope
  record) is a natural hardening follow-up under #572.
- **Repos with `upstream.repos` source dependencies (#577)**: `ci_local.sh`
  builds the upstream sources as an underlay (entries parsed, validated, and
  resolved to commit SHAs host-side before the container runs) and records one
  `upstream-repo: <dir>@<sha>` line per entry in the note; rosdep keys skipped
  via `.agents/ci_local_rosdep_skip_keys.txt` are likewise recorded
  (`rosdep-skip-keys:` line) so the note shows deps the verified environment
  deliberately did not install. For such repos a
  note lacking `upstream-repo:` lines that resolve every `upstream.repos`
  entry is **not valid merge evidence**, even if it says `ci-local: pass` /
  `scope: full` — the upstream state that was built would be unrecorded. This
  is a format extension only: notes on repos without `upstream.repos` are
  byte-identical to before and their meaning is unchanged. Since upstream
  clones need network anyway, `--clean-room` is the natural mode for these
  repos — it replicates hosted CI exactly and avoids relying on the agent
  image having the upstream sources' rosdep deps baked (the #520 bake covers
  layer manifests, not upstream workspaces).
- The hosted-CI speed fix (prebuilt/GHCR image, cube-style) remains worth
  doing independently — a faster, less fragile mirror is a better mirror.

## Alternatives considered

- **Keep waiting for hosted CI** (status quo): rejected — see Context; the
  gate primarily measured third-party infrastructure health.
- **Prebuilt CI image only, keep the hosted gate**: improves latency but
  not availability (mirror outages still block merges) and does nothing
  for offline/field operation. Pursued anyway as a mirror improvement,
  not as the gate.
- **Self-hosted runners**: keeps the Actions UX but adds standing
  infrastructure to operate; Phase 3's Forgejo direction (#572) subsumes
  this more coherently.
- **Making attestation mandatory for every merge including the workspace
  repo**: rejected — decision 4; the workspace repo's hosted checks are
  cheap and provide unique clean-room value.
