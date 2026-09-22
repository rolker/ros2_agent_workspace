# Dependency Policy — Non-ROS Libraries and Local rosdep Keys

How a ROS package in this workspace declares a dependency on something that is
not itself a ROS package. The rule, the mechanism, and the bookkeeping the
mechanism owes back to upstream.

Decision recorded in
[#654](https://github.com/rolker/ros2_agent_workspace/issues/654) (2026-09-22).
Deliberately **not** an ADR yet: the mechanism should survive a second real
consumer before it is promoted. The broader write-up of the general policy is
tracked separately in `rolker/agent_workspace#310`.

## The rule

A ROS package's `package.xml` may only depend on **rosdep keys that resolve**.
That is not a style preference — `rosdep install` is all-or-nothing on key
resolution, and the agent image's bake works around it by computing
`--skip-keys`, which means an unresolvable key is *silently omitted* rather
than loudly failed.

Three cases, in order of how common they are:

| Case | What to do |
|---|---|
| Upstream `ros/rosdistro` already has a key | Just use it. Nothing else to do. |
| Ubuntu ships the package, but `ros/rosdistro` has no key for it | Declare a **local key** (below) **and** owe an upstream PR. |
| Ubuntu does not ship it at all | **Open case.** Do not pip-install native geospatial libraries system-wide. The likely answer is the workspace `.venv` with system site-packages (ADR-0009 Tier 2/3), but no instance has forced the decision yet — surface it when one does rather than guessing here. |

`python3-pystac` and `snakemake` are the Ubuntu 24.04 packages that drove case
2 (needed by `rolker/unh_marine_autonomy#397`).

## The mechanism: local keys via `ROSDEP_SOURCE_PATH`

rosdep reads its source list from `/etc/ros/rosdep/sources.list.d/`, which is
root-owned, invisible to CI, and undoable only by hand. `ROSDEP_SOURCE_PATH`
points it at a different directory instead — so the workspace can own its
sources without touching `/etc`.

**`ROSDEP_SOURCE_PATH` REPLACES `sources.list.d`; it does not add to it.**
Every generator in this workspace therefore copies the system `*.list` files
into the generated directory alongside the local one — *all* of them, not just
`20-default.list` (a host may carry a `10-local.list` or a site overlay, and
naming one file would silently drop the others).

### Declaring a key (project repo)

A project repo opts in by adding a `rosdep.yaml` at **its own root**, in
upstream rosdistro format:

```yaml
python3-pystac:  # upstream PR owed: https://github.com/ros/rosdistro/pull/NNNNN
  ubuntu: [python3-pystac]
  debian: [python3-pystac]
```

The comment naming the owed or open upstream PR is load-bearing bookkeeping —
`rosdep_local_staleness_check.sh` looks for it. A local key is always a
temporary stand-in for an upstream entry; the comment is what stops it
becoming a permanent private dependency database.

Workspace scripts never name a repo: they glob
`layers/main/*_ws/src/*/rosdep.yaml`.

### What declaring a key is trusted to do

Be deliberate about merging one. A `rosdep.yaml` merged in **any** project repo
is read by every consumer below, and each of them feeds it to a **root-level**
`rosdep install -y`:

- the **dev host**, via the generated `ROSDEP_SOURCE_PATH` — `make build`'s
  rosdep pass installs into the machine you work on;
- the **`ci_local` container**, where the verified environment an attestation
  vouches for is built;
- the **agent image bake**, where it lands in a layer every sandboxed agent
  then runs on;
- **hosted CI**, in the project repo's own GitHub Actions job.

So the review of a one-line `rosdep.yaml` is a review of what gets installed as
root on four environments' worth of machine, not of a build-config detail. The
shape rule below is what keeps that blast radius to "an apt package name from a
reviewed repo"; it is a policy gate, and the load-bearing part is still that
project repos are reviewed.

Three of those four are **persistent** — the dev host you keep working on, and
the container images built on it. Hosted CI is the exception: a throwaway
runner container that is destroyed with the job. That difference is why the
shape gate is wired where it is (see below).

### The accepted shape — a rule, and it is enforced

**A `rosdep.yaml` in this workspace may use exactly one form:**

```yaml
<rosdep-key>:            # upstream PR owed: ros/rosdistro#NNNNN
  <os-name>: [<package>, ...]
```

Every OS value is a **list of plain system package names**. Never a bare
string, and never a nested mapping.

That last clause is the point. rosdep's own format is wider than this: a nested
mapping under an OS name is how `pip`, `npm`, `gem` and `source` rules are
written — and a `source` rule downloads an rdmanifest and *runs its install
script*. A `pip:` rule would also route around ADR-0009's Python-package tiers
entirely. These files feed a root-level `rosdep install -y`, so the workspace
accepts the one documented shape and rejects the rest.

The codename-keyed form (`ubuntu: {noble: [pkg]}`) is a nested mapping too, and
is rejected along with the others. That is deliberate rather than an oversight:
a local key is a short-lived stand-in for an upstream entry, not a place to
express a per-release matrix.

[`rosdep_yaml_validate.sh`](../scripts/rosdep_yaml_validate.sh) is the gate, and
every path that can carry a key onto a **persistent** machine runs it **first**:

| Path | What a rejected file does |
|---|---|
| `rosdep_local_sources.sh` (dev host, `make build`) | Excluded from the generated source list; script exits 4, and the Makefile stamp fails the build. |
| `stage_rosdep_manifests.sh` (agent image) | Not staged; script exits 4, and the image build stops. |
| `ci_local.sh` | The run is refused before the container starts. |
| `rosdep_local_staleness_check.sh` (`make validate`) | Reported as a finding (exit 1), so the rejection is visible rather than surfacing later as a silently-missing key. |

All of them **fail closed** when the validator cannot run at all (no python3 or
no `yaml` module): unvalidated and invalid reach root identically.

This is a policy gate, not a security boundary — the real defence is that
project repos are reviewed. It stops the unreviewed shape from reaching root,
and names the rule when it does.

#### Hosted CI is the ungated fourth path — an accepted residual gap

The workflow step below (and the copy in
[`.agent/templates/ci_workflow.yml`](../templates/ci_workflow.yml)) writes the
repo's `rosdep.yaml` straight into a `ROSDEP_SOURCE_PATH` that a root-level
`rosdep install` then reads. It does **not** run the shape gate, and that is a
deliberate, recorded gap rather than an oversight:

- The step has to be **self-contained** — a project repo's workflow does not
  check this workspace out, so the only way to gate there would be to inline a
  second copy of the shape grammar into the template. Two copies of a rule
  drift, and the one that drifts is the one that is wrong.
- The gate exists to protect **persistent** machines. Hosted CI runs in a
  throwaway container that GitHub destroys with the job, so an out-of-shape
  rule there can install something odd into that runner and nothing else. It
  cannot reach the dev host, the `ci_local` container on it, or the agent image.
- The same file **is** gated on every path that does reach a persistent
  machine, including `ci_local.sh` — which on a project repo is the ADR-0018
  merge verification. So a file that would be rejected is caught before the
  branch can merge, just not by the hosted job itself.

If hosted CI ever gains a persistent cache or a self-hosted runner, this
reasoning expires and the gate has to reach the workflow step.

### The four consumers

| Where | How the local keys get in |
|---|---|
| Interactive shell / `make build` | `rosdep_local_sources.sh` writes `<main-root>/.rosdep/sources.list.d/` (gitignored). `setup.bash` exports `ROSDEP_SOURCE_PATH` at it when it exists; the `$(STAMP)/rosdep-local.done` Makefile stamp regenerates it whenever a `rosdep.yaml` changes, is added, or first appears with a newly checked-out repo. `bootstrap.sh` generates it between `rosdep init` and its `rosdep update`. Because one directory serves every shell and every worktree on the host, a regeneration never edits it in place: the new content is built in a sibling slot and published by an atomic symlink swap under a bounded `flock`, so `ROSDEP_SOURCE_PATH` always resolves to a complete directory. |
| `ci_local.sh` | Tests one repo in isolation with no `layers/` tree, so it overlays **that repo's own** `rosdep.yaml` into a container-local sources dir and records a `+rosdep-local` steps token in the attestation note. |
| Agent container image | `stage_rosdep_manifests.sh` stages each repo's `rosdep.yaml` into the build context; the Dockerfile builds `/opt/rosdep-sources` and sets `ENV ROSDEP_SOURCE_PATH`. A key that still will not resolve ends the build step in a labelled `WARNING` block naming it. At launch, `agent-entrypoint.sh` inherits `ROSDEP_SOURCE_PATH` from the bind-mounted workspace (mounted at its host absolute path, so the `file://` URLs resolve) and refreshes **both** rosdep caches — root's and the agent user's, since the cache is per user and keyed by source URL. It skips that refresh when the list it sees is byte-identical to the image's own baked one: a workspace that generated no local list leaves `ROSDEP_SOURCE_PATH` at the baked dir, whose cache is already correct. |
| Hosted CI (project repo) | One workflow step, below. The **only** consumer that does not run the shape gate first — an accepted residual gap, reasoned out above. |

### Cache gotcha

rosdep's cache is **per user** (`$HOME/.ros/rosdep/sources.cache`) and keyed by
**source URL**. Two consequences that have already bitten:

- A cache populated without the local sources has no entry for them, so
  `rosdep install` fails on a key it cannot see. After changing
  `ROSDEP_SOURCE_PATH`, run `rosdep update`.
- Running `rosdep update` *without* `ROSDEP_SOURCE_PATH` rewrites that same
  cache and drops the local keys from it. This is why
  `rosdep_local_staleness_check.sh` probes under a throwaway `HOME`.

## Hosted CI: the copy-pasteable step

Add this to a project repo's workflow **before** its existing `rosdep install`
step. It is self-contained: nothing depends on this workspace being checked
out.

```yaml
      - name: Install repo-local rosdep keys
        if: hashFiles('rosdep.yaml') != ''
        run: |
          set -euo pipefail
          sudo rosdep init 2>/dev/null || true
          SRC=/opt/rosdep-sources
          sudo mkdir -p "$SRC"
          # ROSDEP_SOURCE_PATH REPLACES sources.list.d — copy every system list,
          # not just 20-default.list, or a source the runner has is dropped.
          sudo cp /etc/ros/rosdep/sources.list.d/*.list "$SRC/"
          echo "yaml file://$GITHUB_WORKSPACE/rosdep.yaml" \
            | sudo tee "$SRC/30-workspace-local.list" > /dev/null
          echo "ROSDEP_SOURCE_PATH=$SRC" >> "$GITHUB_ENV"

      - name: rosdep update
        run: rosdep update   # must run AFTER ROSDEP_SOURCE_PATH is in the env
```

Exporting through `$GITHUB_ENV` means the workflow's existing
`rosdep install` step needs no change at all.

The `sudo` calls are for a workflow running directly on a runner. The
workspace's own CI template
([`.agent/templates/ci_workflow.yml`](../templates/ci_workflow.yml)) runs in a
`ros:jazzy-*` **container**, where the job is already root, and carries the
same step without `sudo` — already in place for every repo onboarded from the
template, and a no-op (`if: hashFiles('rosdep.yaml') != ''`) in the repos that
declare no local keys. `onboard-project` says to keep it.

## The bookkeeping: `make validate` nags

`rosdep_local_staleness_check.sh` runs from `make validate` and reports two
things:

1. **A local key that now resolves against the default sources** — the upstream
   entry landed; delete the local one (and the file, if it was its last key).
2. **A key with no upstream-PR marker in its comment** — advisory text
   heuristic (it looks for `PR` or a `rosdistro` URL). The rosdistro format has
   no structured field for this, so the check catches forgotten bookkeeping; it
   cannot verify that the PR exists or that it is still open.

Its exit codes are distinct on purpose: `0` clean (including "no `rosdep.yaml`
files found"), `1` a finding, `2` usage, `3` **SKIPPED**. SKIPPED means the
probe could not run its own isolated `rosdep update` — offline, or no rosdep —
and it reports no resolves/does-not-resolve verdict at all rather than one read
off a cache it did not build. `make validate` prints a SKIPPED and moves on; it
fails only on `1`.

**A flag is not a build break.** The intended lifecycle is: add the local key →
open the upstream `ros/rosdistro` PR when the consuming PR is ready to merge →
the check flags the key once upstream lands → delete the local entry. Seeing a
flag mid-transition is the mechanism working.

## Rebuilding the agent image

Per [#604](https://github.com/rolker/ros2_agent_workspace/issues/604), the
launcher only builds the agent image when it is *missing*, and the startup
scripts are baked from the MAIN checkout. After a change to the bake, rebuild
explicitly:

```bash
make agent-build
```

## See also

- [`.agent/scripts/rosdep_yaml_validate.sh`](../scripts/rosdep_yaml_validate.sh) — the shape gate
- [`.agent/scripts/rosdep_local_sources.sh`](../scripts/rosdep_local_sources.sh) — aggregation
- [`.agent/scripts/rosdep_local_staleness_check.sh`](../scripts/rosdep_local_staleness_check.sh) — enforcement
- [`.agent/scripts/stage_rosdep_manifests.sh`](../scripts/stage_rosdep_manifests.sh) — agent-image staging
- [ADR-0009](../../docs/decisions/0009-python-package-management-policy.md) — Python package management tiers
- [ADR-0018](../../docs/decisions/0018-local-first-ci-verification.md) — local-first CI verification
