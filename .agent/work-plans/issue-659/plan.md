# Plan: rosdep_local_sources.sh: discover rosdep.yaml in layer worktrees, not only layers/main

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/659

## Context

`rosdep_local_sources.sh` (#654) aggregates every project repo's root
`rosdep.yaml` into one host-shared, symlink-published sources directory
(`$MAIN_ROOT/.rosdep/sources.list.d`), which `setup.bash` exports as
`ROSDEP_SOURCE_PATH` for **every shell on the host** — main tree and every
worktree alike. Today the generator only globs
`layers/main/*_ws/src/*/rosdep.yaml`. A `rosdep.yaml` added on a feature
branch lives under `layers/worktrees/<name>/<layer>_ws/src/<repo>/rosdep.yaml`
until the PR merges, so it is invisible to `rosdep install` on the very host
that needs it — confirmed with `unh_marine_autonomy#397`'s
`layers/worktrees/issue-unh_marine_autonomy-397/core_ws/src/unh_marine_autonomy/rosdep.yaml`.
The Makefile's `$(STAMP)/rosdep-local.done` stamp has the identical
`layers/main`-only gap in `ROSDEP_LOCAL_YAMLS`.

The owner's checkpoint decisions (issue comment, 2026-09-23) that this plan
implements:

1. **Host-wide union**, not per-worktree sources dirs: every layer worktree's
   `rosdep.yaml` joins the one shared directory, same as `layers/main`.
2. **Conflict detection**: a key declared with *different* package lists in
   two places (worktree vs. worktree, or worktree vs. `layers/main`) is
   reported as a conflict with its **own distinct exit code** — no silent
   last-wins either direction. An *identical* declaration in two places
   dedupes cleanly (not a conflict).
3. **AGENTS.md** Script Reference row update for `rosdep_local_sources.sh` is
   approved (Ask-First item, already cleared).

The review also asked the PR to state explicitly why `stage_rosdep_manifests.sh`
and `ci_local.sh` are out of scope: both already operate on one repo resolved
to a specific checkout (a project's own root `rosdep.yaml`), not a host-wide
glob over `layers/main` — there is no "which worktree's copy" ambiguity for
either of them to begin with.

## Approach

1. **Extend discovery in `rosdep_local_sources.sh`.** Add a second glob,
   `layers/worktrees/*/*_ws/src/*/rosdep.yaml`, alongside the existing
   `layers/main/*_ws/src/*/rosdep.yaml`, combined before the existing sort.
   The shape gate (`rosdep_yaml_validate.sh`) runs per file exactly as today,
   regardless of which glob it came from.

2. **Add a conflict-detection pass, after the shape gate, before writing
   `30-workspace-local.list`.** For every file that passed the shape gate,
   parse its top-level rosdep keys (a small `python3`/`yaml.safe_load` pass,
   the same tool the staleness check already uses for this) into
   `key -> canonicalized-OS-package-map` (dict, each OS's package list
   sorted, so ordering differences never manufacture a false conflict).
   Build a single `key -> {canonical_value: [files...]}` index across *all*
   files from *both* globs:
   - One distinct `canonical_value` for a key across all files it appears
     in → identical declarations, no conflict; every file that declared it
     is still eligible for inclusion (dedup is about not *flagging* it, not
     about physically merging the two `rosdep.yaml` files).
   - More than one distinct `canonical_value` for the same key → conflict.
     Report every key, every conflicting file, and every distinct value
     (mirroring the shape-gate's per-file error reporting style). Every file
     touched by any conflicting key is **excluded** from
     `30-workspace-local.list` for this run (same exclusion mechanics the
     shape gate already uses for a rejected file — a partial, safe directory
     is still published, just without those files' keys). A file with *no*
     conflicting key is unaffected even if a sibling key it declares is
     involved in a conflict elsewhere.

3. **New exit code 6 for "one or more keys conflict."** Follows the existing
   precedent (2 usage, 3 uninitialized, 4 shape-rejected, 5 lock timeout — all
   distinct so callers can tell them apart). Precedence when both a shape
   rejection *and* a conflict occur in the same run: the directory is written
   either way (partial-but-safe, matching today's exit-4 behavior); the
   **exit status is the shape rejection (4)** if any file was shape-rejected,
   else conflict (6) if any key conflicted, else 0 — shape validity is a
   stricter, pre-existing gate and keeping its exit code load-bearing avoids
   changing established `make build` failure semantics for the already-shipped
   case. Update the script's header comment (the same block documenting exit
   codes 0/2/3/4/5 today) to add 6 and to describe the two-glob discovery.

4. **Makefile: extend `ROSDEP_LOCAL_YAMLS`.**
   ```make
   ROSDEP_LOCAL_YAMLS := $(wildcard $(MAIN_ROOT)/layers/main/*_ws/src/*/rosdep.yaml) \
                          $(wildcard $(MAIN_ROOT)/layers/worktrees/*/*_ws/src/*/rosdep.yaml)
   ```
   This variable already feeds both the stamp's *addition* detection (a
   direct prerequisite — new files make the stamp stale) and, via the
   `FORCE`-driven `$(STAMP)/rosdep-local.list` content-diff file, the
   *deletion/rename* detection (a shrinking prerequisite list alone can't
   stale a stamp, so the recorded path set is diffed on every invocation).
   Extending the variable is sufficient for a worktree `rosdep.yaml` to
   participate in both — no separate mechanism needed. Verify post-change
   with `make -n $(STAMP)/rosdep-local.done` against a fixture worktree
   `rosdep.yaml` (documented as a manual check in Post-Task Verification,
   since driving the real top-level Makefile from a hermetic hermetic test
   fixture would require faking `MAIN_ROOT` and the whole stamp chain).

5. **`rosdep_local_staleness_check.sh`: extend its glob too — decided yes.**
   Its job is to audit *exactly* the key set `rosdep_local_sources.sh` feeds
   into the host-shared `ROSDEP_SOURCE_PATH`; today both scripts glob
   `layers/main` only, so they're in sync by accident. Leaving the staleness
   check on `layers/main` only after widening the generator would open a new
   gap in the other direction: a worktree-declared key would silently feed
   every `rosdep install` on the host while evading the "is this key still
   needed / does it carry an upstream-PR marker" bookkeeping check entirely
   — worse than today's gap, which at least fails closed (the key doesn't
   resolve anywhere). This is the same trade-off `make validate` already
   accepts for `layers/main` local keys today: any local key anywhere in the
   workspace can already fail `make validate` in a worktree that never
   touched that key, because the check audits the whole host-shared key set,
   not just the current repo. Add the same second glob,
   `layers/worktrees/*/*_ws/src/*/rosdep.yaml`, to its `yamls=(...)` array.
   Conflict detection itself stays out of scope for this script — that's
   `rosdep_local_sources.sh`'s job at generation time; the staleness check
   just reports on whatever keys it finds, including possibly-conflicting
   ones (which will also surface as a `rosdep_local_sources.sh` finding via
   `make validate`'s separate `rosdep-local.done` path, or on the next
   `make build`).

6. **Tests** — extend `.agent/scripts/tests/test_rosdep_local_sources.sh`
   (not a new file):
   - Worktree-only `rosdep.yaml` (no `layers/main` counterpart) is picked up:
     appears in `30-workspace-local.list` after regeneration.
   - Conflicting pair — same key, different package lists, one declared
     under `layers/main`, one under `layers/worktrees/*` — is reported
     (message names the key and both files), excluded from
     `30-workspace-local.list`, and the run exits **6**.
   - A second conflicting pair, both sides under `layers/worktrees/*`
     (worktree vs. worktree, no `layers/main` copy at all), to cover the
     issue's other named case.
   - Identical declaration of the same key in `layers/main` and a worktree
     dedupes cleanly: exits 0, no conflict reported, both files' lines are
     still written (or at minimum the key is resolvable — assert no
     "conflict" text and rc 0, not that both specific lines survive, since
     the dedup contract is "no false conflict," not "both copies present").
   - Shape-rejection (exit 4) still wins over a simultaneous conflict (exit
     6) in the same run, per the precedence in step 3.
   - A lightweight Makefile regression guard: grep the committed `Makefile`
     for the `layers/worktrees/*/*_ws/src/*/rosdep.yaml` wildcard pattern in
     `ROSDEP_LOCAL_YAMLS`, so a future edit that drops the worktree glob is
     caught without invoking `make` itself.
   - `rosdep_local_staleness_check.sh`: a worktree-only `rosdep.yaml` with no
     upstream-PR marker is reported (same assertions as the existing
     `layers/main` unmarked-key case, pointed at a worktree-glob fixture).

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/rosdep_local_sources.sh` | Second discovery glob (`layers/worktrees/*/*_ws/src/*/rosdep.yaml`); key-conflict detection pass; new exit code 6; header-comment update (discovery + exit codes) |
| `.agent/scripts/rosdep_local_staleness_check.sh` | Second discovery glob, same pattern; header-comment note |
| `Makefile` | `ROSDEP_LOCAL_YAMLS` gains the worktree wildcard, feeding both the direct prerequisite and the `FORCE`-diffed `rosdep-local.list` deletion/rename detection |
| `.agent/scripts/tests/test_rosdep_local_sources.sh` | New cases: worktree-only discovery, main-vs-worktree conflict, worktree-vs-worktree conflict, identical-declaration dedupe, shape-vs-conflict precedence, Makefile wildcard grep guard, staleness-check worktree glob |
| `AGENTS.md` | `rosdep_local_sources.sh` Script Reference row: document worktree discovery and exit code 6; note the same for `rosdep_local_staleness_check.sh`'s row if its own discovery/exit-code documentation needs it |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Workspace vs. project separation (ADR-0003) | Both globs stay generic (path-shaped, no repo names); worktree naming (`layers/worktrees/<name>/<layer>_ws/src/<repo>/`) is itself workspace-generic infra, not project-specific |
| A change includes its consequences | `AGENTS.md` row updated in this PR (approved Ask-First item); `rosdep_local_staleness_check.sh` widened in the same PR rather than left mismatched with the generator it audits |
| Enforcement over documentation | The new conflict rule is enforced (non-zero exit, files excluded from the generated list), not just noted in a comment |
| Test what breaks | Tests target the two concrete cases named in the issue plus the owner's added dedupe and precedence cases — not blanket coverage-chasing |
| Only what's needed | `stage_rosdep_manifests.sh` and `ci_local.sh` are explicitly left untouched (see Context) — each already resolves one specific repo checkout, so there's no "which worktree" ambiguity for the fix to close |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0007 — Retain Make with dependency tracking | Yes | `ROSDEP_LOCAL_YAMLS` keeps using the existing wildcard + `FORCE`-diffed-content-file idiom for add/rename/delete staleness; the worktree glob is added to the same variable, not a parallel mechanism |
| 0003 — Project-agnostic workspace infra | Yes | Both new globs are path-pattern generic, consistent with the existing `layers/main` glob |
| 0004/0005 — Enforcement hierarchy / layered enforcement | Yes | The shape gate (`rosdep_yaml_validate.sh`) still applies uniformly regardless of glob; the new conflict check is itself a new enforcement layer at generation time, not merely documented policy |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `rosdep_local_sources.sh` discovery + exit codes | `AGENTS.md` Script Reference row | Yes |
| `rosdep_local_sources.sh` discovery | `rosdep_local_staleness_check.sh` discovery (same key set it must audit) | Yes |
| `Makefile`'s `ROSDEP_LOCAL_YAMLS` | Nothing further — the `FORCE`/`rosdep-local.list` diff mechanism already generalizes over whatever paths the variable lists | Yes (no extra work needed beyond the variable itself) |
| New exit code 6 | `.agent/scripts/rosdep_local_sources.sh`'s own header-comment exit-code table | Yes |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): `AGENTS.md`'s Script Reference row
  for `rosdep_local_sources.sh` currently documents `layers/main`-only
  discovery and exit codes 0/2/3/4/5 — both go stale once this lands and are
  updated in the same PR (owner-approved Ask-First item). If
  `rosdep_local_staleness_check.sh`'s row is later found to also describe its
  discovery glob explicitly, update it too for the same reason.
- **Agent-instruction candidates** (proposals only): none identified beyond
  the required `AGENTS.md` row update above — this is a narrow generator-glob
  fix, not a new workflow pattern.

## Open Questions

None — the owner's checkpoint comment resolved reach, conflict semantics, and
the `AGENTS.md` approval. The one item this plan itself decided (rather than
being told) is **staleness-check scope** (step 5 above: extend its glob too)
and **conflict/shape-rejection exit-code precedence** (step 3: shape (4) wins
over conflict (6) when both occur) — both are implementation-level calls
consistent with the owner's stated reasoning, not new policy questions, but
flagged here so review-plan can confirm the precedence choice explicitly if
it disagrees.

## Estimated Scope

Single PR (local-first; no PR opened by this planning step per the issue's
"Do NOT open a PR" instruction — `review-plan`/`/run-issue` publish later).
