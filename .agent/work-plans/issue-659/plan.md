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

## Revision (r2) — plan-checkpoint decisions (owner, 2026-09-23 12:37 -04:00)

After Plan Review r1 (needs-work, 3 findings), the owner decided all three
fold into this plan directly, implemented without a second plan-review round
(the pre-push code review is the next independent read):

1. **Leftover worktree directories.** A worktree's `rosdep.yaml` is included
   only when git still registers that worktree — a directory whose
   registration is gone (e.g. after a `merge_pr.sh` exit-3 worktree-removal
   failure, or a hand-interrupted removal) drops out automatically, with a
   named reason on stderr. **Mechanism chosen**: run
   `git -C <pkg_dir> worktree list --porcelain` **from the discovered
   directory itself** and check whether `<pkg_dir>`'s realpath appears in its
   `worktree <path>` lines. This is preferred over comparing
   `rev-parse --git-common-dir` against a separately-obtained
   `worktree list`, or reading the `.git` gitdir pointer and checking the
   admin dir exists, because `git worktree list` already enumerates every
   worktree of the repository (main and linked alike) regardless of which one
   it is invoked from — so validity ("is this a git checkout at all") and
   registration ("is it still known to git") collapse into one command: a
   worktree whose admin directory was removed can't resolve its own `.git`
   pointer, so the command itself fails rather than merely omitting the path
   from its output. One command, fails closed either way. A file whose
   checkout cannot be checked (git failure of any kind) is excluded, never
   included by default. Test required (both the registered and the
   deregistered-but-present-on-disk case).
2. **`bootstrap.sh` gets an exit-6 branch.** `bootstrap.sh` (lines ~169-179)
   has its own switch on the generator's exit code that the original plan
   missed. Add `elif [ "$BOOTSTRAP_GEN_RC" -eq 6 ]`, mirroring the existing
   `-eq 4` branch: the directory is still written and valid (conflicting
   files are excluded, not the whole tree), so export `ROSDEP_SOURCE_PATH`
   and print a note naming the conflict, rather than falling into the
   generic `else` (which today discards the entire generated directory,
   including every non-conflicting key). Grepped the repo for every other
   caller of `rosdep_local_sources.sh` (`grep -rn rosdep_local_sources.sh`):
   the Makefile's `$(STAMP)/rosdep-local.done` recipe already treats any
   non-zero, non-3 exit as fatal (`exit $$rc`), so 6 fails the build exactly
   like 4 does — no gap there. No other caller exists.
3. **Discovery skips symlinked `*_ws` and symlinked package entries** inside
   `layers/worktrees/*`, following the `wt_layer_*` convention already in
   `.agent/scripts/_worktree_helpers.sh` (`[ -L "$ws_dir" ] && continue` /
   `[ -L "$pkg_dir" ] && continue`): `worktree_create.sh` symlinks every
   sibling package not in `--packages` and every non-target layer straight
   back to `layers/main`, and bash's `*` glob traverses those symlinks
   transparently — without the skip, the generated list and the
   conflict-detection pass would scale with (repos × active worktrees)
   instead of (repos), re-discovering `layers/main` content a second time
   under a different path for every active worktree that happens to include
   that layer/package. Test required (a worktree with an untouched symlinked
   sibling package, and one with a whole symlinked non-target layer).

**Makefile wildcard vs. symlink skip (reconciled)**: `$(wildcard ...)` in the
Makefile follows symlinks the same way the shell glob does, so
`ROSDEP_LOCAL_YAMLS` picking up a symlinked `layers/main` file a second time
under `layers/worktrees` is unavoidable at the Makefile-prerequisite level (make
has no notion of "skip this symlinked path" in `$(wildcard)`). This is
harmless here: extra prerequisite entries only make
`$(STAMP)/rosdep-local.list`'s content-diff check re-run slightly more often
(a symlinked file's mtime moving re-triggers a cmp that then finds the
recorded path SET unchanged, since the generator's own dedup already
collapses it to one path) — never a wrong result, only a cheap extra check.
The generator script itself remains the authoritative discovery + symlink
skip + registration filter; the Makefile variable exists only to keep
`make build` re-checking at the right times, per ADR-0007.

**Consequences table addition**: `bootstrap.sh`'s exit-code switch is a
caller of `rosdep_local_sources.sh` just like the Makefile stamp, and both
must handle every exit code the generator can return — captured as its own
Files-to-Change row and Consequences row below (Finding 1 of Plan Review r1).
Stale/abandoned worktree persistence (Finding 3) is now addressed directly by
decision 1 above (unregistered ⇒ excluded), so it is no longer an open
consequence to merely document.

## Revision (r3) — pre-push review must-fix (address-findings, 2026-09-23)

The pre-push `review-code` pass (Local Review round 1) found decision 1's
"Mechanism chosen" above incomplete: `git -C <pkg_dir> worktree list
--porcelain`, run from the discovered directory itself, proves only that
`<pkg_dir>` is *some* live git repo whose own registry lists itself —
trivially true for ANY standalone git repo, worktree or not, not just a
genuine linked worktree of the corresponding `layers/main` repo. A
rogue/coincidental standalone repo placed at the worktree path shape would
have passed this check and had its `rosdep.yaml` silently merged into the
host-shared `ROSDEP_SOURCE_PATH`.

`wt_is_registered` now takes the corresponding `layers/main/<ws>/src/<pkg>`
repo as a second argument (computed by `wt_discover_local_rosdep_yamls` from
the worktree path's own layer/package basenames) and requires BOTH: (a) the
discovered checkout shares that repo's `git rev-parse --path-format=absolute
--git-common-dir` (the identity anchor the r2 mechanism note argued was
unnecessary — it turned out to be exactly the missing check), and (b) that
repo's OWN `worktree list --porcelain`, queried from the main repo rather
than from the discovered directory, still names it. A worktree path whose
corresponding `layers/main` repo does not exist at all (a stale path
outliving a rename/removal of the main repo — the case the #659 checkpoint
asked to be decided explicitly) is also excluded, fail-closed, with its own
named reason distinct from "not registered".

New tests: a rogue standalone repo at the worktree path shape (excluded,
even though its own `worktree list` trivially lists itself) and a worktree
with no corresponding `layers/main` repo (excluded, "no corresponding ...
repo exists"). Full suite: 154 passed, 0 failed.

## Revision (r4) — pre-push round 2 decision (owner, 2026-09-23 13:23 -04:00)

Round 2's Local Review found a second, distinct must-fix: a worktree
`rosdep.yaml` that passes the shape gate can still be DELETED before this
generator finishes with it — a worktree teardown racing a concurrent
regeneration, not a theoretical race, now that worktree churn feeds this
generator. Two gaps: (a) the conflict-detection Python subprocess's bare
`except Exception: continue` on a later read failure silently treated a
vanished file as "declares no keys" instead of excluding it and reporting
it — which would let a DIFFERENT file's declaration of the same key win
uncontested, undermining the conflict check itself; (b) the final publish
loop wrote a `yaml file://<path>` line for every shape-gate-passed,
non-conflicting file with no re-check that the file still existed, so a
file deleted between conflict-check and publish landed a dangling source
line in the host-shared `ROSDEP_SOURCE_PATH` that every `rosdep install -y`
on the host consults until the next regeneration. A suggestion was also
folded in: canonicalize package lists with `sorted(set(pkgs))` so an
in-file duplicate (`[foo, foo]`) doesn't manufacture a false conflict
against a clean `[foo]` declared elsewhere.

Owner decision (binding, posted as an issue comment): **race + regenerate
on removal**:
1. Fix the race — a file unreadable at conflict-check or publish time is
   excluded (named on stderr), never defaulted in as "declares no keys".
2. `worktree_remove.sh` regenerates the workspace rosdep sources
   (`rosdep_local_sources.sh <MAIN root>`) after removing a layer worktree
   that carried a `rosdep.yaml`, checked BEFORE removal (once the directory
   is gone there is nothing left to check) — so a removal never leaves a
   dangling line for longer than it takes the next regeneration to run.
3. Dedupe package lists before conflict comparison.

### Implementation

**Race fix** (`rosdep_local_sources.sh`): the conflict-detection Python
subprocess now reports `UNREADABLE\t<path>` instead of silently skipping a
file it cannot read; the bash side excludes it and echoes a named error.
The final publish loop re-checks `[ -f "$yaml" ]` immediately before writing
each line and excludes (with a named error) anything that vanished since
the conflict check. A new exit code, **7**, is returned when one or more
files were excluded this way — distinct from 4 (shape rejection) and 6
(conflict), because a vanish is neither: it is not a policy violation
(unlike 4) and it does not indicate two files genuinely disagree about a
key (unlike 6) — it is a transient, single-file race whose fix is "re-run,
the file may still exist." Exit-code precedence: 4 wins over 6 wins over 7
— each is a stricter, earlier-established gate, so `make build` /
`bootstrap.sh` keep today's failure semantics for the already-shipped
cases (4, 6). Two test-only env hooks
(`ROSDEP_LOCAL_SOURCES_TEST_VANISH`, `...TEST_VANISH_AFTER_CONFLICT`) let
the regression suite delete a named file at each of the two checkpoints
deterministically, rather than racing a real background `rm` against the
script's own execution speed. `bootstrap.sh` gained an `elif -eq 7` branch
mirroring the existing exit-4/exit-6 branches (export `ROSDEP_SOURCE_PATH`
anyway — the directory is still valid, minus the vanished file — with a
note); the Makefile stamp needed no change (any non-zero, non-3 exit
already fails the build). Package-list canonicalization now dedupes
(`sorted(set(pkgs))`) before sorting.

**Regenerate on removal** (`worktree_remove.sh`): Phase 1 (which already
walks every non-symlinked `*_ws/src/*` package directory to check for
uncommitted changes) now also checks `[ -f "$pkg_dir/rosdep.yaml" ]` and
sets a flag if any package carries one — checked BEFORE the destructive
Phase 3 cleanup, since the files are gone afterward. After the worktree is
successfully removed, if the flag is set (and `WORKTREE_TYPE` is `layer` —
only layer worktrees can carry a project repo's rosdep.yaml; workspace
worktrees hold `.agent/`/docs infrastructure, not package checkouts), the
script calls `rosdep_local_sources.sh <ROOT_DIR>` (the same main-root
anchoring `setup.bash` uses). Deliberately does **not** run `rosdep
update` — that hits the network and is slow; the cache refreshes on its
own schedule (bootstrap.sh, `make build`, or by hand), and dropping the
dangling `yaml file://` line from the SOURCES directory is what actually
matters. A regeneration failure is reported loudly (`⚠️  Warning:` plus the
exact re-run command) but does not turn the exit status non-zero — the
worktree removal itself already succeeded by that point, and failing the
whole script would make a caller (a human, or `merge_pr.sh`) think the
REMOVAL failed when only the follow-up bookkeeping did. Checked
`merge_pr.sh`: it removes worktrees by calling `worktree_remove.sh`
directly (`"$SCRIPT_DIR/worktree_remove.sh" --issue "$ISSUE_NUM"
--repo-slug "$remove_slug"`), so the merge path is covered by this same
fix with no separate change needed there.

### Tests

`test_rosdep_local_sources.sh`: new cases for the in-file-duplicate dedupe,
a conflict-check-time vanish (exit 7, excluded, other keys still resolve),
a publish-time vanish (exit 7), exit-code precedence (4 over 7, 6 over 7,
each with a genuine simultaneous trigger — a file already excluded by a
conflict never reaches the publish-time existence check, so precedence
tests use a third, unrelated file to make both conditions fire in one run),
and that `bootstrap.sh` has an explicit exit-7 branch that still exports
`ROSDEP_SOURCE_PATH`. Full suite: **171 passed, 0 failed** (was 154; +17
across the new sections — the #659 round-2 additions plus the pre-existing
Makefile/entrypoint sections already counted here).

`test_worktree_remove.sh`: new fixture helper (`setup_with_layer_worktree`)
building a genuine linked layer worktree of a `layers/main/<ws>/src/<pkg>`
repo (matching what `worktree_create.sh` actually produces, same pattern as
`test_rosdep_local_sources.sh`'s `add_layer_worktree`), and three new
cases: removal with a rosdep.yaml regenerates and drops the removed
worktree's line while keeping `layers/main`'s own; removal without one
skips regeneration entirely (no `.rosdep` dir created); and a stubbed
regeneration failure is reported loudly with the exact re-run command
while the removal itself still reports success. Full suite: **5 passed, 0
failed** (was 2; +3 new).

`test_worktree_create.sh`: unaffected by this round's changes (no target
file touched) — re-run to confirm no regression. **33 passed, 0 failed**
(unchanged from r3).

## Revision (r5) — pre-push round 3 decision (owner, 2026-09-23 13:52 -04:00)

Round 3's Local Review (Pre-Push) found two new must-fix issues, both
introduced by round 2's own TOCTOU-handling code and independently
confirmed by two disjoint-lens adversarial passes:

1. `worktree_remove.sh:442-443` — `if ! "$SCRIPT_DIR/rosdep_local_sources.sh"
   "$ROOT_DIR"; then REGEN_RC=$?` captured `$?` from the negated `!` test,
   not the wrapped command's real exit status (`$?` inside that `then`
   branch is always 0). The operator-facing warning ("exit $REGEN_RC — see
   messages above") therefore always claimed "exit 0" regardless of the
   generator's actual failure code.
2. `rosdep_local_sources.sh:277-279,397-399` — the two test-only hooks
   (`ROSDEP_LOCAL_SOURCES_TEST_VANISH`, `..._TEST_VANISH_AFTER_CONFLICT`)
   ran `rm -f "$path"` unconditionally whenever the env var was non-empty,
   with no confinement to a test/tmp root and no check that the path was
   one this run itself discovered — an unconfined delete capability shipped
   in the production script every real invocation runs (dev shells, `make
   build`, `bootstrap.sh`, `worktree_remove.sh`, CI, the agent-image bake).

Owner decision (binding, posted as an issue comment, "Pre-push round 3
decision"):
1. **Test hooks never delete.** `ROSDEP_LOCAL_SOURCES_TEST_VANISH[_AFTER_CONFLICT]`
   only make the script *treat* a named file as missing, and only when that
   file is one of the run's own discovered files. No `rm` in shipped code.
2. **Soften exit 7 in the Makefile.** Treat it like exit 3: print a note
   and leave the stamp stale so the next `make build` retries. A transient
   race no longer fails the build.
3. One-line fix: `worktree_remove.sh` captures the regen's real exit code
   (`cmd || rc=$?`), not the `!` test's, with a test that checks the
   number.

### Implementation

**Test hooks (simulation, never deletion)** (`rosdep_local_sources.sh`):
both hooks now compute whether their named path is one of `passed_yamls`
(this run's own shape-gate-passed discovered files) — if not, the hook is a
no-op with a stderr note ("is not one of this run's own discovered
files"), and the named path is never touched. If it is, the hook records
the path in a `simulate_vanish_*` associative array rather than deleting
anything: `ROSDEP_LOCAL_SOURCES_TEST_VANISH`'s target is passed to the
conflict-detection Python subprocess via a
`ROSDEP_LOCAL_SOURCES_SIMULATE_VANISH` env var (newline-separated paths),
and the subprocess treats any path in that set as unreadable without
opening it — so a real read failure and a simulated one report identically
as `UNREADABLE`. `ROSDEP_LOCAL_SOURCES_TEST_VANISH_AFTER_CONFLICT`'s target
is checked directly in the bash publish loop's existing `[ -f "$yaml" ]`
re-check (`[ -n "${simulate_vanish_publish[$yaml]+x}" ] || [ ! -f "$yaml" ]`)
— same observable behavior (exit 7, the file excluded, named on stderr),
with the underlying filesystem entry untouched throughout.

**Makefile: soften exit 7** — the `rosdep-local.done` stamp recipe gained
an `elif "$$rc" -eq 7` branch, ordered before the general `-ne 0` fail
branch and mirroring the existing exit-3 handling: prints a note naming the
cause (a worktree removal racing the build) and leaves the stamp untouched
so the next `make build` retries. Exits 4 and 6 are unaffected — they still
fail the recipe, per the owner's decision to keep them as policy-violation
gates.

**`worktree_remove.sh` exit-code capture** — `REGEN_RC=0; "$SCRIPT_DIR/rosdep_local_sources.sh"
"$ROOT_DIR" || REGEN_RC=$?` replaces the `if !`/`$?` pattern. `set -e` is
active in this script, so the `||` form (the same pattern `bootstrap.sh`
already used correctly) is required to both capture the real code and
avoid the command's own `set -e` propagation.

### Tests

`test_rosdep_local_sources.sh`: the two existing vanish sections (round-2)
gained a "the hook never deletes — the fixture file still exists" assertion
each (four call sites total, including both precedence tests). A new
section ("the test-vanish hooks only simulate a run's own discovered
files") exercises the ignored-path case: `ROSDEP_LOCAL_SOURCES_TEST_VANISH`
pointed at a file outside this run's discovery set exits 0, prints the
ignore note, resolves the real file normally, and leaves the out-of-scope
path untouched. The Makefile section gained exit-7 and exit-6 cases
alongside the existing exit-3/exit-4 ones (soft-note-and-retry for 7, same
as 3; hard-fail for 6, same as 4). Full suite: **185 passed, 0 failed**
(was 171; +14 across the new race-hook-simulation and Makefile-exit-7
cases).

`test_worktree_remove.sh`: `test_layer_removal_regen_failure_does_not_fail_removal`
gained an assertion that the warning names the stub's actual exit code
(`exit 5 — see messages above`), which the pre-fix `$?`-after-`!` bug would
have failed (it always printed "exit 0"). Full suite: **5 passed, 0
failed** (unchanged count — same 3 round-2 cases, one strengthened).

`test_worktree_create.sh`: unaffected by this round's changes — re-run to
confirm no regression. **33 passed, 0 failed** (unchanged).

## Approach

1. **Extend discovery in `rosdep_local_sources.sh`.** Add a second glob,
   `layers/worktrees/*/*_ws/src/*/rosdep.yaml`, alongside the existing
   `layers/main/*_ws/src/*/rosdep.yaml`. Walk the worktree side directory by
   directory (not a bare glob) so each `*_ws` and each `src/*` package entry
   can be skipped when it is a symlink (revision r2, decision 3), and so each
   surviving package directory can be checked for live git-worktree
   registration before its `rosdep.yaml` is admitted (revision r2, decision
   1) — a file whose checkout fails that check is dropped with a named
   reason on stderr, not silently included. Both globs' surviving files are
   combined and sorted before the existing shape gate. The shape gate
   (`rosdep_yaml_validate.sh`) runs per file exactly as today, regardless of
   which glob or check it came from.

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
   (not a new file). Worktree fixtures use a real throwaway git repo plus
   `git worktree add` so the registration check (decision 1) has something
   genuine to inspect — a plain directory can't exercise it:
   - Worktree-only `rosdep.yaml` (no `layers/main` counterpart), in a
     directory that IS a registered `git worktree add` checkout, is picked
     up: appears in `30-workspace-local.list` after regeneration.
   - A worktree directory that is present on disk with its `rosdep.yaml`
     still there, but whose git registration has been removed (the fixture
     deletes the admin dir named by the worktree's own `.git` gitdir
     pointer, leaving the working directory untouched — the same shape a
     `merge_pr.sh` exit-3 removal failure leaves): excluded from
     `30-workspace-local.list`, run still exits 0, stderr names the file and
     says it is not git-registered.
   - A worktree with an untouched symlinked sibling **package** (not in the
     worktree's own `--packages`) is not rediscovered — the generated list's
     yaml-line count does not grow from the symlinked entry alone.
   - A worktree with a whole symlinked **non-target layer** (`*_ws` itself a
     symlink back to `layers/main`) is not rediscovered either — same count
     assertion.
   - Conflicting pair — same key, different package lists, one declared
     under `layers/main`, one under a registered `layers/worktrees/*`
     checkout — is reported (message names the key and both files), excluded
     from `30-workspace-local.list`, and the run exits **6**.
   - A second conflicting pair, both sides under registered
     `layers/worktrees/*` checkouts (worktree vs. worktree, no `layers/main`
     copy at all), to cover the issue's other named case.
   - Identical declaration of the same key in `layers/main` and a registered
     worktree dedupes cleanly: exits 0, no conflict reported (assert no
     "conflict" text and rc 0, not that both specific lines survive, since
     the dedup contract is "no false conflict," not "both copies present").
   - Shape-rejection (exit 4) still wins over a simultaneous conflict (exit
     6) in the same run, per the precedence in step 3.
   - A lightweight Makefile regression guard: grep the committed `Makefile`
     for the `layers/worktrees/*/*_ws/src/*/rosdep.yaml` wildcard pattern in
     `ROSDEP_LOCAL_YAMLS`, so a future edit that drops the worktree glob is
     caught without invoking `make` itself.
   - `rosdep_local_staleness_check.sh`: a worktree-only `rosdep.yaml` (in a
     registered worktree checkout) with no upstream-PR marker is reported
     (same assertions as the existing `layers/main` unmarked-key case,
     pointed at a worktree-glob fixture).
   - `bootstrap.sh`'s new exit-6 branch is a code-review-level check (read
     the diff), not a new test harness: no test file for `bootstrap.sh`
     exists today (it drives real package installs), and building one from
     scratch to cover one `elif` is disproportionate to this issue's scope —
     noted explicitly rather than silently skipped.

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/_worktree_helpers.sh` | New shared helper: `wt_is_registered <dir> <main_repo_dir>` (git-worktree registration check against the SPECIFIC corresponding `layers/main` repo — git-common-dir identity + that repo's own worktree registry; revision r2 decision 1, revised r3 per the pre-push must-fix) and `wt_discover_local_rosdep_yamls <root_dir>` (the two-glob, symlink-skipping, registration-filtering discovery walk, shared verbatim by the generator and the staleness check so their key sets can never drift apart; computes each worktree's corresponding `layers/main` path and fails closed with a named reason when it doesn't exist) |
| `.agent/scripts/rosdep_local_sources.sh` | Source `_worktree_helpers.sh`; use `wt_discover_local_rosdep_yamls` for discovery; key-conflict detection pass; new exit code 6; header-comment update (discovery + exit codes). **r4**: conflict-detection subprocess reports (rather than silently skips) a read failure; publish loop re-checks existence immediately before writing; canonicalization dedupes package lists; new exit code 7; two test-only env hooks (`ROSDEP_LOCAL_SOURCES_TEST_VANISH`, `...TEST_VANISH_AFTER_CONFLICT`) |
| `.agent/scripts/rosdep_local_staleness_check.sh` | Source `_worktree_helpers.sh`; use `wt_discover_local_rosdep_yamls` for discovery; header-comment note |
| `.agent/scripts/bootstrap.sh` | New `elif [ "$BOOTSTRAP_GEN_RC" -eq 6 ]` branch mirroring the existing exit-4 branch (revision r2 decision 2). **r4**: new `elif -eq 7` branch, same pattern |
| `.agent/scripts/worktree_remove.sh` | **r4**: for a layer worktree, checks each non-symlinked package for a root `rosdep.yaml` before removal; after a successful removal, regenerates the workspace-local rosdep sources dir if any was found (never runs `rosdep update`); a regeneration failure is reported loudly but does not fail the (already-successful) removal |
| `Makefile` | `ROSDEP_LOCAL_YAMLS` gains the worktree wildcard, feeding both the direct prerequisite and the `FORCE`-diffed `rosdep-local.list` deletion/rename detection; comment reconciling symlink-following with decision 3 |
| `.agent/scripts/tests/test_rosdep_local_sources.sh` | New cases: worktree-only discovery (registered checkout), deregistered/leftover worktree exclusion, rogue-standalone-repo exclusion (r3), missing-corresponding-main-repo exclusion (r3), symlinked-package skip, symlinked-layer skip, main-vs-worktree conflict, worktree-vs-worktree conflict, identical-declaration dedupe, shape-vs-conflict precedence, Makefile wildcard grep guard, staleness-check worktree glob. **r4**: in-file-duplicate dedupe, conflict-check-time vanish, publish-time vanish, exit-code precedence (4 over 7, 6 over 7), bootstrap.sh exit-7 branch |
| `.agent/scripts/tests/test_worktree_remove.sh` | **r4**: new `setup_with_layer_worktree` fixture helper; cases for regen-on-removal-with-rosdep-yaml, no-regen-without-one, and a loud-but-non-fatal regeneration failure |
| `AGENTS.md` | `rosdep_local_sources.sh` Script Reference row: document worktree discovery, the registration/symlink filters, and exit code 6; note the same for `rosdep_local_staleness_check.sh`'s row and `bootstrap.sh`'s Environment Setup mention if their own discovery/exit-code documentation needs it. **r4**: `rosdep_local_sources.sh` row gains exit 7 + the test hooks; `worktree_remove.sh` row gains the regen-on-removal contract |

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
| New exit code 6 | `.agent/scripts/rosdep_local_sources.sh`'s own header-comment exit-code table AND `bootstrap.sh`'s exit-code switch (Plan Review r1 Finding 1) | Yes |
| Discovery now needs a git-worktree-registration check + symlink skip | New shared helpers in `.agent/scripts/_worktree_helpers.sh`, used identically by both callers (generator and staleness check) so they can't drift | Yes |

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

None. The owner's original checkpoint comment resolved reach, conflict
semantics, and the `AGENTS.md` approval; Plan Review r1 then found three
concrete gaps (bootstrap.sh's missing exit-6 branch, symlink-induced
re-discovery, stale-worktree persistence), and the owner's plan-checkpoint
comment (revision r2 above) resolved all three directly, explicitly waiving a
second plan-review round. The two items this plan itself decided (rather
than being told) — **staleness-check scope** (step 5: extend its glob too)
and **conflict/shape-rejection exit-code precedence** (step 3: shape (4) wins
over conflict (6) when both occur) — were confirmed, not objected to, by
Plan Review r1's Finding 5. Nothing remains open; the pre-push code review
(`/review-code`) is the next independent read, per the owner's own framing.

## Estimated Scope

Single PR (local-first; no PR opened by this planning step per the issue's
"Do NOT open a PR" instruction — `review-plan`/`/run-issue` publish later).
Still a single narrow generator/enforcement fix after the r2 revision — the
added scope (registration check, symlink skip, `bootstrap.sh` branch) is all
inside the same one script + its two callers + its test file, not a new
surface.
