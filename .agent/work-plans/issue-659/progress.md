---
issue: 659
---

# Issue #659 — rosdep_local_sources.sh: discover rosdep.yaml in layer worktrees, not only layers/main (first-use gap from #654)

## Issue Review
**Status**: complete
**When**: 2026-09-23 (see commit timestamp for exact time)
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #659
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Actions
- [ ] Recommendation: update the `rosdep_local_sources.sh` entry in `AGENTS.md`'s
      Script Reference table in the same PR — it currently documents the
      generator as globbing only `layers/main/*_ws/src/*/rosdep.yaml`, which
      would become inaccurate once the worktree glob is added (Consequences
      Map: "A script in `.agent/scripts/`" → "Script reference table in
      `AGENTS.md`").
- [ ] Recommendation: note in the PR (not necessarily fix) that
      `stage_rosdep_manifests.sh` (agent-image bake) and `ci_local.sh` use
      their own, narrower rosdep.yaml discovery (per-repo root only, or
      package.xml staging from `layers/main`) and are correctly **out of
      scope** here — they operate on a repo already resolved to a specific
      checkout, not on a dev-host-wide glob — but a reviewer should confirm
      that reasoning rather than assume it silently.

## Plan Authored
**Status**: complete
**When**: 2026-09-23 12:26 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-659/plan.md` at `fbadffa`
**Branch**: feature/issue-659 at `fbadffa`
**Phases**: single

### Open questions
- [ ] No open questions — plan is review-plan-ready. (Two implementation-level
      calls the plan itself made, flagged for review-plan to confirm rather
      than re-decided as policy: staleness-check glob extended to match the
      generator (step 5); shape-rejection exit 4 takes precedence over the
      new conflict exit 6 when both occur in one run (step 3).)

## Plan Review
**Status**: complete
**When**: 2026-09-23 12:35 -04:00
**By**: Claude Code Agent (Claude Sonnet)
**Verdict**: needs-work

**Issue**: #659 — rosdep_local_sources.sh: discover rosdep.yaml in layer worktrees, not only layers/main (first-use gap from #654)
**Plan**: `.agent/work-plans/issue-659/plan.md` at `fbadffa`
**Branch**: `feature/issue-659`

### Evaluation

| Dimension | Verdict | Notes |
|---|---|---|
| Scope | Good | Single narrow generator/enforcement fix, well contained |
| Issue alignment | Good | Implements the owner's three checkpoint decisions (host-wide union, conflict detection with its own exit code, AGENTS.md update) |
| File targeting | Needs work | `bootstrap.sh` is a documented caller of `rosdep_local_sources.sh` with its own exit-code switch and is missing from "Files to Change" — see Finding 1 |
| Consequences | Needs work | Worktree-symlink discovery duplication and stale-worktree persistence are real consequences of "host-wide union" that the Consequences table doesn't capture — see Findings 2 and 3 |
| Principle alignment | Good | "A change includes its consequences" is addressed for AGENTS.md/staleness-check but not for bootstrap.sh (Finding 1) |
| ADR compliance | Good | 0007/0003/0004-0005 correctly triggered and addressed |
| ROS conventions | N/A | Workspace infra, no ROS API surface |

### Findings

1. **[File targeting / Consequences — bootstrap.sh not updated]** `bootstrap.sh` (lines ~169-179) has its own exit-code switch on `rosdep_local_sources.sh`'s return value: `-eq 0` exports `ROSDEP_SOURCE_PATH`, `-eq 4` also exports it (with a rejection note, since the directory is still written and usable), and everything else falls through to `else: "workspace-local rosdep sources not generated — using system defaults"` (does NOT export `ROSDEP_SOURCE_PATH`). The plan's new exit 6 (conflict-only, no shape rejection) also leaves the directory written and safe per step 3's own text ("the directory is written either way") — but bootstrap.sh has no `elif -eq 6` branch, so a conflict-only run silently falls into the generic `else` and discards the *entire* generated directory, including every non-conflicting key, even though it's valid and ready to use. This is the same class of bug the exit-4 branch was written to avoid. `bootstrap.sh` is missing from the plan's "Files to Change" table entirely. **Action**: add `bootstrap.sh` to Files to Change with an `elif [ "$BOOTSTRAP_GEN_RC" -eq 6 ]` branch mirroring the exit-4 one. (The Makefile stamp recipe, by contrast, already handles this correctly — any non-zero, non-3 exit is `exit $$rc`, so 6 fails the build exactly like 4 does; no gap there.)

2. **[Design — naive glob rediscovers layers/main through worktree symlinks]** `worktree_create.sh` symlinks every sibling package *not* in `--packages` directly to its `layers/main` checkout (`ln -s "$pkg_path" "$WORKTREE_DIR/${LAYER_WS}/src/$pkg_name"`, ~line 1015), and symlinks every *non-target layer* wholesale to main (`ln -s "../../main/${LAYER_WS}" "$WORKTREE_DIR/${LAYER_WS}"`, ~line 1029). Verified experimentally that bash's `*` glob traverses symlinked directory components transparently, so `layers/worktrees/*/*_ws/src/*/rosdep.yaml` as specified will match every existing `layers/main` `rosdep.yaml` again, once per active layer worktree that happens to include that layer/package — not just genuinely worktree-only files. This isn't a correctness bug (the conflict-detection dedup treats byte-identical content as non-conflicting, so no false conflict is raised), but it means the generated list and the shape/conflict-detection passes scale with (repos × active worktrees) rather than (repos), and nothing in the plan's test list exercises this. The codebase already has established precedent for exactly this problem: `_worktree_helpers.sh`'s `wt_layer_branch`/`wt_layer_pkg_dir` explicitly skip symlinked `*_ws` entries and symlinked `src/*` package entries when walking a layer worktree, for this reason. **Action**: either follow that skip-symlinks convention in the new glob/discovery loop, or dedupe discovered paths by `realpath` before validation, and add a test case for a layer worktree with untouched (symlinked) sibling packages.

3. **[Consequences — stale/abandoned worktree persistence, not addressed]** Confirmed `worktree_remove.sh`'s normal path (`rm -rf "$WORKTREE_DIR"` + `git worktree remove`) does clean up a worktree's `rosdep.yaml` along with everything else. But `merge_pr.sh`'s own documented exit 3 ("merge LANDED but a post-merge step failed" — worktree removal is one of the named steps) and manual/interrupted removals mean a worktree directory can persist on disk after its branch is merged or abandoned, with nothing that expires it. Since `ROSDEP_SOURCE_PATH` is the one host-shared directory for every shell, an orphaned worktree's local key would keep silently feeding every `rosdep install` on the host indefinitely — the unintentional version of the very risk the owner's checkpoint comment flagged for the *intentional* case ("resolves for every build on the host until that worktree is removed"). Neither the plan nor its tests mention this. Given precedent (worktree cleanup relies on the existing scripts/audits, not a new mechanism here), a code fix is arguably out of scope, but the plan should say so explicitly rather than leave it unaddressed — e.g. a one-line note in the script header / AGENTS.md row that a key from an abandoned worktree persists until the directory is removed, with `worktree_list.sh` / periodic audits as the existing mitigation.

4. **[Confirmed — dedup/exclusion mechanism is sound]** Checked the installed `rosdep2` source (`lookup.py`: `RosdepDefinition.reverse_merge`, `RosdepLookup.get_rosdep_view` via `SourcesListLoader.ALL_VIEW_KEY`): a root-level `rosdep install`/`resolve` merges ALL loaded sources into one view, and a key declared in two sources is merged at the OS-name level with **first-loaded-wins** — rosdep does not error on a duplicate key, it silently picks a side (and since `30-workspace-local.list` is generated sorted by path, `layers/main` would sort before `layers/worktrees/...`, i.e. today's un-fixed script would already be quietly deterministic-but-wrong on a real conflict). The plan's approach — excluding *every* file that declares a conflicting key from the generated list, so neither side's declaration ever reaches rosdep — is therefore the right mechanism: it fails closed (key doesn't resolve) rather than reproducing the silent last-wins the owner's checkpoint explicitly rejected. One minor, precedent-consistent side effect worth a one-line callout in the PR: exclusion is per-*file* (matching the existing shape-gate granularity), so a file with one conflicting key and one otherwise-fine, non-conflicting key loses both when excluded — collateral loss of a legitimate key, same trade-off the shape gate already makes for a malformed file.

5. **[Confirmed — the two flagged implementation-level calls]** (a) Extending `rosdep_local_staleness_check.sh`'s glob to worktree yamls: agreed — the check must audit exactly the key set the generator feeds into the host-shared source path, and the plan's own reasoning (same trade-off `make validate` already accepts for `layers/main`-only keys) holds. (b) Shape-rejection exit 4 taking precedence over the new conflict exit 6 when both occur: agreed — shape validity is the stricter, pre-existing gate, and keeping it load-bearing preserves today's `make build`/bootstrap.sh failure semantics for the already-shipped case. No objection to either.

### Summary

The plan correctly implements the owner's three checkpoint decisions and its conflict-exclusion mechanism is validated against rosdep's actual (first-wins, no-error) merge behavior — dedup-by-value is the right call and full-file exclusion is necessary, not optional. However, it has one concrete caller gap (bootstrap.sh's exit-code switch doesn't handle the new exit 6, silently discarding a valid partial directory) and two unaddressed consequences of "host-wide union" (worktree-symlink-induced rediscovery of layers/main content, and stale/abandoned worktree directories persisting a key indefinitely on a host-shared path). Not ready for implementation as written.

### Recommended Actions

- [ ] Add `bootstrap.sh` to Files to Change; add an `elif -eq 6` branch mirroring the existing `-eq 4` branch (export `ROSDEP_SOURCE_PATH`, note that a conflict was excluded).
- [ ] Either skip symlinked `*_ws`/`src/*` entries when walking `layers/worktrees/*` (matching `_worktree_helpers.sh`'s existing convention) or dedupe discovered paths by realpath before shape/conflict validation; add a test case for a layer worktree with untouched (symlinked) sibling packages.
- [ ] Add a one-line note (script header and/or AGENTS.md row) documenting that a key from an abandoned/un-removed worktree directory persists on the host-shared source path until the directory is removed.
- [ ] Optional: one-line PR callout that per-file exclusion can collaterally drop a non-conflicting key sharing a file with a conflicting one (precedent-consistent, not a new defect).

## Implementation
**Status**: complete
**When**: 2026-09-23 12:49 -04:00
**By**: Claude Code Agent (Claude Sonnet)
**Branch**: feature/issue-659 at `cc10852`

Implemented the r2-revised plan (plan.md commit `13d0776`, folding in all
three Plan Review r1 findings the owner approved at the plan checkpoint).

### Changes
- `.agent/scripts/_worktree_helpers.sh`: new `wt_is_registered` (checks git
  still registers a worktree by running `git worktree list --porcelain`
  from the worktree's own directory — one command covers both "is this a
  live checkout" and "is it still registered", fails closed on any git
  error) and `wt_discover_local_rosdep_yamls` (shared two-glob discovery:
  `layers/main` + registered `layers/worktrees/*`, skipping symlinked
  `*_ws`/package entries per the existing `wt_layer_*` convention).
- `rosdep_local_sources.sh`: uses the shared discovery; new conflict-detection
  pass (a key declared with different package lists by two or more files is
  excluded from the generated list, never silently merged) with new exit
  code 6; header comment updated. Exit 4 (shape rejection) still wins over 6
  when both occur in one run.
- `rosdep_local_staleness_check.sh`: uses the same shared discovery function,
  so its audited key set can't drift from the generator's.
- `bootstrap.sh`: new `elif -eq 6` branch mirroring the existing exit-4
  branch (directory still written and valid; export it with a note).
- `Makefile`: `ROSDEP_LOCAL_YAMLS` gains the worktree wildcard; comment
  reconciles `$(wildcard)` symlink-following with the generator's symlink
  skip (harmless — extra prerequisite re-checks only).
- `AGENTS.md`: both Script Reference rows updated (discovery, conflict
  detection, exit 6, shared-discovery note).
- `.agent/scripts/tests/test_rosdep_local_sources.sh`: new cases for
  registered-worktree discovery, leftover/deregistered-worktree exclusion
  (real `git worktree add` fixtures + admin-dir removal, not a plain
  directory), symlinked-layer and symlinked-package skip, main-vs-worktree
  and worktree-vs-worktree conflicts, identical-declaration dedupe,
  shape-vs-conflict precedence, a Makefile wildcard grep guard, and a
  worktree-only unmarked key caught by the staleness check.

### Test results
`.agent/scripts/tests/test_rosdep_local_sources.sh`: **147 passed, 0
failed** (includes all pre-existing #654 cases, unchanged and still green).
All 7 commits' pre-commit hooks (including shellcheck) passed at commit
time; a standalone `pre-commit run` was not separately invocable in this
worktree's shell (no `pre-commit` on PATH outside the git-hook invocation),
but every touched file already passed shellcheck via the commit-time hook.
`make -n -p` confirms the Makefile still parses and the new
`ROSDEP_LOCAL_YAMLS` wildcard resolves against the real worktree fixture
named in the issue (`unh_marine_autonomy#397`'s rosdep.yaml).

### Deviations from the plan
- No new `bootstrap.sh` test harness was built for the exit-6 branch — the
  plan itself flagged this as a code-review-level check (no bootstrap.sh
  test file exists today; it drives real package installs) rather than a
  gap, consistent with the "only what's needed" principle.
- Everything else matches the r2-revised plan as written.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-09-23 13:40 -04:00
**By**: Claude Code Agent (Claude Sonnet 5)
**Verdict**: changes-requested

**Branch**: feature/issue-659 at `7fddd92`
**Mode**: pre-push
**Depth**: Deep (reason: ~490 changed lines across the script/test set, plus override-trigger files Makefile and AGENTS.md, plus command-injection-surface scripts shelling out to python3/git)
**Must-fix**: 1 | **Suggestions**: 3
**Round**: 1 | **Ship**: continue — one real fail-open gap in the exact function the checkpoint decisions rely on for safety; cheap to fix, worth a second pass rather than shipping past it

### Findings
- [ ] (must-fix) `wt_is_registered` only proves `dir` is *some* live git repo whose own `worktree list` includes itself (trivially true for any standalone git repo, worktree or not) — it does not verify the checkout belongs to the expected project repo, so a stray/rogue git repo placed under `layers/worktrees/*/*_ws/src/*/` would be silently trusted and its rosdep.yaml merged into the host-shared `ROSDEP_SOURCE_PATH` — `.agent/scripts/_worktree_helpers.sh:157-178`
- [ ] (suggestion) PyYAML `safe_load` in the conflict-detection heredoc is alias/anchor-amplification DoS-susceptible in principle — low priority, files are local/workspace-controlled — `.agent/scripts/rosdep_local_sources.sh:269-306`
- [ ] (suggestion) `git worktree list --porcelain` parsing via line-by-line `read` would misparse a worktree/branch path containing an embedded newline — fail-closed direction only (false-exclude, never false-include) — `.agent/scripts/_worktree_helpers.sh:163-176`
- [ ] (suggestion) `wt_is_registered`'s header comment (lines 139-156) implies exclusion always comes from the git command itself failing; in practice a plain non-git leftover directory instead succeeds via upward discovery into the outer workspace repo and is excluded by the path-mismatch instead — behavior is still correctly fail-closed, doc-precision only — `.agent/scripts/_worktree_helpers.sh:139-156`
