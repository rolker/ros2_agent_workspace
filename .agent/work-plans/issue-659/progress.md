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
- [x] (must-fix) `wt_is_registered` only proves `dir` is *some* live git repo whose own `worktree list` includes itself (trivially true for any standalone git repo, worktree or not) — it does not verify the checkout belongs to the expected project repo, so a stray/rogue git repo placed under `layers/worktrees/*/*_ws/src/*/` would be silently trusted and its rosdep.yaml merged into the host-shared `ROSDEP_SOURCE_PATH` — `.agent/scripts/_worktree_helpers.sh:157-178`
- [x] (suggestion) PyYAML `safe_load` in the conflict-detection heredoc is alias/anchor-amplification DoS-susceptible in principle — low priority, files are local/workspace-controlled — `.agent/scripts/rosdep_local_sources.sh:269-306` (deferred: review's own assessment — low priority in principle, and every input file is workspace-controlled/local, never attacker-supplied; not trivially fixable without hand-rolling a restricted YAML loader, so left as a known, accepted limitation rather than addressed here)
- [x] (suggestion) `git worktree list --porcelain` parsing via line-by-line `read` would misparse a worktree/branch path containing an embedded newline — fail-closed direction only (false-exclude, never false-include) — `.agent/scripts/_worktree_helpers.sh:163-176` (deferred: review's own assessment confirms the only failure direction is false-exclude — a spurious "not registered" skip, never a false-include — so it cannot reopen the security gap this round exists to close; not trivially fixable within `--porcelain`'s newline-delimited format, so left as a known, accepted limitation)
- [x] (suggestion) `wt_is_registered`'s header comment (lines 139-156) implies exclusion always comes from the git command itself failing; in practice a plain non-git leftover directory instead succeeds via upward discovery into the outer workspace repo and is excluded by the path-mismatch instead — behavior is still correctly fail-closed, doc-precision only — `.agent/scripts/_worktree_helpers.sh:139-156` (superseded: the must-fix's rewrite replaced this header comment entirely with one that names both exclusion paths — main repo missing vs. main repo exists but doesn't register the checkout — precisely, so the imprecision no longer exists to fix separately)

## Implementation
**Status**: complete
**When**: 2026-09-23 13:07 -04:00
**By**: Claude Code Agent (Claude Sonnet 5)

**Branch**: feature/issue-659 at `97c3943`
**Addressed**: Local Review (Pre-Push), When 2026-09-23 13:40 -04:00, SHA `7fddd92`
**Commits**: 1bb5c84, 8ad4df6, 97c3943

### Changes
- `.agent/scripts/_worktree_helpers.sh`: `wt_is_registered` now takes a
  second argument, `main_repo_dir` — the expected
  `layers/main/<ws>/src/<pkg>` repo — and requires BOTH that the discovered
  checkout shares that repo's `git rev-parse --path-format=absolute
  --git-common-dir` (the identity anchor: an independent standalone repo
  never shares a commondir, even though its own `worktree list` trivially
  lists itself) and that the main repo's OWN `worktree list --porcelain`
  (queried from the main repo, not from the discovered dir) still names it.
  `wt_discover_local_rosdep_yamls` computes the expected main-repo path from
  each worktree entry's layer/package basenames and fails closed with a
  named reason distinct from "not registered" when that repo doesn't exist
  at all (a stale worktree path outliving a rename/removal of the main
  repo — the case the issue's checkpoint asked to be decided explicitly).
  Header comment for `wt_is_registered` rewritten to describe both
  exclusion paths precisely (suggestion 3).
- `.agent/scripts/tests/test_rosdep_local_sources.sh`: reworked the
  worktree fixture helpers (`add_layer_worktree` replaces `add_git_worktree`)
  so every worktree fixture in the file is a genuine linked worktree of its
  corresponding `layers/main/<ws>/src/<pkg>` repo, matching what
  `worktree_create.sh` actually produces — the old fixtures built worktrees
  from disconnected throwaway repos with no `layers/main` counterpart at
  all, which is exactly what let the must-fix gap go uncaught. Added two
  regression cases: a rogue standalone repo at the worktree path shape
  (excluded, even though its own `worktree list` trivially lists itself),
  and a worktree with no corresponding `layers/main` repo (excluded, "no
  corresponding ... repo exists").
- `.agent/work-plans/issue-659/plan.md`: added an r3 revision section
  recording why the r2 "mechanism chosen" note was incomplete and what
  replaced it, plus updated the Files-to-Change rows for
  `_worktree_helpers.sh` and the test file.

### Test results
- `.agent/scripts/tests/test_rosdep_local_sources.sh`: **154 passed, 0
  failed** (was 147; +7 checks across the two new sections).
- `.agent/scripts/tests/test_worktree_create.sh`: **33 passed, 0 failed**.
- `.agent/scripts/tests/test_worktree_remove.sh`: **2 passed, 0 failed**.
- All 3 commits' pre-commit hooks (including shellcheck) passed at commit
  time.

### Actions
- [x] must-fix: `wt_is_registered` now verifies the discovered checkout is
      a linked worktree of the corresponding `layers/main/<ws>/src/<pkg>`
      repo specifically, via git-common-dir identity plus that repo's own
      `worktree list --porcelain`; a missing corresponding main repo fails
      closed with its own named reason — `.agent/scripts/_worktree_helpers.sh`
- [x] suggestion 3: header comment now describes both exclusion paths
      (main repo missing vs. main repo exists but doesn't register the
      checkout) — `.agent/scripts/_worktree_helpers.sh`
- [x] suggestion 1 (deferred: low priority, workspace-controlled/local
      files only, not trivially fixable without a restricted YAML loader)
      — `.agent/scripts/rosdep_local_sources.sh:269-306`
- [x] suggestion 2 (deferred: fail-closed direction only — false-exclude,
      never false-include — so it cannot reopen the security gap this
      round exists to close; not trivially fixable within `--porcelain`'s
      newline-delimited format) — `.agent/scripts/_worktree_helpers.sh:163-176`

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-09-23 13:13 -04:00
**By**: Claude Code Agent (Claude Sonnet 5)
**Verdict**: changes-requested

**Branch**: feature/issue-659 at `86d779c`
**Mode**: pre-push
**Depth**: Deep (reason: ~490 changed lines across the script/test set, override-trigger files Makefile and AGENTS.md, and a security-relevant trust-check function feeding a host-shared rosdep source path)
**Must-fix**: 1 | **Suggestions**: 1
**Round**: 2 | **Ship**: recommended — must-fix count held at 1 (not rising vs. round 1), the fix is a precise mechanical existence-recheck, not a design question

### Findings
- [ ] (must-fix) `wt_is_registered`'s round-1 rewrite is verified sound (independently re-derived and empirically re-tested by both adversarial passes: git-common-dir identity + main repo's own `worktree list --porcelain` correctly rejects a rogue standalone repo and a worktree of an unrelated repo) — no new finding on it, confirming the round-1 must-fix is genuinely fixed — `.agent/scripts/_worktree_helpers.sh:157-231`
- [ ] (must-fix) A file that passes the shape gate into `passed_yamls` can be deleted before the final publish loop (routine now that worktree teardown races feed this generator, not just a theoretical layers/main race) — the conflict-detection Python subprocess's bare `except Exception: continue` on a later read failure silently treats it as "declares no keys" rather than excluding it, and the publish loop at the end writes `yaml file://<path>` for every `passed_yamls` entry not explicitly excluded, with no re-check that the file still exists. A worktree removed between the shape-gate pass and publish lands a dangling source line in the host-shared `ROSDEP_SOURCE_PATH`, which every `rosdep install -y` on the host (make build, ci_local, agent-image bake) consults until the next regeneration — Claude Adversarial / Lens B — `.agent/scripts/rosdep_local_sources.sh:235-246,270-282,327-334`
- [ ] (suggestion) Conflict-detection canonicalization sorts each OS's package list but doesn't dedupe first, so an accidental in-file duplicate (`ubuntu: [foo, foo]`) canonicalizes differently from a clean `ubuntu: [foo]` declared elsewhere for the same key — two semantically-identical declarations could spuriously exit 6 as a conflict. Low likelihood, easy fix (`sorted(set(pkgs))`) — Claude Adversarial / Lens A — `.agent/scripts/rosdep_local_sources.sh:~287`

### Round 2 context
Round 1's must-fix (`wt_is_registered` fail-open on a rogue standalone repo) is CONFIRMED FIXED — both fresh-context adversarial passes independently re-derived and empirically re-tested the two-check identity mechanism (git-common-dir match + main repo's own worktree registry) against the same adversarial scenarios and found it sound, with no way to make it wrongly admit an untrusted directory. The two round-1 deferred suggestions (YAML alias-bomb DoS, `--porcelain` embedded-newline misparse) were not re-raised and remain accepted low-priority limitations per round 1's own assessment.

The new must-fix is a distinct issue from round 1's, surfaced only by Lens B's focus on concurrency under the widened discovery set: #659 turns worktree churn (routine create/remove by concurrent agent sessions) into content feeding a directory every build on the host consults, which makes a shape-gate-to-publish TOCTOU gap a realistic failure mode rather than a theoretical one.

## Implementation
**Status**: complete
**When**: 2026-09-23 13:35 -04:00
**By**: Claude Code Agent (Claude Sonnet 5)

**Branch**: feature/issue-659 at `874f1b4`
**Addressed**: the owner's binding "Pre-push round 2 decision" comment on issue #659 (2026-09-23 13:23 -04:00), which itself resolved Local Review (Pre-Push) round 2's two open findings (When 2026-09-23 13:40 -04:00, SHA `86d779c`)
**Commits**: 1ee8ffe, b963204, 874f1b4

### Changes
- `.agent/scripts/rosdep_local_sources.sh`: the conflict-detection Python
  subprocess now reports `UNREADABLE\t<path>` instead of silently treating a
  read failure as "declares no keys"; the bash side excludes and names the
  file. The final publish loop re-checks `[ -f "$yaml" ]` immediately before
  writing each line, closing the remaining race window between
  conflict-check and publish. New exit code **7** (a file vanished mid-run),
  distinct from and outranked by 4 (shape rejection) and 6 (conflict) —
  precedence 4 > 6 > 7, documented in the script header and the AGENTS.md
  row. Package-list canonicalization now dedupes (`sorted(set(pkgs))`)
  before sorting, per the folded-in suggestion. Two test-only env hooks
  (`ROSDEP_LOCAL_SOURCES_TEST_VANISH`, `...TEST_VANISH_AFTER_CONFLICT`) let
  the regression suite reproduce the race deterministically.
- `.agent/scripts/bootstrap.sh`: new `elif -eq 7` branch mirroring the
  existing exit-4/exit-6 branches — exports `ROSDEP_SOURCE_PATH` with a
  note rather than falling into the generic "not generated" branch, which
  would have discarded the whole valid directory over one raced file.
- `.agent/scripts/worktree_remove.sh`: Phase 1's existing per-package walk
  (layer worktrees) now also checks each non-symlinked package for a root
  `rosdep.yaml`, before the destructive cleanup. After a successful
  removal, if any was found, regenerates the workspace-local rosdep
  sources dir via `rosdep_local_sources.sh <MAIN root>` — never runs
  `rosdep update` (network, slow; noted in-line). A regeneration failure
  is reported loudly with the exact re-run command but does not fail the
  script — the removal itself already succeeded. Confirmed `merge_pr.sh`
  removes worktrees by calling this script directly, so the merge path is
  covered with no separate change.
- `AGENTS.md`: both Script Reference rows updated — `rosdep_local_sources.sh`
  gains exit 7 and the test hooks; `worktree_remove.sh` gains the
  regen-on-removal contract.
- `.agent/scripts/tests/test_rosdep_local_sources.sh`: new cases for the
  in-file-duplicate dedupe, a conflict-check-time vanish, a publish-time
  vanish, exit-code precedence (4 over a simultaneous 7, 6 over a
  simultaneous 7 — using a third unrelated file so both conditions
  genuinely fire in one run, since a file already excluded by a conflict
  never reaches the publish-time existence check), and that `bootstrap.sh`
  has an explicit exit-7 branch that still exports `ROSDEP_SOURCE_PATH`.
- `.agent/scripts/tests/test_worktree_remove.sh`: new `setup_with_layer_worktree`
  fixture (a genuine linked layer worktree of a `layers/main/<ws>/src/<pkg>`
  repo, mirroring `test_rosdep_local_sources.sh`'s `add_layer_worktree`) and
  three cases: regen on removal-with-rosdep-yaml, no regen without one, and
  a stubbed regeneration failure that is reported loudly while the removal
  itself still reports success.
- `.agent/work-plans/issue-659/plan.md`: r4 revision section recording the
  round-2 decision, the implementation, and test counts; Files-to-Change
  rows updated for the four touched scripts plus both test files and
  AGENTS.md.

### Decision: exit code 7

A file that vanishes mid-run is neither a shape rejection (4 — a policy
violation, the file's *content* is wrong) nor a conflict (6 — two files
genuinely disagree about a key's value). It is a transient, single-file
race whose remedy is "re-run; the file may still be there" — a different
condition deserving its own code so a caller (a human, `bootstrap.sh`, the
Makefile stamp) can tell them apart. Precedence 4 > 6 > 7: each is a
stricter, earlier-established gate than the one after it, so a run that
hits more than one still reports the strictest, preserving `make build`'s
and `bootstrap.sh`'s existing failure semantics for the already-shipped
cases (4, 6). Like 4 and 6, the directory is still written minus the
excluded file(s) — a raced file costs only itself, not every other key on
the host. `bootstrap.sh` and the Makefile stamp both needed to (Makefile)
or already did (Makefile's recipe treats any non-zero, non-3 exit
identically) handle it; `bootstrap.sh` needed an explicit new branch, added
here, following the exact pattern the plan-review had already flagged as
missing for exit 6's own bootstrap.sh gap in an earlier round.

### Test results
- `.agent/scripts/tests/test_rosdep_local_sources.sh`: **171 passed, 0
  failed** (was 154; +17 across the new race/precedence/dedupe cases).
- `.agent/scripts/tests/test_worktree_remove.sh`: **5 passed, 0 failed**
  (was 2; +3 new).
- `.agent/scripts/tests/test_worktree_create.sh`: **33 passed, 0 failed**
  (unchanged — no target file touched, re-run to confirm no regression).
- All 4 commits' pre-commit hooks (including shellcheck) passed at commit
  time.

### Deviations from the decision
None — implemented as decided: race fix (both checkpoints), regenerate on
layer-worktree removal (checked before removal, regenerated after, failure
reported loudly and non-fatally, merge path confirmed covered via
`merge_pr.sh` → `worktree_remove.sh`), and the dedupe suggestion folded in.
Tests remained hermetic throughout — no test wrote to the real
`$HOME/project11/.rosdep`; both new suites use `ROSDEP_SYSTEM_SOURCES_DIR`
fixtures and mock workspace trees under `mktemp -d`.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-09-23 13:43 -04:00
**By**: Claude Code Agent (Claude Sonnet 5)
**Verdict**: changes-requested

**Branch**: feature/issue-659 at `efd2ade`
**Mode**: pre-push
**Depth**: Deep (reason: ~1800 changed lines across the script/test set, override-trigger files Makefile and AGENTS.md, and a security-relevant concurrency/race-fix touching a host-shared rosdep source path)
**Must-fix**: 2 | **Suggestions**: 1
**Round**: 3 | **Ship**: continue — two must-fix findings, both new to this round's TOCTOU-handling work and cross-pass confirmed by two independent fresh-context reads; one is a precise mechanical one-line fix, the other is a real security-relevant design question (which mitigation to gate the delete-capable test hook with) that warrants a decision, not an obvious-correction patch

### Round 3 verification of round 2's must-fix
Independently re-verified: the round-2 TOCTOU race fix (a shape-gate-passed
`rosdep.yaml` deleted before conflict-check/publish) is genuinely fixed.
Re-read `.agent/scripts/rosdep_local_sources.sh` end to end — the conflict
Python subprocess now reports `UNREADABLE` explicitly instead of treating a
read failure as "declares no keys", and the publish loop re-checks `[ -f
"$yaml" ]` immediately before writing each line, closing both halves of the
race window. Ran both regression suites locally and confirmed green:
`test_rosdep_local_sources.sh` 171/171, `test_worktree_remove.sh` 5/5 (plus
`test_worktree_create.sh` 33/33 as an unrelated-regression check).

### Findings
- [x] (must-fix) `worktree_remove.sh:442-443` — `if ! "$SCRIPT_DIR/rosdep_local_sources.sh" "$ROOT_DIR"; then REGEN_RC=$?` captures `$?` from the negated `!` test, not the wrapped command's real exit status — `$?` is always 0 inside that `then` branch (empirically verified: `f(){ return 4;}; if ! f; then echo $?; fi` prints `0`). The operator-facing warning "exit $REGEN_RC — see messages above" therefore always claims "exit 0" regardless of whether the generator actually hit a lock timeout (5), shape rejection (4), conflict (6), or vanished file (7) — misleading for exactly the person the message is written to help triage. The existing test (`test_layer_removal_regen_failure_does_not_fail_removal`) only asserts the substring "regeneration exited non-zero", never the printed number, so it doesn't catch this. Fix: `"$SCRIPT_DIR/rosdep_local_sources.sh" "$ROOT_DIR" || REGEN_RC=$?` (the form `bootstrap.sh` already uses correctly) or an un-negated `if`. — Claude Adversarial / Lens A + Lens B (independently, cross-pass confirmed)
- [x] (must-fix) `rosdep_local_sources.sh:277-279,397-399` — the two test-only hooks (`ROSDEP_LOCAL_SOURCES_TEST_VANISH`, `..._AFTER_CONFLICT`) run `rm -f "$path"` unconditionally whenever the env var is non-empty, with no confinement to a test/tmp root, no second "under test" sentinel, and no check that the path is one this run itself discovered. This script runs at root/build scope in dev shells, `make build`, `bootstrap.sh`, `worktree_remove.sh` (a new caller added by this PR), CI, and the agent-image bake — several of which can inherit env from a longer-lived parent process. An accidentally-set env var (e.g. left exported in a dev shell after running the test suite by hand) turns a normal production run into an arbitrary `rm -f` of whatever path the var holds — not scoped to rosdep.yaml files. AGENTS.md's Never section treats unapproved deletion as a hard stop; this is the same risk class in a code path, not an operator action. The code's own comment ("mirrors the ROSDEP_SOURCES_FORCE_FALLBACK precedent") doesn't hold — that existing hook only forces a non-destructive fallback branch, never deletes anything. Recommend either (a) a second gate var (e.g. also require `ROSDEP_LOCAL_SOURCES_TEST_MODE=1`), (b) restricting the deletable path to ones already in this run's own `passed_yamls`/`local_yamls` set, or (c) moving both hooks out of the production script into a thin test-only wrapper/stub instead of shipping delete capability in the file every real invocation runs. — Claude Adversarial / Lens A + Lens B (independently, cross-pass confirmed)
- [x] (suggestion) Exit 7 ("file vanished mid-run", described in the script's own header as a routine, transient worktree-teardown race) is treated as a hard build failure by the Makefile's `rosdep-local.done` stamp recipe (any non-zero, non-3 exit fails the recipe) but as a soft, still-usable note by `bootstrap.sh`. A `make build` racing a concurrent `worktree_remove.sh` can fail outright on something self-describedly transient/benign, even though the directory was still written correctly minus the one raced file. Not a new inconsistency — it exactly mirrors the already-shipped exit-4 precedent (same asymmetry exists today) — so not blocking this PR, but worth a follow-up issue if the transient-race framing is meant to be acted on (e.g. retry the Makefile recipe, or special-case 7 the way exit 3 already is). — Claude Adversarial / Lens A

### Round 3 context
Two fresh-context adversarial passes (Lens A: logic/correctness; Lens B:
systemic/safety) independently converged on the same two must-fix findings —
a genuine `$?`-after-`!` exit-code bug and a delete-capable env-var test hook
with no confinement — despite different prompts and no shared context,
which is a strong signal both are real. Both also independently re-verified
`wt_is_registered` (round 1's must-fix subject) remains sound after rounds
2-3's additions, and independently re-derived the exit-code precedence
(4 > 6 > 7) as correctly enforced with no double-reporting. The round-2
must-fix (shape-gate-to-publish TOCTOU race) is confirmed fixed by both
passes and by this reviewer's own re-read and test run.

## Decision summary

Issue [#659](https://github.com/rolker/ros2_agent_workspace/issues/659)
widens `rosdep_local_sources.sh`'s discovery of project-repo `rosdep.yaml`
files from `layers/main` only to also cover registered layer worktrees, so
a `rosdep.yaml` added on a feature branch resolves on the same host before
merge. It adds conflict detection (a key declared with different package
lists by two discovered files excludes both, rather than rosdep's own
silent first-loaded-wins), a new exit code 6 for that case, and — after two
rounds of pre-push review — a fix for a TOCTOU race where a discovered file
can be deleted mid-run (new exit code 7) plus logic in `worktree_remove.sh`
to regenerate the host-shared sources directory when a removed layer
worktree carried a `rosdep.yaml`, so a removal never leaves a dangling
source line. `bootstrap.sh`, the Makefile stamp, `rosdep_local_staleness_check.sh`,
and `AGENTS.md`'s Script Reference table were all updated to match. The
scope for this final round was explicitly widened by the owner (issue
comment, "Pre-push round 2 decision") to: fix the race, regenerate on
worktree removal, and dedupe package lists — all three are implemented and
covered by new regression tests (171/171 and 5/5 passing, both verified
green independently by this review).

Round 3 verified round 2's must-fix is genuinely fixed, then found two new
must-fix issues introduced by this round's own TOCTOU-handling code,
independently confirmed by two disjoint-lens adversarial passes: (1) a
`$?`-after-`!` bug in `worktree_remove.sh` that makes the regeneration-failure
warning always claim "exit 0" regardless of the real failure code, silently
defeating the diagnostic the warning exists to provide; and (2) two
test-only env-var hooks in the production `rosdep_local_sources.sh` script
that run an unconfined `rm -f` on an env-supplied path, a real (if
low-probability) arbitrary-file-deletion risk if either var were ever set
in a non-test invocation. Both are precisely located and each has 2-3
concrete fix options named above. One suggestion (exit-7-vs-Makefile
hard-fail asymmetry) is a pre-existing pattern this PR doesn't newly
introduce and can ride a follow-up issue rather than block this PR.

**Recommendation**: address both must-fix findings (the exit-code capture
is a one-line mechanical fix; the delete-hook gating needs a short decision
among the three named options, then a small patch) and re-run this review
before pushing. Not ready to ship as-is — not because the feature work is
wrong, but because the two new findings are in the exact class of thing
(misleading failure diagnostics, unconfined delete capability) this
workspace's own Quality Standard calls out as not-a-nit.

## Implementation
**Status**: complete
**When**: 2026-09-23 13:59 -04:00
**By**: Claude Code Agent (Claude Sonnet 5)

**Branch**: feature/issue-659 at `b31bbd3`
**Addressed**: the owner's binding "Pre-push round 3 decision" comment on issue #659 (2026-09-23 13:52 -04:00), which resolved Local Review (Pre-Push) round 3's two must-fix findings and one suggestion (When 2026-09-23 13:43 -04:00, SHA `efd2ade`)
**Commits**: b3c7c3e, 9801e44, e81a00b, b31bbd3

### Changes
- `.agent/scripts/rosdep_local_sources.sh`: `ROSDEP_LOCAL_SOURCES_TEST_VANISH`
  and `..._TEST_VANISH_AFTER_CONFLICT` no longer `rm -f` anything. Each hook
  now checks whether its named path is one of this run's own shape-gate-passed
  discovered files (`passed_yamls`) — if not, it is a no-op with a stderr note
  ("is not one of this run's own discovered files"); if so, the path is
  recorded and the run *simulates* it as missing at its checkpoint instead of
  deleting it. The conflict-check hook's simulated paths are passed to the
  conflict-detection Python subprocess via a new
  `ROSDEP_LOCAL_SOURCES_SIMULATE_VANISH` env var; the subprocess treats a
  listed path as unreadable without ever opening it, reporting it identically
  to a real read failure (`UNREADABLE`). The publish-time hook's simulated
  path is checked directly in the existing `[ -f "$yaml" ]` publish-loop
  recheck. Script header and the two inline hook comments rewritten to
  describe simulation, not deletion.
- `Makefile`: the `rosdep-local.done` stamp recipe gained an
  `elif "$$rc" -eq 7` branch (ordered before the general `-ne 0` fail
  branch) that prints a note and leaves the stamp stale, exactly mirroring
  the existing exit-3 handling — a vanished-file race is, by the script's
  own header, transient and self-describedly not a build failure. Exits 4
  and 6 are unchanged and still fail the recipe. Comment block above the
  recipe updated to describe the new branch.
- `AGENTS.md`: the `rosdep_local_sources.sh` Script Reference row updated —
  the test-hook description now says "simulate ... never delete it", and a
  new sentence documents the Makefile's softened exit-7 handling.
- `.agent/scripts/worktree_remove.sh`: replaced
  `if ! "$SCRIPT_DIR/rosdep_local_sources.sh" "$ROOT_DIR"; then REGEN_RC=$?`
  (which captured `$?` from the negated `!` test — always 0 — not the
  wrapped command's real exit status) with
  `REGEN_RC=0; "$SCRIPT_DIR/rosdep_local_sources.sh" "$ROOT_DIR" || REGEN_RC=$?`,
  the same `cmd || rc=$?` form `bootstrap.sh` already used correctly
  (required here too since `set -e` is active in this script).
- `.agent/scripts/tests/test_rosdep_local_sources.sh`: the two round-2
  vanish sections and both precedence tests gained a
  "the hook never deletes — the fixture file still exists" assertion
  (four call sites). A new section exercises the ignored-path case: a hook
  target outside this run's own discovery set exits 0, prints the ignore
  note, resolves the real file normally, and leaves the out-of-scope path
  untouched. The Makefile stamp section gained exit-7 (soft, retries) and
  exit-6 (still hard-fails) cases alongside the existing exit-3/exit-4 ones.
- `.agent/scripts/tests/test_worktree_remove.sh`:
  `test_layer_removal_regen_failure_does_not_fail_removal` now asserts the
  warning names the stub's actual exit code ("exit 5 — see messages
  above"), which the pre-fix `$?`-after-`!` bug would have failed (it
  always printed "exit 0" regardless of the stub's real exit code).
- `.agent/work-plans/issue-659/plan.md`: added an r5 revision section
  recording the round-3 decision, the implementation, and test counts;
  no Files-to-Change rows needed updating (all touched files were already
  listed from earlier rounds).

### Test results
- `.agent/scripts/tests/test_rosdep_local_sources.sh`: **185 passed, 0
  failed** (was 171; +14 across the hook-simulation and Makefile exit-7/6
  cases).
- `.agent/scripts/tests/test_worktree_remove.sh`: **5 passed, 0 failed**
  (unchanged count — same 3 round-2 cases, one strengthened with the
  exit-code assertion).
- `.agent/scripts/tests/test_worktree_create.sh`: **33 passed, 0 failed**
  (unchanged — no target file touched, re-run to confirm no regression).
- All 4 commits' pre-commit hooks (including shellcheck) passed at commit
  time.

### Deviations from the decision
None — implemented exactly as decided: test hooks simulate rather than
delete (scoped to this run's own discovered files, with an ignore note for
anything else), the Makefile stamp softens exit 7 like exit 3, and
`worktree_remove.sh` captures the regen's real exit code via `|| rc=$?`.

### Actions
- [x] must-fix: `worktree_remove.sh`'s `$?`-after-`!` exit-code capture bug
      fixed with `cmd || REGEN_RC=$?` — `.agent/scripts/worktree_remove.sh:441-443`
- [x] must-fix: the two test-only vanish hooks in `rosdep_local_sources.sh`
      now simulate rather than delete, scoped to the run's own discovered
      files — `.agent/scripts/rosdep_local_sources.sh:270-311,428-450`
- [x] suggestion: exit 7 softened to a note-and-retry in the Makefile
      stamp recipe, matching exit 3's treatment — `Makefile`

### Next step
Lifecycle: **Implementation** → **review-code** (re-review the fixes, pre-push round 4)

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-09-23 14:04 -04:00
**By**: Claude Code Agent (Claude Sonnet 5)
**Verdict**: approved

**Branch**: feature/issue-659 at `d5ab0c8`
**Mode**: pre-push
**Depth**: Deep (reason: ~1128 lines changed across the script/test set, plus override-trigger files Makefile and AGENTS.md, plus a security-relevant worktree-trust-check function feeding a host-shared rosdep source path)
**Must-fix**: 0 | **Suggestions**: 0
**Round**: 4 | **Ship**: recommended — no must-fix findings; both independent adversarial passes and this reviewer's own full re-read confirm all three round-3 fixes hold and found nothing new

### Round 4 verification of round 3's must-fixes and suggestion
Independently re-verified, by direct reading of the diff and both regression
suites (185/185 in `test_rosdep_local_sources.sh`, 5/5 in
`test_worktree_remove.sh`, both re-run locally), plus two fresh-context
adversarial passes:
- `worktree_remove.sh:441-443` — `REGEN_RC=0; "$SCRIPT_DIR/rosdep_local_sources.sh" "$ROOT_DIR" || REGEN_RC=$?`
  correctly captures the wrapped command's real exit status (no more
  `$?`-after-`!`). `test_worktree_remove.sh`'s
  `test_layer_removal_regen_failure_does_not_fail_removal` asserts the
  literal "exit 5" from a stub, guarding against regression to the old bug.
- `rosdep_local_sources.sh`'s two test-only hooks
  (`ROSDEP_LOCAL_SOURCES_TEST_VANISH`, `..._AFTER_CONFLICT`) no longer `rm`
  anything — each only simulates a named path as missing at its checkpoint,
  and only when that path is one of the run's own shape-gate-passed
  discovered files (`passed_yamls`); an out-of-scope path is a no-op with a
  stderr note. Verified with the dedicated "ignored path" test section.
- Makefile's `rosdep-local.done` stamp recipe treats exit 7 like exit 3
  (note printed, stamp left stale, build not failed) — verified against the
  script's own header description of exit 7 as a transient, self-describedly
  benign race.

### Findings
- [ ] No issues found. LGTM.

### Specialist notes
- **Claude Adversarial / Lens A** (logic & correctness): read `_worktree_helpers.sh`,
  `rosdep_local_sources.sh` in full, `rosdep_local_staleness_check.sh`,
  `bootstrap.sh`'s exit-code switch, `worktree_remove.sh`, the Makefile
  stamp recipe, and both test files in full — independently re-derived and
  re-tested all three round-3 fixes as sound. No new findings. One purely
  cosmetic observation (not a finding): `HAD_ROSDEP_YAML` in
  `worktree_remove.sh` is set from a raw `[ -f ... ]` check without
  re-verifying `wt_is_registered`, but the package being checked is always
  the worktree currently being torn down, so no incorrect result was
  constructible — at most a harmless extra regeneration call.
- **Claude Adversarial / Lens B** (systemic & safety): read the same file
  set with focus on the trust-check function, the simulate-only hooks, and
  the flock/atomic-swap concurrency mechanism (unchanged by this diff). No
  new findings; confirmed the exit-7-as-soft-retry Makefile handling is
  safe (the published directory is always internally consistent — only the
  stamp is left stale) and noted a pre-existing, non-blocking asymmetry
  (rosdep's cache isn't refreshed on exit 7, same as exit 3, a staleness
  window rather than a safety issue).
- **Governance / Plan Drift** (this reviewer, lead pass): owner's three
  binding decision comments on issue #659 (plan checkpoint, round 2, round 3)
  all correctly implemented; plan.md's Files-to-Change table matches the
  actual diff with no drift; AGENTS.md's two Script Reference rows
  (`rosdep_local_sources.sh`, `worktree_remove.sh`) correctly reflect the
  final behavior (exit 6/7, test-hook simulation semantics, Makefile exit-7
  softening, regen-on-removal contract). No governance concerns.
- **Static analysis**: pre-commit's shellcheck ran clean at commit time on
  every touched file (per the Implementation entries above); a standalone
  `shellcheck` binary was not available in this review's shell to re-run
  independently, so this round relies on the commit-time hook rather than a
  fresh invocation.

### Convergence note
This is round 4 of a narrow infra fix (rosdep.yaml discovery + a TOCTOU
race + an exit-code bug, all previously found and now fixed). No
must-fix or suggestion-level findings survived three independent fresh
reads (two adversarial sub-agents plus this lead pass) against the same
code. Shippable as-is.
