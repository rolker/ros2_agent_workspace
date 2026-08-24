---
issue: 609
---

# Issue #609 — `make sync` reports success and exits 0 even when repos fail to update

## Issue Review
**Status**: complete
**When**: 2026-08-24 12:16 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #609
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: needs-more-detail

### Fact-check against source

Verified against `.agent/scripts/sync_repos.py` and `Makefile` on `main`
(read at the worktree's branch point):

- `sync_repo()` returns `True` unconditionally at line 178 regardless of
  pull/fetch outcome — **confirmed**.
- Failure branches only `print`, never set a failure flag: `❌ Update
  failed` at line 155 — **confirmed at that line**; `❌ Fetch failed` is
  at line 176, not 175 as the issue states — **off by one, immaterial**.
- The unconditional summary `print("\n✅ Sync complete.")` is at line
  284 — **confirmed**.
- The only `sys.exit(1)` in the script is the `list_overlay_repos` import
  failure at line 39 — **confirmed**; no other exit path exists.
- `make sync` is `Makefile:221-222` (`sync:` target, bare
  `python3 ./.agent/scripts/sync_repos.py`) — **confirmed**, exit code
  passes straight through.
- Secondary consequence (`sync_gitbug()` still runs after a failed
  update): confirmed by construction — since `sync_repo()` always
  returns `True`, both call sites (`main()` ~248 for the root repo, ~281
  for each overlay repo) always invoke `sync_gitbug()` regardless of
  pull/fetch success.
- `make sync` running at the end of `merge_pr.sh` (post-merge, least
  likely to be noticed): confirmed — `merge_pr.sh:372` calls
  `make -C "$ROOT_DIR" sync` unconditionally after branch cleanup, under
  `set -eo pipefail` (line 26). **Consequence not mentioned in the
  issue**: once `sync_repos.py` starts exiting non-zero on failure,
  `merge_pr.sh` will abort at that line (via `set -e`) *before* printing
  its own `✅ Done: PR #… merged, cleaned up, and synced.` banner — which
  is desirable (surfaces the failure) but means the plan should decide
  whether `merge_pr.sh` needs a clearer message distinguishing "merge +
  cleanup succeeded, sync failed" from a generic script abort, since the
  worktree/branches are already gone by that point.
- Framing that the retry/pacing logic (`retry_transient`,
  `ADAPTIVE_THROTTLE`) is working as intended and out of scope: confirmed
  by reading `lib/remote_utils.py` — retry-then-report is the existing
  behavior for transient SSH resets; this issue is scoped to the
  reporting/exit-status path only, not retry behavior.

### Scope Assessment

**Well-scoped?** Mostly — single file (`sync_repos.py`), no Makefile
change needed (the `sync:` target already passes through whatever exit
code the script returns). One design point the issue doesn't resolve:
`sync_repo()` currently returns `False` for several *distinct* cases —
path missing, dirty working tree (intentional skip to protect local
changes), detached HEAD, *and* (after the fix) real network failures.
The proposed fix's item 1 asks to "distinguish failed-to-update from
skipped/path-unresolved," but doesn't fully specify the outcome
taxonomy needed for a correct exit code: a dirty or detached-HEAD repo
is an intentional, benign skip today (by design, to avoid clobbering
local work) and must **not** flip the run to exit 1, or every
`make sync` / `merge_pr.sh` run on a host with one intentionally-dirty
repo would start reporting false failures. The implementation needs an
explicit tri-state (success / benign-skip / real-failure) rather than
a bare bool, with only real-failure cases counted toward the summary
and `sys.exit(1)`. This is a correctness point for `plan-task` to
resolve, not a blocker on the issue itself.

**Right repo?** Yes — `.agent/scripts/sync_repos.py` and the `sync`
Makefile target are workspace infrastructure, not project content.

**Dependencies**: none identified.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Test what breaks | Action needed | No existing test file for `sync_repos.py` (`find` for `*sync_repos*test*` returns nothing). This changes exit-code/reporting logic that unattended callers (CI, `merge_pr.sh`) rely on — needs a test covering: all-success (exit 0), a real failure (exit 1, repo named in summary), a benign skip like a dirty repo (exit 0, not counted as failure), and the `sync_gitbug` gate not firing after a failed update. |
| A change includes its consequences | Watch | `merge_pr.sh` consumes `make sync`'s exit code via `set -e` (see fact-check above) — not itself broken, but plan-task should decide if `merge_pr.sh` needs an explicit message for "sync failed after otherwise-successful merge," since today's abrupt abort would read as a merge failure. |
| Only what's needed | OK | Fix is localized to `sync_repos.py`; no scope creep evident in the proposed fix. |
| Enforcement over documentation | OK | This is itself an enforcement fix (a script starts reporting truthfully) rather than a doc-only change. |
| Human control and transparency | OK | Louder, accurate failure reporting is squarely in line with this principle. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| 0010 — git-bug for local issue tracking | Watch | `sync_gitbug()` gating already follows the "only run when the base sync succeeded" pattern conceptually; the fix should preserve graceful degradation (no git-bug installed / no bridge configured are not failures) — confirmed the current early-returns for those cases are unaffected by this fix. |
| 0007 — Retain Make with dependency tracking | OK | No Makefile target-shape change needed; `sync:` already passes exit code through. |

### Consequences

- `AGENTS.md`'s script reference table entry for `sync_repos.py`
  ("Sync all workspace repositories (includes git-bug)") doesn't
  currently mention exit-code semantics; not required reading today, but
  once failures are surfaced via exit code, a one-line addition (fails
  non-zero on repo update failure) would help scripted callers. Not a
  blocker — flagging as a nice-to-have, not required by the consequences
  map (no interface/param/topic change here).

### Recommendations

- Resolve the tri-state outcome design (success / benign-skip /
  real-failure) explicitly in the plan before implementation, per the
  Scope Assessment above.
- Add tests per the "Test what breaks" finding.
- Decide in the plan whether `merge_pr.sh` needs a clearer failure
  message for the "sync failed" case, given `set -eo pipefail` will now
  abort it there.

### Actions
- [ ] Define explicit tri-state sync outcome (success / benign-skip / real-failure) so dirty/detached-HEAD repos don't flip `make sync` to a false-red exit 1
- [ ] Add tests for `sync_repos.py`'s new accumulation/exit-code logic (all-success, real failure, benign skip, `sync_gitbug` gating)
- [ ] Decide whether `merge_pr.sh` needs a distinct message for "sync failed after successful merge/cleanup" given `set -eo pipefail`
- [ ] Optionally note exit-code semantics in `AGENTS.md`'s `sync_repos.py` script-table entry

## Plan Authored
**Status**: complete
**When**: 2026-08-24 12:19 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-609/plan.md` at `c1d0805`
**Branch**: feature/issue-609 at `c1d0805`
**Phases**: single

### Open questions
- [ ] Confirm the `merge_pr.sh` messaging approach (step 5): catch `make sync`'s exit code and print a distinct "merged and cleaned up, but sync failed" warning banner while still exiting non-zero, instead of letting `set -eo pipefail` abort the script with no closing banner
- [ ] Should the tri-state `SyncOutcome` enum live in `sync_repos.py` or move to `lib/`? Plan defaults to local unless review disagrees

## Plan Review
**Status**: complete
**When**: 2026-08-24 12:23 -04:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-609/plan.md` at `c1d0805`
**PR**: PR-less (`--issue` mode, branch `feature/issue-609`)
**Verdict**: approve-with-suggestions

### Evaluation

| Dimension | Verdict | Notes |
|---|---|---|
| Scope | Good | One behavior file + one caller + one test module + one doc row; no drift into retry/pacing tuning (explicitly excluded, correctly) |
| Issue alignment | Good | All four Issue Review action items are answered, including the tri-state design it asked for |
| File targeting | Good | Verified by search that `sync_repos.py` and `merge_pr.sh:372` are the *only* callers (see finding 6) |
| Consequences | Good | `merge_pr.sh` handled explicitly; the `test_merge_pr.sh` row can be closed now rather than left open (finding 5) |
| Documentation & instruction impact | Good | Non-silent; AGENTS.md row named, instruction candidates explicitly "none" with a reason |
| Principle alignment | Good | False-red avoidance is the core design driver; "Only what's needed" respected |
| ADR compliance | Good | 0010 correctly identified as the only one really touched; no exit-code convention doc exists in `.agent/knowledge/` to comply with |
| ROS conventions | N/A | Workspace infrastructure, no ROS surface |

### Early-return enumeration (independent re-read of `sync_repo()`)

| Line | Path | Plan says | Review |
|---|---|---|---|
| 126-128 | `repo_path` missing | FAILED | OK (near-unreachable from `main()`, which pre-checks `.exists()`) |
| 131-136 | dirty working tree | SKIPPED | Correct |
| 139-141 | `not branch` | SKIPPED | **Conflates two cases — see finding 1** |
| 155 | `❌ Update failed` (falls through to `return True`) | FAILED, explicit return | Correct — this is the actual bug |
| 176 | `❌ Fetch failed` (falls through to `return True`) | FAILED, explicit return | Correct |
| 168-174 | fetch OK, branch behind | SYNCED | Correct (fetch-only is the designed behavior) |
| 167 | `git status -sb` itself fails | not addressed | Prints `✅ Fetched.` regardless — cosmetic, see finding 7 |
| dry-run legs | `run_git_cmd` short-circuits `(True, "")` | SYNCED | Correct; `is_dirty`/`get_current_branch` still execute for real, so dry-run skips stay skips |

### Findings
- [x] (must-fix) `get_current_branch()` returns `""` for a real detached HEAD but `None` when the git command *fails* (not a repo, corrupt `.git`); `if not branch` collapses both. Classifying both as SKIPPED leaves a genuinely broken repo silently stale under a green exit — the exact failure class this issue exists to kill. Split: `""` → SKIPPED, `None` → FAILED, and split the message at `sync_repos.py:140` — `plan.md` step 1
- [x] (must-fix) Test isolation seams are unspecified ("monkeypatch `subprocess.run` / `run_git_cmd`"). `main()` derives `root_dir` from the module-level `SCRIPT_DIR` and calls `list_overlay_repos.get_overlay_repos()`, so as written the `main()`-level cases would read this host's real `layers/` and be non-hermetic. Name the seams: `monkeypatch.setattr(sync_repos, "SCRIPT_DIR", tmp_path/".agent"/"scripts")` (read at call time), stub `sync_repos.list_overlay_repos.get_overlay_repos`, create the repo dirs under `tmp_path` so `candidate_path.exists()` is real-but-controlled, stub `sync_repos.sync_gitbug` to count calls, and assert exit via `pytest.raises(SystemExit)`. Split the layers: test `sync_repo()` classification at the `run_network_cmd` seam (that is the test that actually catches the fall-through-to-`True` bug), and test `main()` accumulation/exit with `sync_repo` itself stubbed to return canned outcomes — otherwise the end-to-end cases assert the implementation back to itself — `plan.md` step 6
- [x] (correction) Step 2's premise is wrong: `sync_gitbug()` does **not** run today for dirty / detached / missing-path repos — `sync_repo()` early-returns `False` there, and both call sites are already gated. The only behavior change from gating on `SYNCED` is that git-bug stops running after a failed pull/fetch, which *is* the bug fix, not a "small behavior tightening beyond the literal bug fix". Drop that framing so the PR body doesn't claim a change it isn't making — `plan.md` step 2
- [x] (suggestion) Unresolvable-path → FAILED (step 3) is right, and carries no false-red risk on a fully set-up workspace (verified: `sync_repos.py --dry-run` on this host resolves every repo, zero skips). But on a host that deliberately hasn't imported a layer it makes `make sync` permanently red. Give it a distinct reason string ("not checked out — path unresolved") so the summary distinguishes it at a glance from a network failure, and say so in the PR body — `plan.md` step 3
- [x] (suggestion) `merge_pr.sh` snippet: prefer `sync_status=0; make -C "$ROOT_DIR" sync || sync_status=$?` over the `if/else` + bare `$?` form — same `set -e` safety, no reliance on `$?` surviving into an `else` branch. Also note GNU make exits **2** on a failed recipe, so the banner will usually print "exit 2", not 1 — `plan.md` step 5
- [x] (suggestion) Close the `test_merge_pr.sh` consequences row now: verified it only exercises pre-merge guard paths (arg handling, worktree resolution, field-mode guard) and by design never reaches line 372, so no new case is feasible — `bash -n` is the realistic coverage. Record that rather than leaving it as open scope — `plan.md` Consequences table
- [x] (suggestion) Enum truthiness trap: every `enum.Enum` member is truthy, so a call site left as `if sync_repo(...)` keeps working and silently treats FAILED as success. Only two call sites exist, and the planned "git-bug not called on FAILED" test catches it — worth one line in the plan so the implementer doesn't leave a bare truthiness check — `plan.md` step 2
- [x] (optional) `sync_repos.py:167-174`: when `git status -sb` itself fails, the code still prints `✅ Fetched.` Cosmetic mis-report in exactly this issue's theme; fix in passing or declare out of scope — `plan.md` step 1
- [x] (context, no action) Caller audit: `make sync` / `sync_repos.py` are invoked only from `Makefile:221-222` and `merge_pr.sh:372`. `dashboard.sh` uses `vcs` directly and never calls the script; no Makefile target depends on `sync`; no GitHub workflow references it; no skill consumes `merge_pr.sh`'s exit code; nothing in the repo documents an always-exit-0 contract (the only mention of `Sync complete` is the print itself). No further consequences

### Open-question recommendations

- **`merge_pr.sh` messaging (step 5) — recommend adopting it.** This is not scope
  creep: the plan's own change to the exit code is what creates the abort, so
  handling it is "a change includes its consequences." It is also the highest-value
  caller — the unattended one — and the one place where a bare `set -e` abort
  actively misinforms: the merge, worktree removal, and branch deletion have all
  already succeeded and are irreversible, so an abort with no closing banner reads
  as "the merge failed" and invites a retry of work that is already done. Adopt with
  the `|| sync_status=$?` form from finding 5.
- **Enum location — recommend keeping it in `sync_repos.py`.** `lib/` is for
  genuinely shared helpers (`remote_utils`, `workspace`); nothing else consumes a
  sync outcome, and promoting it now would be speculative generality against "Only
  what's needed." Moving it later costs one import line. Use a plain `enum.Enum`
  (not `str`/`IntEnum`) so members never compare equal to the old booleans.

### Summary

The plan is sound and its central design call — tri-state rather than widening the
boolean — is the right one; it correctly identifies the real bug (the two failure
prints fall through to `return True`) and correctly keeps retry/pacing out of scope.
Two things need fixing before implementation: the detached-HEAD-vs-broken-git-state
conflation, and the test isolation seams. Everything else is refinement.

### Recommended Actions
- [x] Split `not branch` into `""` (SKIPPED) and `None` (FAILED) with distinct messages
- [x] Specify the test seams (`SCRIPT_DIR`, `get_overlay_repos`, `tmp_path` repo dirs, `sync_gitbug` call counter, `pytest.raises(SystemExit)`) and split `sync_repo()` classification tests from `main()` accumulation tests
- [x] Correct step 2's "behavior tightening" framing — git-bug already skips dirty/detached repos today
- [x] Give the unresolvable-path failure a distinct reason string in the summary
- [x] Adopt the `merge_pr.sh` handling with `|| sync_status=$?`; keep `SyncOutcome` in `sync_repos.py`

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-08-24 13:43 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-609 at `41e708c`
**Mode**: pre-push
**Depth**: Deep (reason: 200+ changed lines; `AGENTS.md` governance override trigger)
**Must-fix**: 5 | **Suggestions**: 9
**Round**: 1 | **Ship**: continue — two independently-reproduced false-signal defects (one green, one red) sit on either side of the signal this PR exists to make trustworthy

**Specialists**: static analysis (pre-commit: shellcheck/black/flake8/pylint all pass), governance, plan drift, Claude Adversarial Lens A + Lens B. Copilot off (default). **Local Adversarial skipped: the Ollama server is up on this host but has no models pulled (`/api/tags` -> `{"models":[]}`), so `qwen3.5:35b` was unavailable** — the quota-free cross-model read did not happen on this diff.

**Verification performed**: full `run_script_tests.sh` green (21 shell, 87 pytest). 11 mutations run against a scratch copy — all eight core-classification regressions (pull/fetch fall-through, `None` collapsed into the detached-HEAD skip, dirty->FAILED, detached->FAILED, unresolvable path back to a silent skip, FAILED tallied as skipped, unconditional git-bug, removed `sys.exit(1)`) each broke a named test. Three mutations passed untouched (M9/M10/M11, below). The `is_dirty` false-green and the worktree all-clear were reproduced live, not inferred.

### Findings
- [x] (must-fix) Root workspace repo's outcome is untested in every `main()` test — `outcomes.get()` never keys on `"ros2_agent_workspace"`; mutation M9 (root call site reverted to the `if sync_repo(...)` truthiness trap) and M10 (root FAILED demoted to SKIPPED) both leave all 14 tests passing. It is the repo `merge_pr.sh` just merged into — `.agent/scripts/tests/test_sync_repos.py:163`
- [x] (must-fix) `is_dirty()` returns `success and bool(output)`, so a failed `git status` reads as "clean" — reproduced: corrupt `.git/index` plus real uncommitted changes on a feature branch classifies SYNCED, a false green of the exact class #609 removes, and contradicts the `SyncOutcome` docstring's "cannot even read" claim — `.agent/scripts/sync_repos.py:111,165`
- [x] (must-fix) `exit "$sync_status"` propagates make's 2, already this script's usage-error code (asserted by `test_merge_pr.sh`); rc=2 read as "invoked wrong" invites retrying an irreversible merge. Use a reserved code, and add the contract to `merge_pr.sh`'s own `AGENTS.md` row — `.agent/scripts/merge_pr.sh:395`, `AGENTS.md:572`
- [x] (must-fix) `path not resolved` -> FAILED makes `make sync` permanently red on a supported host config: `optional_layers.txt` lists `site`, and `setup_layers.sh:349-360` `rm -rf`s an optional layer that fails import and exits 0. Also fresh clones and the vcs-missing path (all 35 repos). This is the false-red the design names as worse than the false green — discriminate layer-absent (SKIPPED) from repo-missing-inside-imported-layer (FAILED) — `.agent/scripts/sync_repos.py:346-354`
- [x] (must-fix) An empty repo list reports a quantified all-clear: run from a workspace worktree (no `configs/manifest`), the new summary prints `✅ Sync complete — 1 synced, 0 skipped, 0 failures.` while 35 configured repos were never enumerated — a stronger false claim than the old bare success line — `.agent/scripts/sync_repos.py:294,370`
- [x] (suggestion) Failure summary records the literal `"sync failed"` for every cause, against `plan.md` step 4's "reuse the captured `output`" and inconsistent with line 353, which does give a specific reason — fix or record the decision — `.agent/scripts/sync_repos.py:314`
- [x] (suggestion) `test_outcomes_are_all_truthy` asserts a Python language invariant, not a property of this code — M9 shows it passes with the truthiness trap reintroduced — `.agent/scripts/tests/test_sync_repos.py:118`
- [x] (suggestion) Success-summary counts unasserted — mutation M11 (dropping the synced/skipped counts) passes all 14 tests; both tests only match the substring `"0 failures"` — `.agent/scripts/tests/test_sync_repos.py:180,209`
- [x] (suggestion) `git status -sb` failing still prints `✅ Fetched.` — Plan Review finding 8 asked for a fix or an out-of-scope declaration; neither happened — `.agent/scripts/sync_repos.py:218-226`
- [x] (suggestion) `run_git_cmd` catches only `CalledProcessError`, so an unreadable repo dir raises `PermissionError` and kills the run mid-way with no summary; still loud rather than false-green, but the new docstring overclaims — `.agent/scripts/sync_repos.py:90-96`
- [x] (suggestion) Failure banner writes to stdout while every other failure block in the script uses stderr — `.agent/scripts/merge_pr.sh:388-394`
- [x] (suggestion) Plan-sync: the `preflight_repo()` extraction and its pylint-return-limit rationale are absent from "Review-driven corrections", and the `test_merge_pr.sh` consequences row still reads "open question" — independently confirmed that file only exercises pre-merge guards and never reaches the sync tail, so `bash -n`/shellcheck is the realistic coverage; close the row with that — `.agent/work-plans/issue-609/plan.md`
- [x] (suggestion) git-bug failures warn but are never tallied (sync stays green) with the ADR-0010 graceful-degradation rationale recorded nowhere; `AGENTS.md:571` also omits two FAILED causes (non-existent path, unreadable git state) — `.agent/scripts/sync_repos.py:265-267`, `AGENTS.md:571`
- [x] (suggestion) A plain offline error (`Could not resolve hostname`) is not a retried transient signature, so an off-network host FAILs every repo; dev-side only today, but `transient_error_seen()` already carries the signal to report "remote unreachable" distinctly — `.agent/scripts/sync_repos.py:207,229`

### Clean
- Static analysis: `pre-commit` on all 4 changed files — shellcheck, black, flake8, pylint, identity and branch hooks all pass.
- Caller audit re-verified independently (not taken on trust from the Plan Review): `Makefile:221` and `merge_pr.sh:379` are the only invokers. No CI workflow, skill, hook, dashboard (it uses `vcs` directly), container or dispatch script consumes it, and no Makefile target lists `sync` as a prerequisite.
- `sync_status=0; make ... || sync_status=$?` is correct under `set -eo pipefail`, and no bannerless abort is reachable after the irreversible merge — every command from line 367 on is `|| true`-protected or captured. Only the exit *value* is wrong (must-fix 3).
- Tri-state classification of the four core cases is sound and genuinely tested; `test_net_retry.py` is unaffected (asserts nothing on `sync_repo`'s return).
- Commit hygiene: 8 commits, correct agent identity, atomic, no issue-closing keywords.
- Both settled operator decisions honoured: `merge_pr.sh` handling is in this PR, and `SyncOutcome` stays in `sync_repos.py`.

## Implementation
**Status**: complete
**When**: 2026-08-24 14:01 -04:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-609 at `a5ad33d`
**Addressed**: `## Local Review (Pre-Push)` (2026-08-24 13:43 -04:00, branch at `41e708c`) — Round 1, 5 must-fix + 9 suggestions
**Commits**: 6e41984 3afbed7 178b63f e7eb6ea b6895d2 48ef21b fb1644a 6198735 e8ffae7 a5ad33d 

All 14 findings actioned; none deferred. The two false-signal defects on
either side of this PR's own signal were fixed first, and each fix was
mutation-checked rather than assumed.

### Actions
- [x] (must-fix) Root workspace repo's outcome untested in every `main()` test — two tests now pin its FAILED and SKIPPED paths (including that git-bug does not run after its failed pull); `test_outcomes_are_all_truthy` removed as a language invariant that survived the same mutation — `.agent/scripts/tests/test_sync_repos.py`
- [x] (must-fix) `is_dirty()` false green — now returns True/False/**None**, and an unreadable working tree is FAILED, not clean; `run_git_cmd()` also catches `OSError` so an unreadable repo dir is a per-repo failure instead of an un-summarised crash — `.agent/scripts/sync_repos.py:106-124,205-215`
- [x] (must-fix) `exit "$sync_status"` propagated make's 2 (the usage-error code) — reserved `SYNC_FAILED_RC=3` for "merged and cleaned up, sync failed", all four exit codes documented in the script header, and `AGENTS.md`'s `merge_pr.sh` row now carries the contract — `.agent/scripts/merge_pr.sh`, `AGENTS.md`
- [x] (must-fix) `path not resolved` → FAILED made `make sync` permanently red where an optional layer is legitimately absent — split into optional-layer-not-set-up (SKIPPED), required-layer-not-set-up (FAILED, naming `setup_layers.sh`), and repo-missing-inside-an-imported-layer (FAILED). `optional_layers.txt` parsing moved to `lib/workspace.py` and shared with `validate_workspace.py` — `.agent/scripts/sync_repos.py`, `.agent/scripts/lib/workspace.py`
- [x] (must-fix) Empty repo list reported a quantified all-clear — no enumerable repos is now a named failure (`configs/manifest: no repositories configured`) — `.agent/scripts/sync_repos.py`
- [x] (suggestion) Generic `"sync failed"` reason — `sync_repo()` returns `SyncResult(outcome, reason)`; every failure path supplies its own cause and the summary prints it — `.agent/scripts/sync_repos.py`
- [x] (suggestion) `test_outcomes_are_all_truthy` asserted a language invariant — removed; the root call-site tests guard the trap instead
- [x] (suggestion) Success-summary counts unasserted (M11) — both success tests now assert the full summary line
- [x] (suggestion) `git status -sb` failing still printed `✅ Fetched.` — **settled by fixing**: it now says `✅ Fetched (could not read ahead/behind status).` and stays SYNCED, since the fetch itself succeeded
- [x] (suggestion) `run_git_cmd` caught only `CalledProcessError` — `OSError` is now reported rather than killing the run mid-way
- [x] (suggestion) Failure banner on stdout — moved to stderr, with the leading blank/rule lines inside the same redirected block
- [x] (suggestion) Plan-sync — `preflight_repo()` extraction + its pylint rationale recorded, a Local-Review corrections section added, and the `test_merge_pr.sh` consequences row **closed** with the reason it cannot reach the sync tail — `.agent/work-plans/issue-609/plan.md`
- [x] (suggestion) git-bug failures never tallied — ADR-0010's graceful-degradation rationale now recorded at `sync_gitbug()`; the `AGENTS.md` row lists every FAILED and SKIPPED cause — `.agent/scripts/sync_repos.py`, `AGENTS.md`
- [x] (suggestion) Plain offline error not a retried transient signature — an off-network host now reports `remote unreachable (pull): …` instead of 35 identical pull failures — `.agent/scripts/sync_repos.py`

### Verification
- `run_script_tests.sh` green: 21 shell tests, 98 pytest (25 in `test_sync_repos.py`, up from 14).
- `pre-commit` clean on every changed file (shellcheck, black, flake8, pylint) — **no lint rule suppressed**: `sync_repo()`'s pre-sync guards stay in `preflight_repo()`, and the test stub became a probe table with one `fails` set to stay under the argument/return limits.
- **The three mutations that survived Round 1 now fail**: root call site reverted to `if sync_repo(...)`; root FAILED demoted to SKIPPED; synced/skipped counts dropped from the success summary. Each new behaviour was also mutation-checked in both directions — `is_dirty()` back to `success and bool(output)`, and both an over- and under-permissive optional-layer rule, each break a named test.
- **Live check of the false red**: a dry run of this branch's code against the host's real 35 repos (via a mirrored root) reports `35 synced`; removing `site_ws` from that mirror yields `34 synced, 1 skipped` with `optional layer 'site' is not set up` and no failure. A dry run from a workspace worktree now exits 1 saying no repos could be enumerated, instead of `1 synced … 0 failures`.

### Deferred actions
None — every finding was actioned.

### Notes for the re-review
- `sync_repo()`'s return type changed from `SyncOutcome` to `SyncResult(outcome, reason)`; both are truthy, so the truthiness trap is unchanged in kind — the enum docstring says so and the call-site tests now enforce it.
- `make sync` run from a *workspace worktree* (no `configs/manifest`) now exits 1 by design. `merge_pr.sh` is unaffected: its `ROOT_DIR` is the main worktree, so the manifest is always present there.
- Observed but **not** actioned (outside this issue's scope, no finding): `validate_workspace.py` prints `✅ Workspace validation PASSED` with 0 configured repos in the same no-manifest situation — the same false-all-clear class, in a different script.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-08-24 14:13 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-609 at `4207766`
**Mode**: pre-push
**Depth**: Deep (reason: 860+ changed source lines; `AGENTS.md` governance override trigger)
**Must-fix**: 4 | **Suggestions**: 5
**Round**: 2 | **Ship**: recommended — the classification core is now correct and well-tested; all four must-fixes are precise mechanical corrections (three doc/contract one-liners in two files, one two-line classification change plus a test flip), none reopens a design question

**Specialists**: static analysis (`pre-commit` on all 7 changed files: shellcheck, black, flake8, pylint, identity/branch hooks — all pass), governance, plan drift, Claude Adversarial Lens A + Lens B. Copilot off (default). **Local Adversarial skipped: Ollama answers on this host but has no models pulled (`/api/tags` -> `{"models":[]}`), so `qwen3.5:35b` was unavailable** — the quota-free cross-model read did not happen on this diff, for the second round running.

**Verification performed**: full `make test-scripts` green (21 shell, 98 pytest; `test_sync_repos.py` 25, up from 14). **22 mutations run independently rather than trusting the fix pass's report — 19 killed, 3 survived.** All three Round-1 survivors are now killed (root call site reverted to the `if sync_repo(...)` truthiness trap; root FAILED demoted; summary counts dropped), and each new behaviour is killed in both directions (`is_dirty` back to `success and bool(output)`; over- and under-permissive optional-layer rules; the `OSError` catch; the offline signature; the unreadable `status -sb` line; the empty-repos failure; per-cause reasons). The three survivors are listed as suggestions below. Both false-signal fixes re-verified live, not inferred: `make sync` from this worktree exits 1 with the named failure, and all 35 configured repos locate correctly on this host (no false red here today). **All four must-fixes were independently reproduced by both adversarial lenses and by the lead — no single-source findings among them.**

### Findings
- [x] (must-fix) The new hard-failure path's only remediation instruction names a Makefile target that does not exist: it prints ``Run `make setup``` but the target is `setup-all` (`make -n setup` → "No rule to make target 'setup'"). This is the path an operator hits on an un-bootstrapped clone or from a worktree, so the printed advice produces a second, more confusing error — `.agent/scripts/sync_repos.py:447`
- [x] (must-fix) The newly documented exit-code contract is factually wrong about the code: both the header block and the `AGENTS.md` row state `1 = failed before the merge, nothing irreversible happened, safe to retry`, but the worktree-removal failure exits 1 *after* `gh pr merge` has already landed (a `set -e` abort on `cd "$ROOT_DIR"` does the same). An agent reading the table rather than the banner retries a completed, irreversible merge — the exact misread code 3 was reserved to prevent, left uncovered on the sibling path. Reword 1, or reserve a fourth code for post-merge cleanup failure — `.agent/scripts/merge_pr.sh:26-33,371`, `AGENTS.md:572`
- [x] (must-fix) Exit code 3 is invisible through the entry point `AGENTS.md` advertises: GNU make collapses any non-zero recipe status to its own 2 (verified empirically — a recipe of `exit 3` yields make rc=2), so `make merge-pr` surfaces the post-merge-sync failure as exactly the usage-error code the rationale comment says must be avoided. The same flattening hits the other half of this change: `AGENTS.md` documents `FAILED (exits 1)` for sync, but `make sync` returns 2. Document the flattening in both rows and steer exit-code-sensitive callers to the scripts directly — `Makefile:222,229`, `.agent/scripts/merge_pr.sh:31-33`, `AGENTS.md:571-572`
- [x] (must-fix) The optional-layer carve-out is gated on `layer_src.exists()`, which is true in two states `setup_layers.sh` exits 0 on: a *partially* imported optional layer (it deletes the directory only when `LAYER_DIR_EXISTED=false` — "preserve a previously-cloned layer"), and a host with no `vcs` (it `mkdir -p`s an empty `src` and prints "Setup complete"). In both, an inaccessible optional repo falls through to FAILED "path not resolved", so `make sync` goes permanently red on a supported host — and, via the new wiring, every merge then ends at exit 3. That is the false red the docstring itself calls worse than the false green being removed, and it makes this script disagree with `validate_workspace.py`, which treats any repo in an optional layer as allowed-missing regardless of layer presence — while now sharing the same parser. Skip a missing repo whose layer is optional regardless of `layer_imported`, flip `test_repo_missing_inside_an_imported_optional_layer_is_a_failure`, and correct the docstring's one-branch claim about `setup_layers.sh`. (Recording the divergence deliberately also closes this, but it cannot stay silent.) — `.agent/scripts/sync_repos.py:312-344`
- [x] (suggestion) `.agent/hooks/post-checkout` and `.agent/hooks/README.md` promise that a workspace worktree "can use `make sync` ... as usual" / "Enables `make sync` in worktrees". `configs/manifest` is gitignored and untracked, so a worktree can never enumerate repos — the claim was already false (sync silently did nothing) and this change makes the command a guaranteed exit 1. A change carries its consequences: fix the two lines — `.agent/hooks/post-checkout:75`, `.agent/hooks/README.md:14`
- [x] (suggestion) Mutation survivor: deleting `locate_repo`'s explicit-`path` fallback leaves all 25 tests green. Pre-existing behaviour, but this PR extracted it into a testable unit and did not test it — `.agent/scripts/sync_repos.py:299-306`
- [x] (suggestion) Mutation survivors: neither the comment/blank-line stripping nor the missing-file→`set()` path of the new shared `get_optional_layers` is covered — dropping the `#` strip, or returning a non-empty set when the file is absent, both leave all 25 tests green. The "byte-compatible with `is_optional_layer`" claim is now load-bearing for two callers with zero assertions behind it; one `== {"site"}` assertion closes it — `.agent/scripts/lib/workspace.py:80-101`
- [x] (suggestion) Plan drift: `.agent/scripts/lib/workspace.py` and `.agent/scripts/validate_workspace.py` both changed but appear nowhere in `plan.md` — not in "Files to Change", not in the corrections narrative. Copilot flags exactly this — `.agent/work-plans/issue-609/plan.md:150`
- [x] (suggestion) Follow-up scope, deliberately NOT expanded into this PR: `pull_remote.py:42`'s `is_dirty` carries the identical `success and bool(output)` false green fixed here — and then *merges* on it — alongside the `validate_workspace.py` 0-repo all-clear the fix pass already recorded. Same defect class, different entry points. File one follow-up issue covering both rather than widening a diff that has had two review rounds (deferred: filing is the host orchestrator's step — this sub-agent has no GitHub write role, and the handoff explicitly reserved `pull_remote.py`/`validate_workspace.py` for that follow-up)

### Clean
- **Classification core**: both adversarial lenses independently reported the tri-state `SyncOutcome`, the `is_dirty` None/False/True split, the `""`-vs-`None` branch split, the `OSError` catch, and the root-repo truthiness-trap avoidance as sound. My mutation run agrees — every core regression is killed by a named test.
- **False-green fix, caller audit**: `is_dirty`/`get_current_branch` are module-local to `sync_repos.py`; the only caller is `preflight_repo`, which tests `is None` before truthiness. No caller treats the new `None` as falsy-clean. `retry_transient` is unaffected — the `OSError` message cannot match a `TRANSIENT_ERRORS` signature, so no spurious retry.
- **Shared-parser byte-compatibility**: `get_optional_layers` matches `setup_layers.sh:is_optional_layer` exactly (strip from first `#`, trim, skip blanks); the `workspace_root=None` default preserves `validate_workspace.py`'s positional call. A *required* repo that cannot be located still fails, as the design requires.
- **The worktree exit 1 is right, not a trap** (item 3 of the review brief): `configs/manifest` is gitignored, so a worktree could never enumerate repos — the old code silently synced the root repo alone and printed success. The change converts a silent no-op into a named failure with an actionable remedy. `merge_pr.sh` is genuinely unaffected: its `ROOT_DIR` is the first entry of `git worktree list --porcelain` (always the main worktree) and it calls `make -C "$ROOT_DIR" sync`. Claim verified.
- **Exit-code 3 collision**: unused elsewhere in `merge_pr.sh`, not shell-reserved (126/127/128+n), and no script or skill inspects the status. The only collision is the make-wrapper flattening (must-fix 3).
- **Caller audit**: `Makefile:222` and `merge_pr.sh:393` remain the only invokers of sync. No CI workflow, skill, hook, dashboard (it uses `vcs` directly), or dispatch script consumes it; no Makefile target depends on `sync`.
- Static analysis clean; ADR-0010's "failures warn but don't block" verified against the ADR text; git-bug gating matches pre-change behaviour. Commit hygiene: 20 commits, single correct agent identity, atomic, no issue-closing keywords. Both settled operator decisions still honoured.

## Implementation
**Status**: complete
**When**: 2026-08-24 14:25 -04:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-609 at `bb282e8`
**Addressed**: `## Local Review (Pre-Push)` Round 2 (2026-08-24 14:13 -04:00, branch at `4207766`) — 4 must-fix, 5 suggestions
**Commits**: `674736a`, `bdce0c5`, `7cf6af9`, `dc92ad1`, `d7463d1`, `9d32836`, `eef42d4`, `7f832ca`, `bb282e8`

### Actions
- [x] (must-fix) The no-repos remedy named a nonexistent Makefile target — corrected to `make setup-all`, verified against the Makefile (`make -n setup` → "No rule to make target 'setup'"; `setup-all` is the real `.PHONY` target). No other `make setup` reference exists in the changed surface — `.agent/scripts/sync_repos.py:447` (`674736a`)
- [x] (must-fix) The exit-code contract now matches the code rather than being reworded around it: `SYNC_FAILED_RC` → `POST_MERGE_RC`, and **every** post-merge failure exits 3 — the worktree-removal refusal (was 1), the post-merge `make sync`, and the `cd "$ROOT_DIR"` that `set -e` would have aborted bannerless with 1. The `cd` gained an explicit failure branch printing the same "PR is already MERGED" cleanup instructions. Code 1 is now truthfully "failed **before** the merge". Two new `test_merge_pr.sh` assertions pin it: no `exit 1` anywhere after the `gh pr merge` line, and the `cd` is guarded — both mutation-checked — `.agent/scripts/merge_pr.sh:26-40,54,358-370,391`, `AGENTS.md:572` (`7cf6af9`)
- [x] (must-fix) The make flattening is documented where it bites: `AGENTS.md`'s sync row now reads "the script exits 1; `make sync` reports make's own 2, since GNU make flattens any non-zero recipe status", the merge row states that through `make merge-pr` none of the codes survive, and both steer exit-code-sensitive callers to the scripts directly. The script header carries the same note. Verified empirically on this host: a recipe of `exit 3` yields make rc=2 — `AGENTS.md:571-572`, `.agent/scripts/merge_pr.sh:26-40` (`7cf6af9`, `dc92ad1`, `9d32836`)
- [x] (must-fix) The optional-layer carve-out is no longer gated on `layer_src.exists()` — a missing repo whose layer is optional is a SKIP, full stop. The docstring's one-branch claim about `setup_layers.sh` is replaced with both real exit-0 states (partial import preserved via `LAYER_DIR_EXISTED=true`; a host with no `vcs` leaving an empty `src` and printing "Setup complete") and with why the directory therefore cannot distinguish them, plus the `validate_workspace.py` agreement this now restores. The skip message was reworded — "optional layer 'X' is not set up" was itself false in the partial case — to "not present — layer 'X' is optional on this host". `test_repo_missing_inside_an_imported_optional_layer_is_a_failure` is flipped to `test_repo_missing_from_a_present_optional_layer_is_still_a_skip`; a would-be duplicate required-layer test was dropped because `test_unresolvable_path_is_a_failure` already covers it — `.agent/scripts/sync_repos.py:312-352` (`bdce0c5`)
- [x] (suggestion) `.agent/hooks/post-checkout` and `.agent/hooks/README.md` no longer promise `make sync` works in a workspace worktree; both now name the cause (`configs/manifest` is gitignored, so no repo can be enumerated) and the remedy (sync from the main tree). Swept the rest of the repo for the same claim — `WORKTREE_GUIDE.md` and the ADRs make it nowhere else — `.agent/hooks/post-checkout:75-78`, `.agent/hooks/README.md:14-17` (`d7463d1`)
- [x] (suggestion) Mutation survivor closed: `locate_repo`'s explicit-`path` fallback now has four tests — layer path preferred, relative fallback, absolute fallback (not re-rooted), and every tried path reported. Deleting the fallback fails three of them — `.agent/scripts/tests/test_sync_repos.py` (`7f832ca`)
- [x] (suggestion) Mutation survivors closed: new `.agent/scripts/tests/test_workspace_lib.py` pins `get_optional_layers`'s comment/blank/whitespace stripping, its missing-file `set()`, its empty-file `set()`, and — rather than restating the rule — its agreement with the real `is_optional_layer()` function extracted verbatim out of `setup_layers.sh` and run under bash. Dropping the `#` strip, dropping the blank skip, or returning a non-empty set for a missing file each fail a named test — `.agent/scripts/lib/workspace.py:80-101` (`eef42d4`)
- [x] (suggestion) Plan drift closed: `plan.md`'s "Files to Change" now lists `lib/workspace.py`, `validate_workspace.py`, and `test_workspace_lib.py` with why each was added mid-implementation; a "Review-driven corrections (Round 2)" section records all four must-fixes and the suggestions; the two Round-1 statements the Round-2 change superseded are annotated rather than left standing; Estimated Scope updated — `.agent/work-plans/issue-609/plan.md` (`bb282e8`)
- [x] (suggestion) The `pull_remote.py:42` / `validate_workspace.py` follow-up (deferred: filing is the host orchestrator's step — this sub-agent has no GitHub write role, and the dispatch explicitly reserved both files for that separate issue to keep this PR's scope where it is). Neither file was touched.

### Verification
- `make test-scripts` green: 21 shell suites (`test_merge_pr.sh` 20/20) and **106 pytest cases**, up from 98 — `test_sync_repos.py` 25 → 29, plus 4 in the new `test_workspace_lib.py`.
- **Every new behaviour mutation-checked in both directions**, not trusted: re-gating the optional-layer carve-out on the directory *and* skipping unconditionally each break a named test; reverting the post-merge exit to 1 and unguarding the `cd` each break a named `test_merge_pr.sh` assertion; dropping the parser's `#`-strip, dropping its blank-skip, returning a non-empty set for a missing file, and deleting `locate_repo`'s explicit-path fallback each break a named test. One equivalent mutant noted and *not* papered over with a test: dropping `locate_repo`'s `is_absolute()` guard changes nothing, because `Path.__truediv__` with an absolute right operand already returns that operand.
- **Live, not inferred** — a mirrored root over this host's real 35 repos and real `configs/manifest`: `35 synced, 0 skipped` with `site_ws` intact; with `site_ws/src` present-but-empty (the no-`vcs` / partial-import state that used to go red) `34 synced, 1 skipped`; with `site_ws` removed entirely, likewise `34 synced, 1 skipped`. Both states now classify benignly and identically. From this worktree, `sync_repos.py --dry-run` still exits 1 with the named `configs/manifest` failure and now prints the working `make setup-all` remedy.
- `pre-commit` clean on all ten changed files (shellcheck, black, flake8, pylint, identity/branch hooks); `bash -n` clean on `merge_pr.sh` and `post-checkout`. No lint rule suppressed — the one black reformat was accepted rather than worked around.

## Implementation
**Status**: complete
**When**: 2026-08-24 14:43 -04:00
**By**: Claude Code Agent (Claude Opus 5)

**Branch**: feature/issue-609 at `b3f6d2f`
**Scope**: widened past the previous `## Implementation` entry's last line, which recorded the `pull_remote.py` / `validate_workspace.py` siblings as deferred to a follow-up issue and both files untouched. **The operator overruled that deferral**, on AGENTS.md's filing discipline ("bundle related changes into one PR, atomic commits inside, rather than fanning out many small issues") and the Quality Standard ("fix it completely… never leave a 'good enough' fix"). That line is superseded, not withdrawn — `plan.md` is corrected inline.
**Commits**: `4fe0cf2`, `35a3e1b`, `f3c2fae`, `b3f6d2f` — one per script, so any single widening commit can be dropped without unpicking the rest.

### What changed

- [x] **`pull_remote.py` — a failed git probe is an error, not a benign answer** (`4fe0cf2`). `is_dirty()` was `success and bool(output)`, so a failed `git status` read as "clean"; this is the script that then **merges** on that answer. `get_current_branch()` was `output if success and output else None`, collapsing a genuine detached HEAD with a failed git command — the identical bug #609 split apart in `sync_repos.py`, split the same way here. Both are now tri-state (`True/False/None`, `name/""/None`). `_check_pull_preconditions()` returns a `(status, message)` **problem** instead of a bare skip reason, so an unreadable tree or git state classifies `error` (exit 1 via the existing `print_summary_and_exit`) while dirty / detached / wrong-branch stay `skip` (exit 0). The existing `sys.exit(1 if errors else 0)` in the `--json` arm and the `results["error"]` accumulation in `iter_repos` were already correct and are unchanged — the fix routes into them rather than adding a second mechanism.
- [x] **Neighbouring claim in the same file**: `_fetch_into_branch()` read the current branch to decide whether `git branch -f` is safe. With the probe failing it fell through on `None != target_branch` and force-updated a branch it could not prove was not checked out. Now an explicit error.
- [x] **`validate_workspace.py` — an empty configuration is not a pass** (`35a3e1b`). `not (missing or extra or version_mismatches)` is vacuously true over zero configured repos, so an un-bootstrapped clone or a workspace worktree printed `✅ Workspace validation PASSED!` and exited 0. The verdict is now a three-state `ValidationResult` with an explicit `EXIT_CODES` table: `UNCONFIGURED` exits **3**, not 1 (it is not drift, and `--fix` cannot help — there are no `.repos` files to import from, so the message points at `make setup-all`, mirroring `sync_repos.py`'s handling of the identical state) and not 2 (argparse owns 2 — the same collision Round 2 caught in `merge_pr.sh`).
- [x] **Neighbouring claim in the same `main()`**: a successful `--fix` re-validated the workspace and then **discarded** the answer, exiting 1 on the stale result — a workspace `--fix` had just repaired still read red. The re-validation's result is now taken.
- [x] **`dashboard.sh:166`, the consumer** (`35a3e1b`). It ran the script under `&>/dev/null` and branched pass/fail, so the new outcome would have printed "Workspace drift detected. Run: make validate" — sending the operator back to the same empty answer. It now reads the code three ways (0 match / 3 "No repos configured — nothing validated. Run: make setup-all" / anything else drift). `dashboard.sh` has no `set -e`, so capturing `$?` is safe.
- [x] **`lib/remote_utils.py` + `push_remote.py` — a fourth site, found while re-verifying `process_repo`'s neighbouring claims** (`f3c2fae`). `remote_exists()` returned False when `git remote` itself failed, so both `pull_remote.py` and `push_remote.py` reported "remote 'gitcloud' not found" and skipped — green — a repo they had never been able to read. Now `remote_probe()` → `RemoteState.PRESENT/ABSENT/UNREADABLE`; `remote_exists()` is kept as the boolean wrapper for `add_remote.py`, its one remaining caller, where False is the *right* answer (it goes on to read the origin URL and reports an explicit error when that fails). Separately, `run_git()` caught only `CalledProcessError`, so an unusable repo directory raised `OSError`, escaped, and killed the whole run with no summary — the same hole `sync_repos.py`'s `run_git_cmd` closed on this branch, and the thing that makes `is_dirty()`'s new `None` reachable for that case. `pull_remote.py`'s `--json` arm carried the same false skip and now records it as an error.
- [x] **Docs** (`4fe0cf2`, `35a3e1b`, `f3c2fae`, `b3f6d2f`): AGENTS.md exit-code rows for `pull_remote.py`, `push_remote.py`, `validate_workspace.py`; module docstrings carrying the same contract; the `make validate` help line; and `plan.md` corrected **inline** — the Round-2 "Deliberately not widened" bullet now records the overrule, and a "Scope widened after Round 2" section names the four sites, the three consequences, and what was left alone.

### Verification

- `run_script_tests.sh` green: **22 shell suites** (up from 21 — the new `test_dashboard_validate_status.sh`) and **152 pytest cases** (up from 106): `test_pull_remote.py` 24, `test_validate_workspace.py` 14, `test_remote_probe.py` 8, existing suites unchanged.
- `pre-commit` clean on every changed file. **No lint rule suppressed anywhere.** Three pylint findings were fixed by restructuring, following the precedent set by `preflight_repo` and the plain-helper convention in `test_sync_repos.py`: the `protected-access` cluster was removed by driving the classification tests through `process_repo()` — the entry point `iter_repos` actually calls, and a better test for it — rather than the module-private precondition helper; `too-few-public-methods` by using `SimpleNamespace` for the stub argparse namespace; `import-outside-toplevel` by hoisting the imports.
- **Every new behaviour mutation-checked in both directions — 29 mutations, all killed.**
  - `pull_remote.py` (10): `is_dirty` reverted to `success and bool(output)`; failed-status→clean; failed-status→dirty; `get_current_branch` reverted to its original expression; failed-git→`""`; unreadable tree demoted to skip; unreadable state demoted to skip; the detached-HEAD skip branch deleted; `_fetch_into_branch`'s `None` guard deleted; the precondition problem collapsed back to a skip.
  - `validate_workspace.py` + `dashboard.sh` (10): the no-repos early return deleted; `UNCONFIGURED` demoted to `PASSED`; its exit code renumbered to 0; renumbered to argparse's 2; the `--fix` gate widened to any non-`PASSED` result; the post-fix re-validation result discarded again; all drift classified `PASSED`; the optional-layer carve-out removed; the dashboard reverted to a two-way branch; the dashboard treating 3 as a pass.
  - `lib/remote_utils.py` + `push_remote.py` (9): `UNREADABLE` collapsed into `ABSENT`; into `PRESENT`; the `OSError` arm deleted; the `OSError` arm swallowing the `CalledProcessError` arm; `remote_exists` loosened to `is not ABSENT`; the unreadable case demoted to a skip in `pull_remote`, in `push_remote`, and in the `--json` arm; an absent remote wrongly counted as an error.
  - One mutation initially **survived** and was closed rather than accepted: widening the `--fix` gate from `is DRIFTED` to `is not PASSED` is behaviour-equivalent today, because `UNCONFIGURED` always returns an empty `missing` list. A test now drives the outcome directly to pin the *intent* (`--fix` imports from `.repos` files; `UNCONFIGURED` means there are none), with a docstring saying why the state is synthetic.
- **Live, not inferred.** From this worktree, `validate_workspace.py` exits **3** with the `make setup-all` remedy, and the real `dashboard.sh --quick` prints "No repos configured — nothing validated. Run: make setup-all". Through a mirrored root over this host's real `configs/manifest` and `layers/` (44 configured / 45 actual repos) it exits **1** and the dashboard prints "Workspace drift detected" — the host's pre-existing orphan `image_warper`, unchanged by this work. Both branches of the new code confirmed on a real configuration.
- **Consumer audit** for the new exit code: `dashboard.sh:166` and `Makefile:142` are the only invokers of `validate_workspace.py`. **No CI workflow references it** — verified against `.github/workflows/` — so there is no hosted-gate exposure; `make validate` is the documented local gate, and (like `make sync`) GNU make flattens any non-zero recipe status to its own 2, which the AGENTS.md rows and module docstrings now say.

### Deliberately not touched

- `lib/remote_utils.py:119` and `:124` (now `:137`/`:141`) — the default-branch fallback chain. Each probe falls through to the next and finally to `"main"`; a failed probe is not a false answer there, it is a step in a designed sequence. Re-verified against the source, not taken on trust.
- `pull_remote.py`'s `_compare_branches()` new-commit log listing — a failed `git log` only omits detail from a message that has already said the branch is behind.
- `pull_remote.py`'s exit-code handling (`sys.exit(1 if errors else 0)` and `iter_repos`'s error accumulation) — already correct; the fix routes into it rather than replacing it.
- `add_remote.py` keeps `remote_exists()`. Its `if remote_exists(...)` means "skip, already added", so False for an unreadable repo sends it down the path that reads the origin URL and reports an explicit error — the correct outcome already.
- `sync_repos.py`, `merge_pr.sh`, `lib/workspace.py` — untouched this round; their Round-2 state stands.

### Open for the reviewer

- The **fourth site** (`lib/remote_utils.py` + `push_remote.py`, commit `f3c2fae`) was not in the dispatch brief's three-site list; it was found by re-verifying the neighbouring claims in `process_repo()`, the function the first site feeds. It is the same defect class and it is fixed and tested, but it widens the diff into a fourth and fifth script. It is a single atomic commit and can be dropped on its own if the operator wants the scope held to the three named sites — `pull_remote.py` would then need its `remote_probe` import reverted to `remote_exists`.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-08-24 15:05 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-609 at `a5ddac6`
**Mode**: pre-push
**Depth**: Deep (reason: 21 files / +2861 −108 changing exit-code contracts consumed by other scripts, Makefile targets, hooks and skills)
**Must-fix**: 4 | **Suggestions**: 12
**Round**: 3 | **Ship**: continue — first review of the operator-widened scope; it surfaced a silent-data-loss path feeding `/import-field-changes`, must-fix count is flat at 4 (Round 2 also 4), and two of the four are not mechanical (one is an operator scope decision)

**Specialists**: Static Analysis (pre-commit clean on the full diff) · Governance · Plan Drift · Claude Adversarial ×2 (Lens A + Lens B, Deep horizon). **Local Adversarial skipped: `qwen3.5:35b` is not pulled on this host** — the Ollama server answers on :11434 but `ollama list` is empty, so `local_review.sh` would exit 2. Copilot Adversarial off (default).

**Lead verification**: full suite green (22 shell suites + 152 pytest); **19 independent mutations against a pristine `git archive` copy, all 19 killed**, including the false-RED direction (promoting dirty / detached-HEAD / wrong-branch to an error each break a named test). The three sites deliberately left alone were re-checked and are correct: `add_remote.py`'s boolean `remote_exists()` falls through to `get_origin_url()` and returns an explicit **error**, not a skip; the default-branch fallback chain is a designed sequence; `_compare_branches`'s log listing only omits detail from a message that already says the branch is behind. GNU make's flattening of any non-zero recipe status to 2 confirmed empirically on this host. `f3c2fae` (the fourth site) **belongs** — without it `process_repo()` still returned "remote not found" for a repo whose `git remote` had failed.

### Findings
- [x] (must-fix) Three new doc lines attribute the script's exit code to a `make` target — the exact false claim this branch disclaims in both new AGENTS.md rows and both new docstrings; `make validate` prints `Error 3` and exits 2 — `Makefile:68`, `.agent/hooks/post-checkout:77`, `.agent/hooks/README.md:16`
- [x] (must-fix) `make validate` no longer reaches `test_layer_sourcing.sh` wherever `configs/manifest` is absent (every workspace worktree, every un-bootstrapped clone): `UNCONFIGURED`→3 aborts the two-line recipe, silently retiring ADR-0016's named enforcement path — `Makefile:141-143`, claimed by `AGENTS.md:582` and `docs/decisions/0016-*.md:56,81`
- [x] (must-fix) `--json` still drops a repo on a failed probe and exits 0: `_json_report` collapses "remote branch absent" with "`rev-list` failed" into `None` and appends no error; reproduced (remote-tracking-only default branch → rc 128), and `/import-field-changes` reads the gap as "No field changes to import" — `.agent/scripts/pull_remote.py:315-345`
- [x] (must-fix) The widening stopped at the per-repo probes; the enumeration layer beneath still reports green over repos never seen — `pull_remote.py` exits 0 with zero repos enumerated where `sync_repos.py` exits 1 in the identical state; `get_overlay_repos` swallows `yaml.YAMLError`; `missing` never makes the run non-zero. **Operator scope call: fix here or file a follow-up and name it in the PR body** — `.agent/scripts/lib/remote_utils.py:188-238`, `.agent/scripts/lib/workspace.py:60-73`
- [x] (suggestion) Surviving mutation: dropping `extra_repos` from the PASSED expression passes all 152 tests — orphan-only drift is unasserted — `.agent/scripts/validate_workspace.py:327-331`
- [x] (suggestion) Surviving mutation: `remote_name in output.splitlines()` → `in output` passes all tests, and that line is now the sole PRESENT/ABSENT discriminator — `.agent/scripts/lib/remote_utils.py:106`
- [x] (suggestion) Surviving mutation: deleting the sync-failure `exit "$POST_MERGE_RC"` passes the whole suite — restores #609 verbatim — `.agent/scripts/merge_pr.sh:437`, `.agent/scripts/tests/test_merge_pr.sh:170-203`
- [x] (suggestion) A failed `rev-list` reports a bare `ok / fetched`; `sync_repos.py:272-276` fixed the byte-identical line on this branch — `.agent/scripts/pull_remote.py:100-106`
- [x] (suggestion) The `*)` arm reports argparse's 2, a missing `python3` (127) and a traceback as "Workspace drift detected" — `.agent/scripts/dashboard.sh:171-177`
- [x] (suggestion) No `Exit status:` docstring block, though behaviour changed — `.agent/scripts/push_remote.py:1-19`; and none in `sync_repos.py`, the script this issue is about
- [x] (suggestion) `UNREADABLE_STATE` re-typed as a literal so the two classifications can drift; `_push_args()` omits `set_default_branch`, read at line 161 — `.agent/scripts/push_remote.py:132`, `.agent/scripts/tests/test_remote_probe.py:87-90`
- [x] (suggestion) Plan still says "Agent-instruction candidates: none identified"; the pattern now spans four sites in five scripts — propose it as an operator-approved `.agent/knowledge/` candidate — `.agent/work-plans/issue-609/plan.md:206-208`
- [x] (suggestion) "Every post-merge failure path exits 3" overstates — the branch-deletion step ends in `|| true` — `AGENTS.md:572`
- [x] (suggestion) Files-to-Change table omits four changed files and carries two `validate_workspace.py` rows; Plan Review action items still unchecked — `.agent/work-plans/issue-609/plan.md`, `.agent/work-plans/issue-609/progress.md:172-180,207-211`
- [x] (suggestion) Residual same-class instance: both git probes failing plus a 40-hex configured version records no mismatch — `.agent/scripts/validate_workspace.py:284-324`
- [x] (suggestion) `--json` fetches the workspace root without a `remote_probe`, so a root with no such remote errors where non-JSON mode benignly skips — contradicts the new `AGENTS.md:575` contract — `.agent/scripts/pull_remote.py:328-334`
- [x] (suggestion) Unverifiable from here: `classify_unlocatable_repo`'s required-vs-optional split depends on each field host's `optional_layers.txt` (gitcloud). Check the ccomjhc/echoboats manifests before this reaches gabby/salmon

## Implementation
**Status**: complete
**When**: 2026-08-24 15:31 -04:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-609 at `0a8bd3d`
**Addressed**: `## Local Review (Pre-Push)` — Round 3, 2026-08-24 15:05 -04:00, branch at `a5ddac6` (4 must-fix, 13 suggestions)
**Commits**: `f548535` `6ab3621` `ea8e36c` `8b20ecf` `1e7df5c` `e2773c2` `1583a41` `87d2916` `0a8bd3d`

All four must-fixes are fixed here, none deferred — including must-fix 4, which
the operator ruled in scope for this PR rather than a follow-up.

**Verification**: full suite green (23 shell suites + 191 pytest, up from 22 +
152); `pre-commit` clean on every changed file with no rule suppressed. **13
independent mutations, all 13 killed**, and the false-RED direction re-checked
on each of the enumeration changes: making an optional-layer absence red, and
reverting the detached-HEAD test from `not current_branch` to `is None`, each
break a named test.

**Field caveat resolved (operator-verified, recorded here so it is not
re-derived):** the reviewer could not check whether `classify_unlocatable_repo`'s
required-vs-optional split holds on the field hosts. All three manifest repos on
this host — `unh_marine_autonomy` (dev), `ccomjhc_project11` (operator/salmon),
`unh_echoboats_project11` (boat) — carry an **identical** `optional_layers.txt`
whose only entry is `site`. The split is therefore consistent across every host
this reaches and will not go permanently red on gabby or salmon.

### Actions
- [x] (must-fix) Doc lines attributing a script's exit code to a `make` target — `Makefile:68`, `.agent/hooks/post-checkout:77`, `.agent/hooks/README.md:16` — `ea8e36c`. Verified empirically on this host: a recipe exiting 3 prints `Error 3` and returns **2**. `make help` now points at `validate_workspace.py` for the codes; the two hook docs say the sync fails in a worktree by design without promising a code the operator never sees.
- [x] (must-fix) `make validate` no longer reached `test_layer_sourcing.sh` wherever `configs/manifest` is absent — `Makefile:141-143` — `ea8e36c` + `8b20ecf`. Both checks always run; the first failure's status is what the recipe returns. `test_make_validate.sh` runs the real recipe against stub scripts; reverting to the two plain lines fails its unconfigured and drift checks.
- [x] (must-fix) `--json` dropped a repo on a failed probe and exited 0 — `.agent/scripts/pull_remote.py` — `6ab3621`. `json_report()` now returns `(entry, error)` and separates three answers the old bare `None` collapsed: the remote has no such branch, the remote is not ahead, and **a probe failed**. The reproduced trigger (a default branch existing only as a remote-tracking ref) is named precisely and separately from a `rev-list` that failed otherwise, from output that is not two integers, and from a `git log` that failed while the remote is known ahead. Seven direct tests plus four through `main()`.
- [x] (must-fix) The enumeration layer beneath the probes — `lib/remote_utils.py`, `lib/workspace.py` — `f548535` (**operator scope call: fixed here, not deferred**). `get_overlay_repos()` raises `WorkspaceConfigError` instead of printing a parse error and returning the survivors; `find_repo_version()` went through the same shared loader rather than answering `"unknown"`. `iter_repos()` records a zero-repo enumeration as an error, so `pull_remote.py` / `push_remote.py` now exit 1 in the exact state `sync_repos.py` already exited 1 naming `configs/manifest`. The `missing` bucket now counts toward the exit status — routed to a benign `skip` when the layer is optional on this host, through the one `repo_absence_is_allowed()` predicate `sync_repos.py` also classifies on.
- [x] (suggestion) Orphan-only drift unasserted — `validate_workspace.py:327-331` — `f548535`. Both surviving mutations (dropping `extra_repos`, dropping `version_mismatches`) now fail a named test.
- [x] (suggestion) `remote_probe`'s exact-line match — `lib/remote_utils.py:106` — `f548535`. `cloud` vs `gitcloud` pinned in both directions.
- [x] (suggestion) Deleting `merge_pr.sh`'s sync-failure `exit` passed the suite — `.agent/scripts/tests/test_merge_pr.sh` — `1583a41`. Structural check on the branch, since no test reaches past the pre-merge guards by design; deleting line 437 now fails it.
- [x] (suggestion) A failed `rev-list` reported a bare `ok / fetched` — `pull_remote.py:100-106` — `6ab3621`. Now an error, matching the byte-identical line `sync_repos.py` fixed on this branch.
- [x] (suggestion) `dashboard.sh`'s `*)` arm called every non-zero code drift — `dashboard.sh:171-177` — `e2773c2`. 1 is drift; 2/127/anything else is surfaced with its code and points at a direct run. Tested for 2, 4 and 127.
- [x] (suggestion) No `Exit status:` docstring in `push_remote.py` or `sync_repos.py` — `f548535`. Both added, both stating make's flattening.
- [x] (suggestion) `UNREADABLE_STATE` re-typed as a literal; `_push_args()` omitted `set_default_branch` — `f548535`. Both reason strings moved into `lib/remote_utils.py`, and both scripts now classify `RemoteState` through one shared `classify_remote_state()` helper. The test fixture mirrors the real parser, with a test that reaches the end of `process_repo()` so a wrong fixture cannot pass again.
- [x] (suggestion) Plan's "Agent-instruction candidates: none identified" — `plan.md` — `0a8bd3d`. Revised: the pattern now spans seven scripts and two shared libs, so the plan **proposes** an `.agent/knowledge/` entry for operator approval (AGENTS.md § Ask First), including its hard half — prefer a benign skip wherever a supported host configuration produces the same signal.
- [x] (suggestion) "Every post-merge failure path exits 3" overstated — `AGENTS.md:572` — `ea8e36c`. Corrected: the two branch-deletion steps end in `|| true` on purpose, since GitHub's auto-delete-on-merge routinely gets there first.
- [x] (suggestion) Files-to-Change omissions, duplicate row, unchecked Plan Review items — `0a8bd3d`. Six files added, the duplicate labelled as a continuation, and the five Plan Review actions checked after verifying each on the branch — one of which was **not** in fact done: the plan still claimed `sync_gitbug()` runs unconditionally today. Verified against `main` that both call sites were already gated on `sync_repo()`'s return; the plan now says so.
- [x] (suggestion) Residual same-class instance in the version chain — `validate_workspace.py:284-324` — `1e7df5c`. Both git probes failing on a directory that has a `.git` is now its own reported mismatch. Also corrected the neighbouring claim in the same chain: `current_branch is None` was commented "Detached HEAD", but `git branch --show-current` succeeds on a detached HEAD and prints nothing — `""`, not `None`. Tested in both directions, including that a SHA-pinned, genuinely detached repo still passes.
- [x] (suggestion) `--json` fetched the workspace root without a `remote_probe` — `pull_remote.py:328-334` — `6ab3621`. The root now goes through the same probe as every other repo, so a root with no such remote is skipped as the `AGENTS.md` contract says, not reported as a fetch failure.
- [x] (suggestion) `classify_unlocatable_repo`'s split unverifiable from the review — **verified, no code change**. All three manifests carry an identical `optional_layers.txt` listing only `site`; see the field-caveat note above.

### Consequences handled beyond the listed findings
- `AGENTS.md`'s `pull_remote.py` / `push_remote.py` / `validate_workspace.py` rows describe the widened contracts, not only the per-repo probe fixes (`87d2916`).
- `list_overlay_repos.py` gained a `main()` so its CLI arm is testable, and exits 1 naming the file rather than printing `[]` for a manifest that will not parse.
- Two pylint findings (`too-many-return-statements`, `duplicate-code`) were resolved by restructuring — an `_ahead_behind()` split and the shared `classify_remote_state()` — never by suppression.

### Next step
`review-code` re-review of `f548535..0a8bd3d`.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-08-24 15:56 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-609 at `efcc286`
**Mode**: pre-push
**Depth**: Deep — **targeted re-read of `f548535~1..efcc286`** (the Round-3 fix pass, 10 commits), not a fourth full review; scoping was the Round-3 reviewer's own recommendation, accepted by the operator
**Must-fix**: 3 | **Suggestions**: 13
**Round**: 4 | **Ship**: continue — the count is falling (5 → 4 → 4 → 3) and the *core* of the fix pass verifies clean, but all three must-fixes are this issue's own defect class sitting one consumer down from the fixes, and one is a live-reproduced false RED, which the design's own docstring calls worse than the false green it removes

**Specialists**: Static Analysis (pre-commit clean on all 23 changed files — shellcheck/black/flake8/pylint, nothing suppressed) · Governance · Plan Drift · Claude Adversarial ×2 (Lens A + Lens B, Deep horizon). Copilot Adversarial off (default). **Local Adversarial could not run: the Ollama server answers on :11434 but `/api/tags` returns `{"models":[]}` — `qwen3.5:35b` is not pulled on this host, so `local_review.sh` would exit 2.** The quota-free cross-model read did not happen on this diff, for the fourth round running.

**Lead verification**: full suite green (23 shell suites + 191 pytest, up from 22 + 152). **34 independent mutations against a pristine `git archive` sandbox — 31 killed, 3 survived** (suggestions 3 and 4 below). Both directions exercised: the false-RED mutations (optional-layer absence forced red, `repo_absence_is_allowed` forced False, `optional_layers` forced empty, a remote that simply lacks the branch made an error, the detached-HEAD `is None` revert) each break a named test. `make validate` run live in this worktree: both checks now run, script returns 3, make reports 2 — Round-3 must-fix 2 is genuinely fixed. The `--json` vs non-JSON asymmetry in must-fix 3 was **reproduced in a scratch repo**, not inferred. `.github/workflows/validate.yml` runs only `make lint` / `make test-scripts` / `check_pr_authors.py`, so no CI job consumes a changed exit code.

**Not re-derived** (host ground truth, per the dispatch): all three manifest repos carry an identical `optional_layers.txt` whose only entry is `site`.

### Findings
- [x] (must-fix) The priority fix's consumer was never updated: `/import-field-changes` still says "If the result is empty, report 'No field changes to import' and stop", with no instruction to read the exit status or the new `ERROR:` stderr lines. `json_report()`'s `(None, error)` and `--json`'s exit 1 are invisible at the one place the docstring says the false green cost the most — an agent following the procedure verbatim still lands on "nothing to import" for a repo whose probe failed. Cross-confirmed by both adversarial lenses — `.claude/skills/import-field-changes/SKILL.md:50`
- [x] (must-fix) `find_repo_version()`'s new raise changes nothing at either consumer, and regressed multi-manifest robustness. `worktree_create.sh:52-57` runs it in an embedded `python3 -c` with `2>/dev/null`, so `WorkspaceConfigError` is discarded into an empty version — indistinguishable from "not found" — and `create_draft_pr` opens the draft PR against an empty base branch, silently. `get_repo_info.py:31` has no handler at all: a traceback and rc 1 where its own docstring and `.agent/scripts/README.md:582-596` still promise `unknown`. Separately, the old code did `continue` past an unparseable file (verified against `origin/main`); the shared `_load_repos_file()` now aborts at the **first** bad file in glob order, so one broken `site.repos` breaks version lookup for repos declared in perfectly valid manifests. `lib/workspace.py:192-194` justifies the raise with "callers branch a worktree off that answer" — that is still what the actual caller does. Cross-confirmed by both lenses — `.agent/scripts/worktree_create.sh:52-57`, `.agent/scripts/get_repo_info.py:31`, `.agent/scripts/lib/workspace.py:191-199`
- [x] (must-fix) **False RED, reproduced live**: the same repo in the same state is an error in `--json` and a benign skip in default mode. A checkout whose default branch exists only as a remote-tracking ref (what `vcs import` produces for a SHA- or tag-pinned manifest entry) gives `json_report()` → `(None, "no local 'main' to compare against gitcloud/main …")` → whole run exit 1, while `_fetch_and_report()` on the identical repo gives `('skip', 'fetched (no main on local branch)')` → exit 0. This same PR's `validate_workspace.py:320-330` explicitly protects that state ("a repo pinned to a SHA is legitimately detached, and flagging it would be a false red"). The two modes must agree, and the answer is knowable — `origin/<branch>` is present and is what tracks the field remote's counterpart, so "cannot tell whether the remote is ahead" is not quite true. Needs a one-line operator call on which way they agree — `.agent/scripts/pull_remote.py:296-301` vs `:92-98`
- [x] (suggestion) Three consumers swallow `list_overlay_repos.py`'s new exit 1 with `2>/dev/null` / `|| echo "[]"`, converting "the survivors" into *zero* repos: `pr_status.sh` then prints a clean triage table over nothing, and dashboard's Untracked detection turns itself off. Same defect class, one consumer down — `.agent/scripts/pr_status.sh:47`, `.agent/scripts/dashboard.sh:295`, `.agent/scripts/dashboard.sh:465`
- [x] (suggestion) `add_remote.py` inherited the widened exit contract through the shared `run_script` → `iter_repos` → `print_summary_and_exit` (now exit 1 on a required-layer repo with no checkout, and on an empty enumeration) but is the one sibling that got neither an `Exit status:` docstring nor an updated `AGENTS.md` row. It is also the script most likely to run *before* every layer is imported — `.agent/scripts/add_remote.py:84`, `AGENTS.md:573`
- [x] (suggestion) **Surviving mutation**: both new error returns in `_compare_branches` can be replaced with `("ok", "fetched")` and all 191 tests still pass. That is `pull_remote.py`'s *default* mode, and `6ab3621` added those lines specifically to close a false green; nothing references `_compare_branches` or `_fetch_and_report` in any test file — `.agent/scripts/pull_remote.py:105-114`
- [x] (suggestion) **Surviving mutations**: two of the three exit-3 paths `AGENTS.md:572` names can be turned into no-ops with the whole suite green — the `cd "$ROOT_DIR" ||` failure and the worktree-removal failure. Only the sync branch is structurally pinned (that check does work; deleting its `exit` fails the suite) — `.agent/scripts/merge_pr.sh:369,392`, `.agent/scripts/tests/test_merge_pr.sh:197-215`
- [x] (suggestion) `Summary: N repos` now miscounts: `iter_repos` records the *enumeration* failure into the same results dict before the loop, so `total = sum(results.values())` includes it. Reproduced live — "Summary: 2 repos" over one repo processed. Cosmetic, but it miscounts in the output line the fix exists to make trustworthy — `.agent/scripts/lib/remote_utils.py:329`
- [x] (suggestion) The new unreadable-repo record is appended to `version_mismatches`, so it prints under "Version mismatches" and returns `DRIFTED`, which `dashboard.sh:180` renders as "Workspace drift detected. Run: make validate". A corrupt `.git` is not drift and `make validate` is not its remedy — the same conflation the dashboard `case` was just fixed to stop making — `.agent/scripts/validate_workspace.py:276-291`
- [x] (suggestion) `get_optional_repo_names()` is a fourth implementation of the optional-layer rule that `f548535` consolidated everywhere else onto the one shared `repo_absence_is_allowed()` "so this and the remote scripts cannot drift". Currently equivalent; nothing pins that — `.agent/scripts/validate_workspace.py:197-207`
- [x] (suggestion) `get_repos()` applies the `--manifest` filter *before* `iter_repos` tests for emptiness, so `--manifest <typo>` — or any filter selecting nothing — reports `NO_REPOS_ENUMERATED`: "configs/manifest: no repositories configured … Run `make setup-all`". Neither the diagnosis nor the remedy fits a filter that matched nothing — `.agent/scripts/lib/remote_utils.py:221-227,262-264`
- [x] (suggestion) `json_report`'s docstring asserts "Exactly one of the two is ever set" and then documents `(None, None)` three lines later. That sentence is the stated contract this pass exists to introduce — `.agent/scripts/pull_remote.py:323`
- [x] (suggestion) The local-ref probe is `rev-parse --verify <branch>` (loose lookup — also matches `refs/tags/<branch>`), while `get_default_branch()` deliberately verifies `refs/heads/{version}`. A tag named like the default branch satisfies the check and the subsequent `rev-list` compares the tag — `.agent/scripts/pull_remote.py:296`
- [x] (suggestion) `_get_ahead_commits` caps at 50 and the JSON entry carries no truncation marker, though non-JSON mode prints "… and N more". A consumer diffing `len(commits)` against `behind` sees an unexplained shortfall on any repo more than 50 commits behind — `.agent/scripts/pull_remote.py:258,347-359`
- [x] (suggestion) `test_json_mode_skips_a_workspace_root_with_no_such_remote` installs a `fetch_fails` stub that `_json_main` immediately overwrites with an always-succeeding one, so the test does not exercise what its docstring claims. Confirmed by mutation: removing the guard it names is killed by an *older* test, not this one — `.agent/scripts/tests/test_pull_remote.py:534-553`
- [x] (suggestion) Missing sentence terminator: "… a leftover branch strands nothing **Through `make merge-pr` none of these survive**" — `AGENTS.md:572`

### What verified clean
- `json_report()`'s tri-state and every path through `_ahead_behind()` / `_get_ahead_commits()`: 8 mutations, all killed, benign and error answers pinned in both directions.
- The enumeration layer: 12 mutations, all killed, including every false-RED variant of the optional-layer predicate.
- `make validate`, `dashboard.sh`'s exit-code arms, `sync_repos.py`'s manifest-error path, `validate_workspace.py`'s unparseable-manifest and unreadable-repo arms, `merge_pr.sh`'s sync-failure exit: all mutation-killed.
- Plan drift: none. Every changed file is in the plan's Files-to-Change table (`validate_workspace.py` under a `(cont.)` row); the two plan-only rows are `validate_workspace.py` itself and `test_net_retry.py`, which the plan listed as "verify, no changes expected".
- Governance: consequences-map rows for a changed `.agent/scripts/` script (AGENTS.md table, Makefile) are both done. The gap is the unnamed one — `docs/PRINCIPLES.md` "A change includes its consequences" over the *consuming skills*, which is must-fix 1.
- No CI job consumes a changed exit code.

### Next step
`address-findings` on this entry, then a Round-5 re-review scoped to the fix commits. Must-fix 3 needs an operator decision (which way the two modes agree) before it can be worked.

## Implementation
**Status**: complete
**When**: 2026-08-24 16:28 -04:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-609 at `fd94e07`
**Addressed**: `## Local Review (Pre-Push)` (Round 4) of 2026-08-24 15:56 -04:00, branch at `efcc286`
**Commits**: cb1dbc4, 2ebc1a3, c485228, b31f85d, b7a9c81, 55a76bc, c125718, 4859fd9, 7e2a16d, 5c37204, fd94e07

All 3 must-fixes and all 13 suggestions actioned; none deferred. Suite green
throughout: 24 shell suites + 215 pytest (up from 23 + 191). Every new or
changed guard mutation-checked in **both** directions — 18 further mutations
this round, all killed, including a false-RED probe on each.

### Actions
- [x] (must-fix) `/import-field-changes` now reads the exit status and the `ERROR:` stderr lines before concluding anything from an empty report, consumes the new per-entry `state`, and reports the repos that were *not* checked under their own "Not checked (no conclusion drawn)" heading — the producer fix's consumer, which still said "if the result is empty, report 'No field changes to import' and stop" — `.claude/skills/import-field-changes/SKILL.md:43-95`, `AGENTS.md:575` (2ebc1a3)
- [x] (must-fix) `find_repo_version()` skips and continues past an unparseable manifest again, so one broken `site.repos` no longer breaks lookups in valid ones, and raises only when the lookup found nothing *and* a file went unread. Both consumers now handle it: `get_repo_info.py` gained a `main()`, a one-line `ERROR:` and a documented exit 1 (was a traceback where its docs promise "unknown"), and `worktree_create.sh`'s `resolve_repos_branch` no longer swallows the failure into an empty version — both call sites say when they fall back. `dashboard.sh` distinguishes the refusal from "unknown". Manifest globs sorted so no answer depends on inode order. Enumeration-layer strictness untouched and pinned by its own test — `.agent/scripts/lib/workspace.py:164-215`, `.agent/scripts/get_repo_info.py`, `.agent/scripts/worktree_create.sh:47-75`, `.agent/scripts/dashboard.sh:330`, `.agent/scripts/README.md:582` (c485228)
- [x] (must-fix) The false RED, resolved the way the operator decided: **reported, not failed.** `STATE_NO_LOCAL_BRANCH` is a named state both modes agree on — default mode a visible `skip`, `--json` its own entry with `state` and `detail` — and neither contributes to a non-zero exit. `_ahead_behind()` is now `(counts, state, error)`; every `--json` entry carries an explicit `state` so the consumer reads it rather than inferring "nothing to import" from an absence. Live-reproduced with real git in a scratch repo: identical wording, both modes, exit 0 — `.agent/scripts/pull_remote.py:57-84,105-125,285-330,340-405` (cb1dbc4)
- [x] (suggestion) `pr_status.sh` and both `dashboard.sh` call sites report a failed `list_overlay_repos.py` instead of converting it to zero repos; the dashboard names the check it had to switch off ("untracked-repo detection is OFF") rather than silently calling every repo tracked — `.agent/scripts/pr_status.sh:46-56`, `.agent/scripts/dashboard.sh:295-306,470-479` (b31f85d)
- [x] (suggestion) `add_remote.py` documents the exit contract it inherited through `run_script`, in its docstring and its `AGENTS.md` row — `.agent/scripts/add_remote.py:13-30`, `AGENTS.md:573` (b7a9c81)
- [x] (suggestion) **Surviving mutation killed**: `_compare_branches`'s two error returns are now exercised through `process_repo()` in default mode (four new tests, including the "up to date" false-RED direction) — `.agent/scripts/tests/test_pull_remote.py:410-470` (cb1dbc4)
- [x] (suggestion) **Surviving mutations killed**: the `cd "$ROOT_DIR"` and worktree-removal exit-3 paths are now pinned structurally, the same way the sync branch already was; deleting either `exit` fails the suite — `.agent/scripts/tests/test_merge_pr.sh:218-250` (55a76bc)
- [x] (suggestion) The enumeration failure records into its own `ENUMERATION_FAILURE` bucket: still exit 1, but out of the `Summary: N repos` count and stated separately — `.agent/scripts/lib/remote_utils.py:45-70,320-345` (4859fd9)
- [x] (suggestion) An unreadable repo is `ValidationResult.UNREADABLE` (exit 4) with its own heading and remedy, not drift; `dashboard.sh` gained the matching arm, so it no longer answers a corrupt `.git` with "Run: make validate" — `.agent/scripts/validate_workspace.py:53-90,296-310,375-425`, `.agent/scripts/dashboard.sh:182`, `AGENTS.md:576`, `Makefile:70` (5c37204)
- [x] (suggestion) `get_optional_repo_names()` decides through the one shared `repo_absence_is_allowed()` instead of a fourth copy of the rule, pinned by a spy test — `.agent/scripts/validate_workspace.py:197-217` (5c37204)
- [x] (suggestion) `get_repos()` returns `(repos, empty_reason)` and asks the emptiness question of the *unfiltered* list, so `--manifest <typo>` names the filter instead of diagnosing an unbootstrapped workspace — `.agent/scripts/lib/remote_utils.py:48-64,219-240` (4859fd9)
- [x] (suggestion) `json_report`'s docstring no longer claims "exactly one of the two is ever set" while documenting three outcomes; it enumerates the states instead — `.agent/scripts/pull_remote.py:340-372` (cb1dbc4)
- [x] (suggestion) Both ref probes are namespaced (`refs/heads/{branch}`, `refs/remotes/{remote_ref}`), so a tag named like the default branch can no longer satisfy the check — `.agent/scripts/pull_remote.py:107-118,296-310` (cb1dbc4)
- [x] (suggestion) `--json` entries carry `commits_truncated`, so a consumer diffing `len(commits)` against `behind` past the 50-commit cap sees the shortfall explained — `.agent/scripts/pull_remote.py:395-400` (cb1dbc4)
- [x] (suggestion) `_json_main` takes the fetch stub as an option instead of overwriting the one the test installed; the workspace-root test now genuinely exercises the guard it names (confirmed: the mutation it claims to kill now fails *it*) — `.agent/scripts/tests/test_pull_remote.py:595-615,640-660` (7e2a16d)
- [x] (suggestion) Sentence terminated before the `make merge-pr` caveat — `AGENTS.md:572` (c125718)

### Neighbouring claims re-verified (not flagged by the review)
- `AGENTS.md`'s `pull_remote.py` row still asserted that `/import-field-changes` "reads an empty report as no field changes to import" — true before 2ebc1a3, stale after it. Corrected in the same commit.
- `.agent/knowledge/field_mode_hotfix.md` and `/wrap-up-deployment` step 8 were checked for the same inference: neither draws a conclusion from an empty `--json` report (step 8 is operator-driven), so neither needed a change.
- `validate_workspace.py`'s `--fix` path was checked against the new UNREADABLE state: `--fix` is gated on DRIFTED, and an unreadable repo contributes nothing to `missing_repos`, so `--fix` cannot try to import over it. Asserted.
- `.agent/scripts/README.md`'s `get_repo_info.py` section promised "unknown if not found" with no exit-status contract at all. Updated alongside the script's docstring.

### Live verification (not inferred)
- The `no-local-branch` state reproduced with real git in a scratch repo (clone, delete the local default branch, add the second remote): `--json` returns the labelled entry with `err is None`, default mode returns `("skip", …)`, and the two print the same sentence.
- `validate_workspace.py` run against the **real 44-repo workspace** (root overridden, read-only): returns DRIFTED/1 over one pre-existing orphan (`image_warper`), **not** UNREADABLE — the new state does not fire on a healthy host.
- `make validate` semantics unchanged in this worktree (script 3, no manifest); `pull_remote.py --json` exits 1 here naming `configs/manifest`, as designed.

### Deferred
None.

### Next step
`review-code` re-review, scoped to `c198240..HEAD` (11 commits).
