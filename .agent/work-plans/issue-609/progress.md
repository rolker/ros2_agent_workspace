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
- [ ] (must-fix) `get_current_branch()` returns `""` for a real detached HEAD but `None` when the git command *fails* (not a repo, corrupt `.git`); `if not branch` collapses both. Classifying both as SKIPPED leaves a genuinely broken repo silently stale under a green exit — the exact failure class this issue exists to kill. Split: `""` → SKIPPED, `None` → FAILED, and split the message at `sync_repos.py:140` — `plan.md` step 1
- [ ] (must-fix) Test isolation seams are unspecified ("monkeypatch `subprocess.run` / `run_git_cmd`"). `main()` derives `root_dir` from the module-level `SCRIPT_DIR` and calls `list_overlay_repos.get_overlay_repos()`, so as written the `main()`-level cases would read this host's real `layers/` and be non-hermetic. Name the seams: `monkeypatch.setattr(sync_repos, "SCRIPT_DIR", tmp_path/".agent"/"scripts")` (read at call time), stub `sync_repos.list_overlay_repos.get_overlay_repos`, create the repo dirs under `tmp_path` so `candidate_path.exists()` is real-but-controlled, stub `sync_repos.sync_gitbug` to count calls, and assert exit via `pytest.raises(SystemExit)`. Split the layers: test `sync_repo()` classification at the `run_network_cmd` seam (that is the test that actually catches the fall-through-to-`True` bug), and test `main()` accumulation/exit with `sync_repo` itself stubbed to return canned outcomes — otherwise the end-to-end cases assert the implementation back to itself — `plan.md` step 6
- [ ] (correction) Step 2's premise is wrong: `sync_gitbug()` does **not** run today for dirty / detached / missing-path repos — `sync_repo()` early-returns `False` there, and both call sites are already gated. The only behavior change from gating on `SYNCED` is that git-bug stops running after a failed pull/fetch, which *is* the bug fix, not a "small behavior tightening beyond the literal bug fix". Drop that framing so the PR body doesn't claim a change it isn't making — `plan.md` step 2
- [ ] (suggestion) Unresolvable-path → FAILED (step 3) is right, and carries no false-red risk on a fully set-up workspace (verified: `sync_repos.py --dry-run` on this host resolves every repo, zero skips). But on a host that deliberately hasn't imported a layer it makes `make sync` permanently red. Give it a distinct reason string ("not checked out — path unresolved") so the summary distinguishes it at a glance from a network failure, and say so in the PR body — `plan.md` step 3
- [ ] (suggestion) `merge_pr.sh` snippet: prefer `sync_status=0; make -C "$ROOT_DIR" sync || sync_status=$?` over the `if/else` + bare `$?` form — same `set -e` safety, no reliance on `$?` surviving into an `else` branch. Also note GNU make exits **2** on a failed recipe, so the banner will usually print "exit 2", not 1 — `plan.md` step 5
- [ ] (suggestion) Close the `test_merge_pr.sh` consequences row now: verified it only exercises pre-merge guard paths (arg handling, worktree resolution, field-mode guard) and by design never reaches line 372, so no new case is feasible — `bash -n` is the realistic coverage. Record that rather than leaving it as open scope — `plan.md` Consequences table
- [ ] (suggestion) Enum truthiness trap: every `enum.Enum` member is truthy, so a call site left as `if sync_repo(...)` keeps working and silently treats FAILED as success. Only two call sites exist, and the planned "git-bug not called on FAILED" test catches it — worth one line in the plan so the implementer doesn't leave a bare truthiness check — `plan.md` step 2
- [ ] (optional) `sync_repos.py:167-174`: when `git status -sb` itself fails, the code still prints `✅ Fetched.` Cosmetic mis-report in exactly this issue's theme; fix in passing or declare out of scope — `plan.md` step 1
- [ ] (context, no action) Caller audit: `make sync` / `sync_repos.py` are invoked only from `Makefile:221-222` and `merge_pr.sh:372`. `dashboard.sh` uses `vcs` directly and never calls the script; no Makefile target depends on `sync`; no GitHub workflow references it; no skill consumes `merge_pr.sh`'s exit code; nothing in the repo documents an always-exit-0 contract (the only mention of `Sync complete` is the print itself). No further consequences

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
- [ ] Split `not branch` into `""` (SKIPPED) and `None` (FAILED) with distinct messages
- [ ] Specify the test seams (`SCRIPT_DIR`, `get_overlay_repos`, `tmp_path` repo dirs, `sync_gitbug` call counter, `pytest.raises(SystemExit)`) and split `sync_repo()` classification tests from `main()` accumulation tests
- [ ] Correct step 2's "behavior tightening" framing — git-bug already skips dirty/detached repos today
- [ ] Give the unresolvable-path failure a distinct reason string in the summary
- [ ] Adopt the `merge_pr.sh` handling with `|| sync_status=$?`; keep `SyncOutcome` in `sync_repos.py`
