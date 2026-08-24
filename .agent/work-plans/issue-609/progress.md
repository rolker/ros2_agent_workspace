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
