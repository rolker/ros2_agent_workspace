# Plan: `make sync` reports success and exits 0 even when repos fail to update

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/609

## Context

`.agent/scripts/sync_repos.py`'s `sync_repo()` returns a bare `True`/`False`
that conflates two very different situations: an *intentional, benign skip*
(missing path, dirty working tree, detached HEAD — all by-design guards
against clobbering local work) and a *real failure to reach the remote*
(pull/fetch error). `main()` never accumulates outcomes, and the trailing
`print("\n✅ Sync complete.")` at line 284 is unconditional with no
`sys.exit(1)` path. `make sync` (`Makefile:221-222`) passes the script's exit
code straight through, so both `merge_pr.sh` (`merge_pr.sh:372`, under
`set -eo pipefail`) and any operator running `make sync` by hand see a green
result even when a repo silently failed to update.

The [Issue Review](progress.md) already confirmed every claim in the issue
against source and flagged the one thing the issue's own proposed fix
doesn't resolve: a naive "any `False` → failure" change would make dirty or
detached-HEAD repos — which are supposed to be silently skipped — start
failing `make sync`. That would trade a false-green for a false-red, which
is worse (a red everyone learns to ignore). This plan introduces an explicit
tri-state outcome instead of widening the boolean.

## Approach

1. **Introduce a tri-state result for `sync_repo()`.** Replace the bare
   `bool` return with a small enum (`SyncOutcome`, e.g.
   `SYNCED | SKIPPED | FAILED`, defined near the top of `sync_repos.py`) or
   an equivalent sentinel-string return — enum is preferred for
   self-documentation and to make an unhandled case a `NameError`/lint hit
   rather than a silent string typo:
   - `FAILED`: path missing, pull/fetch actually failed (the `❌ Update
     failed` / `❌ Fetch failed` branches).
   - `SKIPPED`: dirty working tree, detached HEAD/no branch — unchanged
     from today's behavior, just labeled instead of collapsed into `False`.
   - `SYNCED`: pull/fetch succeeded (including "already up to date" and the
     dry-run branches).
   Note the *existing* code already puts the pull/fetch failure prints
   inside the `if branch in [...]:` / `else:` legs without an early
   `return` — `sync_repo()` currently falls through to `return True` after
   printing `❌ Update failed`/`❌ Fetch failed`. The fix must add explicit
   `return SyncOutcome.FAILED` right after each of those two prints (not
   just change the final `return`), or the failure paths will keep
   reporting `SYNCED`.

2. **Gate `sync_gitbug()` on `SYNCED` only** (not "not FAILED") — a
   `SKIPPED` repo (e.g. deliberately dirty) should not run git-bug sync
   either, since that's arguably still "the repo wasn't touched this run."
   Verify this matches current behavior's *intent* (today `sync_gitbug()`
   runs unconditionally due to the `True`-everywhere bug) — call this out
   explicitly since it's a small behavior tightening beyond the literal
   bug fix, not just a bugfix-neutral refactor. Both call sites in `main()`
   (root repo ~248, per-overlay-repo ~281) change from
   `if sync_repo(...):` to `if sync_repo(...) == SyncOutcome.SYNCED:`.

3. **Accumulate failures in `main()`.** Track a list of `(repo_name,
   reason)` for every `FAILED` outcome (both the root repo and the loop
   over `repos`). The already-existing `continue` for an unresolvable path
   (~line 279, "could not resolve repository path") is also a real failure
   for exit-code purposes — the Issue Review flagged it as "silent about
   that too" — so it must append to the same failure list, not just print import list_overlay_repos ' Skipping...' and move on.

4. **Replace the unconditional summary with an outcome-aware one:**
   - All succeeded (no `FAILED` entries): keep the current
     `✅ Sync complete.` message (optionally note skip count, e.g.
     `✅ Sync complete — N repos, M skipped, 0 failures.` — keep it a
     single line consistent with today's brevity).
   - Any `FAILED`: print `❌ Sync finished with N failure(s):` followed by
     one `  - <repo_name>: <reason>` line per failed repo, then
     `sys.exit(1)`.
   Reason text should reuse what's already captured (the `output` string
   from the failed `run_network_cmd`/path-check), not a re-derived guess.

5. **`merge_pr.sh` messaging decision (flagged for operator sign-off —
   see Open Questions).** Once `sync_repos.py` can exit 1,
   `merge_pr.sh:372`'s bare `make -C "$ROOT_DIR" sync` under `set -eo
   pipefail` (line 26) will abort the script *before* printing
   `✅ Done: PR #… merged, cleaned up, and synced.` — even though the
   merge and worktree/branch cleanup already fully succeeded by that
   point. An abrupt `set -e` abort reads to the operator as "the merge
   script failed," which is misleading right when a stale-repo warning
   most needs to be legible (this is exactly the moment identified in the
   issue's "Why it matters" as least likely to be noticed).
   Proposed handling: wrap the sync call so a sync failure degrades to a
   **loud but distinct warning** rather than a hard abort of the whole
   script, since the operator does not need `merge_pr.sh` itself to fail —
   they need to know sync didn't fully succeed:
   ```bash
   echo "  Syncing all repos..."
   if make -C "$ROOT_DIR" sync; then
       sync_status=0
   else
       sync_status=$?
   fi

   echo ""
   echo "========================================"
   if [[ "$sync_status" -eq 0 ]]; then
       echo "✅ Done: PR #${PR_NUM} merged, cleaned up, and synced."
   else
       echo "⚠️  PR #${PR_NUM} merged and cleaned up, but the post-merge sync"
       echo "   reported failures (exit $sync_status) — see the sync output"
       echo "   above. Re-run 'make sync' to retry."
   fi
   echo "========================================"
   [[ "$sync_status" -eq 0 ]] || exit "$sync_status"
   ```
   This still exits non-zero (so scripted/CI callers of `merge_pr.sh` see
   red) but the operator-facing banner correctly attributes *what*
   failed instead of reading as a generic script crash — and it survives
   `set -e` because the failure is captured via the `if`, not left to
   propagate. This is the one part of the plan that changes a second
   script beyond `sync_repos.py`; see Open Questions for the go/no-go.

6. **Tests** — new `.agent/scripts/tests/test_sync_repos.py` (matching the
   existing `test_net_retry.py` / `test_progress_read.py` pytest
   convention: `sys.path.insert` for `SCRIPTS_DIR` and `SCRIPTS_DIR/lib`,
   monkeypatch `subprocess.run` / `run_git_cmd` rather than hitting real
   git). Cover, at minimum:
   - All-success run: every repo returns `SYNCED`, `main()` prints the
     success summary, process exits 0.
   - One real failure (simulate `run_network_cmd` returning
     `(False, "...")` for pull): repo is named in the `❌` summary,
     `sync_gitbug()` is NOT called for that repo (assert via a stub/mock
     call count), process exits 1.
   - One benign skip (dirty tree via `is_dirty` stub returning `True`):
     repo is *not* counted as a failure, `sync_gitbug()` is NOT called for
     it either (per step 2), overall exit code is still 0 if nothing else
     failed.
   - Unresolvable path in the `repos` loop (step 3): counts as a failure
     and appears in the summary / exit 1.
   - Mixed run (one `SYNCED`, one `SKIPPED`, one `FAILED`): exit 1, summary
     names only the `FAILED` repo, skip is silent as today.
   Existing `test_net_retry.py` already imports `sync_repos` and exercises
   `run_network_cmd`/`sync_gitbug` routing — check it after the enum change
   for any assertions on `sync_repo`'s return value that need updating
   (e.g. anywhere it asserts `is True`); grep before assuming it's
   unaffected.

7. **`AGENTS.md` script-table update** — extend the `sync_repos.py` row
   ("Sync all workspace repositories (includes git-bug)") with one clause
   noting the new exit-code contract, e.g.: "Sync all workspace
   repositories (includes git-bug); exits non-zero if any repo fails to
   update (dirty/detached-HEAD skips don't count as failures)."

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/sync_repos.py` | Tri-state `SyncOutcome` enum; `sync_repo()` returns it with explicit returns on the two failure prints; `main()` accumulates `FAILED` entries (including unresolved-path skips) and the two `sync_gitbug()` call sites gate on `== SyncOutcome.SYNCED`; outcome-aware summary + `sys.exit(1)` on any failure |
| `.agent/scripts/tests/test_sync_repos.py` | New — all-success, real-failure, benign-skip, unresolved-path, `sync_gitbug`-gating, mixed-run cases |
| `.agent/scripts/merge_pr.sh` | Post-merge `make sync` call captures its exit status instead of relying on bare `set -e`; distinct warning banner + propagated non-zero exit on sync failure, vs. today's implicit crash-looking abort |
| `AGENTS.md` | One-clause addition to the `sync_repos.py` script-table row noting exit-code semantics |
| `.agent/scripts/tests/test_net_retry.py` | Check for `sync_repo` return-value assertions that need updating for the enum change (no changes expected if it only checks `run_network_cmd`/`sync_gitbug` call routing, but verify) |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Test what breaks | New `test_sync_repos.py` covers exactly the accumulation/exit-code/gating logic this issue is about — the previously-untested surface |
| A change includes its consequences | `merge_pr.sh`'s consumption of `make sync`'s exit code is handled explicitly (step 5) instead of left to a `set -e` side effect; `AGENTS.md` row updated (step 7) |
| Only what's needed | No change to `retry_transient`/pacing logic (out of scope per the issue and Issue Review); `merge_pr.sh` change is scoped to the one call site that consumes the exit code, not a broader rewrite |
| Enforcement over documentation | The fix makes the script enforce truthful reporting via exit code, not just document the caveat |
| Human control and transparency | Tri-state design specifically avoids trading a false-green for a false-red that operators would learn to ignore; `merge_pr.sh` banner keeps the operator-facing message accurate about what succeeded vs. failed |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0010 — git-bug for local issue tracking | Yes (minor) | `sync_gitbug()` gating tightens from "always runs" (today's bug) to "runs only after a real `SYNCED` outcome" — preserves existing graceful-degradation behavior (no git-bug installed / no bridge configured still no-op) since those checks live inside `sync_gitbug()` itself, untouched |
| 0018 — Local-first CI verification | Watch, not triggered | `sync_repos.py` isn't part of the `ci-local` attestation path; noted only because "a green signal should be trustworthy" is the same spirit this issue is fixing for `make sync` |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `sync_repo()`'s return type (bool → enum) | Every call site (`main()`, both `sync_gitbug()` gates) | Yes — step 2 |
| `sync_repos.py` exit code semantics | `AGENTS.md` script-table row | Yes — step 7 |
| `merge_pr.sh`'s handling of `make sync`'s exit code | `merge_pr.sh`'s existing `test_merge_pr.sh` test file — check whether it stubs `make sync` and needs a case for the new failure-banner path | Yes — step 5 implementation; test coverage for `merge_pr.sh` itself is an open question (see below) rather than committed scope, since the existing `test_merge_pr.sh` conventions need to be read first |
| Test suite composition | `run_script_tests.sh` picks up `test_*.py` in `tests/` automatically (already confirmed — no runner change needed) | N/A, no action needed |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): `AGENTS.md`'s `sync_repos.py`
  script-table row currently says only "Sync all workspace repositories
  (includes git-bug)" with no exit-code semantics — this becomes
  inaccurate once the fix ships (a caller could reasonably assume exit 0
  always meant success, which will no longer be true). Step 7 updates it.
- **Agent-instruction candidates** (proposals only): none identified —
  the tri-state pattern (real-failure vs. benign-skip) is local to this
  script's domain (dirty/detached-HEAD guards), not a generalizable
  workspace convention worth promoting to `.agent/knowledge/`.

## Open Questions

- [ ] **Confirm the `merge_pr.sh` messaging approach (step 5)** — this
  plan proposes catching `make sync`'s exit code and printing a distinct
  "merged and cleaned up, but sync failed" warning banner while still
  exiting non-zero, instead of letting `set -eo pipefail` abort the
  script with no final banner at all. This changes `merge_pr.sh` itself
  (not just `sync_repos.py`), which is a second file beyond the issue's
  literal scope — flagging explicitly per the issue review's finding #3
  rather than silently deciding it. Alternative if rejected: leave
  `merge_pr.sh` as-is (accept the abrupt `set -e` abort with no closing
  banner) and only fix `sync_repos.py`'s own reporting/exit code.
- [ ] Should the tri-state enum live in `sync_repos.py` itself or move to
  a shared location (e.g. `lib/`)? Nothing else currently needs it, so
  the plan keeps it local to `sync_repos.py` unless review disagrees.

## Estimated Scope

Single PR. Two files change behavior (`sync_repos.py`, `merge_pr.sh`),
one file gains a new test module, `AGENTS.md` gets a one-line doc update.
