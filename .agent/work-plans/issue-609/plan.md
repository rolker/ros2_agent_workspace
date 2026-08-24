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
| `.agent/scripts/lib/workspace.py` | Added during implementation — new shared `get_optional_layers()`, extracted so `sync_repos.py` and `validate_workspace.py` decide "this layer may be absent" from one parser (byte-compatible with `setup_layers.sh`'s `is_optional_layer()`) instead of two |
| `.agent/scripts/validate_workspace.py` | Added during implementation — switched from its own optional-layer parsing to the shared helper; behaviour unchanged (a repo in an optional layer stays allowed-missing) |
| `.agent/scripts/tests/test_workspace_lib.py` | Added during implementation — pins the shared parser's comment/blank handling, its missing-file `set()`, and its agreement with `setup_layers.sh` |
| `AGENTS.md` | Exit-code semantics for the `sync_repos.py`, `pull_remote.py`, `push_remote.py` and `validate_workspace.py` script-table rows |
| `.agent/scripts/pull_remote.py` | Scope widening — tri-state `is_dirty()` / `get_current_branch()`; `_check_pull_preconditions()` returns an (status, message) problem so an unreadable tree or git state is an `error` (exit 1), not a `skip`; the same carve-out in `_fetch_into_branch()` and in the `--json` arm |
| `.agent/scripts/tests/test_pull_remote.py` | New — classification driven through `process_repo()`, the entry point `iter_repos` calls |
| `.agent/scripts/validate_workspace.py` | Scope widening — three-state `ValidationResult` + `EXIT_CODES`; zero configured repos is `UNCONFIGURED` (exit 3), not a pass; `--fix` gated on drift and its re-validation result no longer discarded |
| `.agent/scripts/tests/test_validate_workspace.py` | New — outcome classification and exit status, stubbed at the `get_overlay_repos` / `get_actual_repos` seams |
| `.agent/scripts/dashboard.sh` | Consequence of the above — the validation check reads the exit code three ways instead of pass/fail |
| `.agent/scripts/tests/test_dashboard_validate_status.sh` | New — runs the real `dashboard.sh` against a stub validate script for each exit code |
| `.agent/scripts/lib/remote_utils.py` | Scope widening — `remote_probe()`/`RemoteState` tri-state (`remote_exists()` kept as the boolean wrapper for `add_remote.py`); `run_git()` catches `OSError` instead of letting it kill the run |
| `.agent/scripts/push_remote.py` | Consequence — an unreadable repo is an `error`, not a "remote not found" skip |
| `.agent/scripts/tests/test_remote_probe.py` | New — the shared probe, the `OSError` guard, and `push_remote.py`'s consumption of both |
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
| `merge_pr.sh`'s handling of `make sync`'s exit code | `merge_pr.sh`'s existing `test_merge_pr.sh` test file | Yes — step 5 implementation. **Row closed**: `test_merge_pr.sh` exercises only pre-merge guards (arg handling, worktree resolution, field mode) and by design never reaches the sync tail, so no end-to-end case is feasible. It instead locks the piece that *is* a contract — the reserved exit code 3, distinct from the usage-error 2 — alongside `bash -n`/shellcheck |
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

Both resolved; recorded here rather than deleted so the decision trail survives.

- [x] **`merge_pr.sh` messaging (step 5)** — **RESOLVED at the plan checkpoint:
  include it in this PR.** The reviewer's argument, which the operator accepted:
  this is not scope creep, because the exit-code change is what *creates* the
  abort, so handling it is that change's own consequence. By the time
  `merge_pr.sh` reaches its sync call the merge, worktree removal and branch
  deletion are all done and irreversible, so a bannerless `set -e` abort reads
  as "the merge failed" and invites retrying finished work. Implemented with
  `sync_status=0; make -C "$ROOT_DIR" sync || sync_status=$?` rather than
  `if/else` plus a bare `$?`, and the banner names *make's* exit code (make
  reports 2 for a failed recipe, not sync_repos.py's own 1).
- [x] **Where the enum lives** — **RESOLVED: stays in `sync_repos.py`** as a
  plain `enum.Enum`. `lib/` is for genuinely shared helpers; nothing else
  consumes `SyncOutcome`, and moving it later costs one import.

## Review-driven corrections (Plan Review, `5e3f763`)

- **A miscategorised early return, now fixed.** `get_current_branch()` returns
  `""` for a genuine detached HEAD but `None` when the git command itself
  *fails* (not a repo, corrupt `.git`). The original `if not branch` collapsed
  both, and this plan had classified both as SKIPPED — which would leave a
  genuinely broken repo silently stale under a green exit, the exact failure
  class #609 exists to kill. Split: `""` → `SKIPPED`, `None` → `FAILED`, each
  with its own message.
- **Test isolation seams named rather than left to "monkeypatch subprocess".**
  `main()` derives `root_dir` from module-level `SCRIPT_DIR` and calls
  `list_overlay_repos.get_overlay_repos()`, so end-to-end cases would otherwise
  read this host's real `layers/`. The tests patch `SCRIPT_DIR` and
  `get_overlay_repos`, build repo dirs under `tmp_path` so `.exists()` is
  real-but-controlled, count `sync_gitbug` calls, and assert exit status via
  `pytest.raises(SystemExit)`. Classification tests (at the `run_network_cmd`
  seam) are kept separate from `main()` accumulation tests — the former are the
  ones that catch the original fall-through.
- **A factual correction to this plan's step 2**, which claimed `sync_gitbug()`
  "runs unconditionally due to the True-everywhere bug". It does not: dirty,
  detached and missing-path repos already early-returned `False` and skipped
  git-bug. The only change is that git-bug now also stops running after a
  *failed pull/fetch* — which is the fix, not a behaviour tightening. Corrected
  so the wrong framing does not reach the PR body.
- **Second call site caught during implementation.** The root repo was synced
  via `if sync_repo(...)` at a separate line from the loop — the precise
  truthy-enum trap the review warned about, since every Enum member is truthy.
  Both call sites now route through one `record()` helper that compares against
  members explicitly, and a test asserts all members are truthy so the trap
  stays documented.
- **`preflight_repo()` extracted from `sync_repo()`.** The tri-state outcome
  turned `sync_repo()` into a function with more early returns than pylint's
  `too-many-return-statements` limit allows. The pre-sync guards (path, working
  tree, branch) moved into their own function rather than suppressing the check
  — the same call the tests make with a probe table instead of one flag per
  probe. No lint rule is disabled anywhere in this change.
- **Unresolvable paths report their own reason** (`path not resolved`) rather
  than a generic failure, so a host with an un-imported layer can tell that
  case apart from a network failure.

## Review-driven corrections (Local Review pre-push, `576a3ff`)

Round 1 of `review-code` found two independently-reproduced false-signal
defects sitting on either side of the very signal this PR exists to make
trustworthy, plus three more must-fixes and nine suggestions.

- **False green: `is_dirty()` treated an unreadable working tree as clean.**
  `success and bool(output)` collapsed "no changes" and "could not look",
  so a repo with a corrupt index *and* real uncommitted changes classified
  SYNCED — the exact class of false green #609 removes, and a contradiction of
  `SyncOutcome.FAILED`'s own "a state we cannot even read". Now True/False/None,
  with None → FAILED. `run_git_cmd()` also caught only `CalledProcessError`, so
  an unreadable repo directory raised `OSError` and killed the run with no
  summary; it is now a per-repo failure.
- **False red: an optional layer that was never imported failed the run.**
  `configs/manifest/optional_layers.txt` lists `site`, and `setup_layers.sh`
  deliberately removes an optional layer whose private repos it cannot import
  and exits 0 — so "path not resolved → FAILED" made `make sync` permanently
  red on a supported host. Three cases are now distinguished: optional layer
  absent → SKIPPED; required layer absent → FAILED, naming `setup_layers.sh`;
  repo missing inside an imported layer → FAILED, "path not resolved". A
  *required* repo that cannot be located still fails, as designed. (Round 2
  narrowed the middle case — see below: the carve-out is keyed on the layer
  being optional, not on the layer directory being absent.)
- **The root workspace repo's outcome was untested.** No `main()` test keyed an
  outcome on `ros2_agent_workspace`, so reverting its call site to the
  truthiness trap left all 14 tests green — on the repo `merge_pr.sh` merges
  into. Its failure and skip paths are now pinned, and the
  language-invariant `test_outcomes_are_all_truthy` (which passed under that
  same mutation) is gone.
- **`merge_pr.sh` propagated make's exit code 2** — already its own usage-error
  code, asserted as such by `test_merge_pr.sh`. rc=2 read as "invoked wrong"
  invites retrying a merge that is already done and irreversible. A reserved
  code 3 now means "merged and cleaned up, sync failed"; all four exit codes are
  documented in the script header and in `AGENTS.md`'s row, which previously
  never mentioned the script can exit non-zero with the merge complete.
- **An empty repo list claimed a quantified all-clear.** Where `configs/manifest`
  is missing (un-bootstrapped clone, or a workspace worktree) the summary read
  `✅ Sync complete — 1 synced, 0 skipped, 0 failures` while all 35 configured
  repos went unenumerated — a stronger false claim than the bare success line
  this issue replaced. It is now a named failure.
- **Suggestions actioned**: per-cause failure reasons (plan step 4's
  "reuse what's already captured" — the summary had recorded the literal
  "sync failed" for everything); an off-network host reported as "remote
  unreachable" rather than 35 identical pull failures; `git status -sb` failing
  no longer prints a bare `✅ Fetched.` (Plan Review finding 8, settled by
  fixing rather than deferring); exact summary counts asserted; the
  ADR-0010 rationale for git-bug failures not failing the run written down;
  the `AGENTS.md` row extended to every FAILED/SKIPPED cause; the failure banner
  moved to stderr; this plan re-synced.

## Verification performed

- All 14 new tests pass; full `run_script_tests.sh` green (87 pytest, up from 73).
- **Mutation-checked in both directions**: restoring the pull-failure
  fall-through fails `test_failed_pull_is_failed`, and re-collapsing the
  unreadable-git-state branch into the detached-HEAD skip fails
  `test_unreadable_git_state_is_failed`. The tests catch the bug they exist for
  rather than asserting the implementation back to itself.
- `bash -n` clean on `merge_pr.sh`; `sync_repos.py --dry-run` exits 0 on a clean
  run. Note a *workspace worktree* has an empty `layers/`, so a dry run there
  only walks the root repo — the loop is exercised by the tests, not by that.

After the Local Review round:

- 25 tests in `test_sync_repos.py` (from 14), full `run_script_tests.sh` green
  (21 shell, 98 pytest).
- **The three mutations that previously survived now fail**: the root call site
  reverted to `if sync_repo(...)`, a root FAILED demoted to SKIPPED, and the
  synced/skipped counts dropped from the success summary. Re-checked in both
  directions for each new behaviour as well — reverting `is_dirty()` to
  `success and bool(output)`, and both an over- and under-permissive
  optional-layer rule, each break a named test.
- **Live, not inferred**: a dry run against this host's real 35 repos through a
  mirrored root reports `35 synced`; removing `site_ws` from that mirror turns
  it into `34 synced, 1 skipped` with an optional-layer skip (Round 2 reworded
  the message), and no failure — confirming the false red is gone on the exact configuration
  that produced it.

## Review-driven corrections (Local Review pre-push Round 2, `d3d93d0`)

Round 2 verdict was changes-requested with **Ship: recommended** — four
mechanical must-fixes, no reopened design question.

- **The no-repos remedy named a target that does not exist.** It printed
  ``Run `make setup``` on the one path an operator reaches from an
  un-bootstrapped clone or a worktree; the target is `setup-all`
  (`make -n setup` → "No rule to make target 'setup'"). Corrected.
- **The exit-code contract was false about code 1.** Both the `merge_pr.sh`
  header and the `AGENTS.md` row promised `1 = failed before the merge, nothing
  irreversible happened`, while the worktree-removal refusal exited 1 *after*
  `gh pr merge` had landed — the exact misread code 3 was reserved to prevent,
  left uncovered on the sibling path. Every post-merge failure now exits 3:
  the worktree-removal refusal, the post-merge `make sync`, and the `cd` back to
  the workspace root, which `set -e` would otherwise have aborted bannerless
  with 1. `SYNC_FAILED_RC` is renamed `POST_MERGE_RC` for the widened meaning,
  and `test_merge_pr.sh` now asserts that no path after `gh pr merge` exits 1
  and that the `cd` is explicitly guarded.
- **Code 3 is invisible through the advertised entry point.** GNU make reports
  its own 2 for any non-zero recipe status (verified: a recipe of `exit 3`
  yields make rc=2), so `make merge-pr` surfaces a completed-merge failure as
  exactly the usage-error code, and `make sync` returns 2 rather than the
  documented 1. Both `AGENTS.md` rows and the script header now state what each
  entry point actually returns and steer exit-code-sensitive callers to the
  scripts directly.
- **The optional-layer carve-out was gated on the layer directory existing.**
  `setup_layers.sh` exits 0 in two states that leave the directory in place: a
  *partially* imported optional layer (it deletes the directory only when it
  created it that run) and a host with no `vcs` (empty `src`, "Setup complete").
  Both went permanently red — the false red the design's own docstring calls
  worse than the false green it removes — and both disagreed with
  `validate_workspace.py`, which allows any repo in an optional layer to be
  missing while now sharing the same parser. The carve-out is now keyed on the
  layer being optional, full stop; the corresponding test is flipped and the
  docstring's one-branch claim about `setup_layers.sh` corrected.
- **Suggestions actioned**: `.agent/hooks/post-checkout` and
  `.agent/hooks/README.md` no longer promise `make sync` works inside a
  workspace worktree (`configs/manifest` is gitignored there, so since this
  change it is a named exit 1); the three mutation survivors are closed —
  `locate_repo`'s explicit-path fallback (relative, absolute, and
  tried-paths cases) and the shared parser's comment-stripping and
  missing-file paths, the latter checked against `setup_layers.sh`'s own
  `is_optional_layer()` rather than a restatement of it; this plan re-synced
  with the two files the work added.
- **Round 2 proposed not widening** — the same `success and bool(output)` false
  green in `pull_remote.py:42` and the 0-repo all-clear in
  `validate_workspace.py` were to be tracked as a separate follow-up rather
  than folded into a diff that had had three review rounds. **The operator
  overruled this**; see "Scope widened after Round 2" below.

**Round 2 verification**: 106 pytest cases green across
`.agent/scripts/tests/` (29 in `test_sync_repos.py`, 4 in the new
`test_workspace_lib.py`), `test_merge_pr.sh` 20/20. Each new behaviour
mutation-checked in both directions: gating the optional-layer carve-out on the
directory again, and skipping unconditionally, each break a named test; so do
reverting the post-merge exit to 1, unguarding the `cd`, dropping the parser's
`#`-strip or blank-skip, returning a non-empty set for a missing file, and
deleting `locate_repo`'s explicit-path fallback.

## Scope widened after Round 2 (operator decision)

Round 2 ruled the sibling instances a follow-up issue. The operator overruled
that on two workspace rules, and they land on this branch, in this PR, as one
atomic commit per script:

- AGENTS.md's **filing discipline** — "for consolidation/cleanup work, default
  to bundling related changes into one PR (atomic commits inside) rather than
  fanning out many small issues".
- The **Quality Standard** — "when fixing a bug, fix it completely… never leave
  a 'good enough' fix when the proper one is within reach". A defect class
  found and left in place in a sibling script is exactly that.

The scope is *this defect class*: a git command that failed being reported as a
benign answer, so a run that did nothing still exits 0. Four sites, all of them
verified against source, not inferred:

1. **`pull_remote.py` `is_dirty()`** — `success and bool(output)`, so a failed
   `git status` read as "clean". The most consequential of the set: `--pull`
   *merges* on this answer.
2. **`pull_remote.py` `get_current_branch()`** — `output if success and output
   else None` collapsed a genuine detached HEAD with a failed git command. The
   identical bug #609 split apart in `sync_repos.py`, split the same way here:
   `""` is a benign skip, `None` is a repo we cannot read.
3. **`validate_workspace.py`** — `not (missing or extra or mismatched)` is
   vacuously true over an empty configuration, so a workspace with **zero**
   repos configured printed `✅ Workspace validation PASSED!` and exited 0.
4. **`lib/remote_utils.py` `remote_exists()` / `run_git()`** — found while
   re-verifying the neighbouring claims in `process_repo()`, the function site 1
   feeds. `remote_exists()` answered False when `git remote` itself failed, so
   both `pull_remote.py` and `push_remote.py` reported "remote not found" and
   skipped, green, a repo they had never read; and `run_git()` caught only
   `CalledProcessError`, so an unusable repo directory raised `OSError`,
   escaped, and killed the run with no summary. `remote_exists()` is kept as a
   boolean wrapper for `add_remote.py`, where False is the correct answer.

Three consequences the widened scope pulls in, each handled rather than noted:

- **`dashboard.sh:166`** runs `validate_workspace.py` and branched on pass/fail,
  so the new outcome would have printed "Workspace drift detected. Run: make
  validate" — sending the operator back to the same empty answer. It now reads
  the exit code three ways. (No CI workflow invokes the script — verified
  against `.github/workflows/`; `make validate` is the documented local gate.)
- **Exit-code numbering.** `validate_workspace.py`'s new UNCONFIGURED outcome is
  **3**, not 2: argparse already owns 2, the same collision Round 2 caught in
  `merge_pr.sh`. And as with `make sync`, GNU make flattens any non-zero recipe
  status to its own 2, so both AGENTS.md rows and the module docstrings say to
  branch on these codes only when calling the scripts directly.
- **A neighbouring bug in `validate_workspace.py`'s `main()`**: a successful
  `--fix` re-validated the workspace and then discarded the answer, exiting 1 on
  the stale result — a workspace `--fix` had just repaired still read red.

**Deliberately not touched** (each verified correct as written):
`lib/remote_utils.py:119`/`:124`, whose fallback chains are by design — each
probe falls through to the next and finally to `"main"`; `pull_remote.py`'s
`_compare_branches()` log listing, where a failure only omits detail from a
message; and `pull_remote.py`'s existing exit-code handling, which was already
correct. `add_remote.py` keeps `remote_exists()`.

## Estimated Scope

Single PR. After the Round-2 scope widening, six files change behavior
(`sync_repos.py`, `merge_pr.sh`, `pull_remote.py`, `validate_workspace.py`,
`lib/remote_utils.py`, `push_remote.py`), one consumer follows
(`dashboard.sh`), `validate_workspace.py` also moves to the shared
`lib/workspace.py` helper, six test modules are added, and `AGENTS.md`, the
`Makefile` help text and the two hook docs get doc updates. One atomic commit
per script, so any single widening commit can be dropped without unpicking the
rest.
