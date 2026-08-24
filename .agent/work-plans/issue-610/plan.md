# Plan: merge_pr.sh reports "CI checks failed" when a repo has no CI configured

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/610

## Context

`merge_pr.sh`'s CI-wait step (`gh pr checks "$PR_NUM" -R "$GH_REPO" --watch
--fail-fast`) treats every non-zero exit the same way — it prints `ERROR: CI
checks failed` and refuses to merge. But `gh pr checks` returns non-zero for
two structurally different situations: a real check failure/pending timeout,
and **no checks configured on the repo at all** (message on stderr: `no
checks reported on the '<branch>' branch`). 26 of 45 project-repo checkouts
(58%) have no `.github/workflows` — so the "no CI configured" path is the
common case, not the exception, and today it always reads as a false
"failed" report that blocks an otherwise-mergeable PR.

ADR-0018 already accepts a full-scope `refs/notes/ci-local` attestation as
merge verification for project-repo PRs, and its own Consequences section
names the exact gap this issue closes: *"`merge_pr.sh` does not yet check
for the attestation before merging."* `merge_pr.sh` never references
`ci-local` today.

The [Issue Review](progress.md) entry additionally found that the
`no checks reported on` string is an **unversioned `gh` CLI message** — not
a documented exit code or `--json` field — so depending on it for the
pass/fail distinction would itself be fragile. This plan avoids that
dependency (see Approach step 1).

## Approach

1. **Detect "no checks configured" via a stable JSON field, not the stderr
   string.** `gh pr view <N> -R <repo> --json statusCheckRollup` returns
   `{"statusCheckRollup":[]}` when no checks exist on the PR (verified live
   against `rolker/mru_transform#40`, a merged zero-workflow PR) and a
   populated array when checks do exist (verified against
   `rolker/ros2_agent_workspace#600`). This is a documented, stable `gh
   --json` field — unlike the stderr string, it's safe to depend on
   directly. Query it **before** deciding whether to call `gh pr checks
   --watch`:
   - Non-empty array → checks are configured; keep today's behavior
     unchanged (`gh pr checks --watch --fail-fast`; non-zero → `ERROR: CI
     checks failed` → exit 1). This path is untouched by this issue.
   - Empty array → no checks are configured at all; skip `gh pr checks`
     entirely (there is nothing to wait for) and move to step 2. The
     stderr-string match is never needed for the empty-vs-failed
     distinction, since we never call `gh pr checks --watch` in this
     branch — the reason it's *possible* to hit that string is now
     structural (we only take the "no checks" path when the JSON already
     told us so), not a parse of the message itself.

2. **When no checks are configured, look for a full-scope `ci-local`
   attestation on the PR's exact head commit.** New sourced helper
   `.agent/scripts/_ci_verification_helpers.sh` (alongside the existing
   `_worktree_helpers.sh` / `field_mode.sh` sourcing convention), exposing
   `ci_local_attestation_status <repo_path> <head_sha>` :
   - Resolves `head_sha` from `gh pr view <N> --json headRefOid` — the
     PR's actual remote head, not whatever the local checkout happens to
     have — so a stale local branch can't produce a false verdict either
     way.
   - Checks `git -C <repo_path> notes --ref=ci-local show <head_sha>`
     **local-first**; if empty, best-effort fetches
     `refs/notes/ci-local` from `origin` into a **scratch ref**
     (`refs/notes/ci-local-merge-check`, deleted after use) and checks
     there too. Never fetches directly onto `refs/notes/ci-local` — that
     would risk clobbering a local, not-yet-pushed attestation made by
     `ci_local.sh` in the same session (append-only is how `ci_local.sh`
     itself treats the ref; the read side must respect that too).
   - Git notes are exact-match on commit hash — `git notes show
     <head_sha>` finds nothing if the only attestation is on an
     **ancestor** commit. That is the intended behavior, not a gap to
     patch: an attestation for an earlier commit isn't evidence for a
     head that may carry unverified changes since. No ancestor-walk
     fallback is implemented (see Open Questions — none; this is decided).
   - A note can hold multiple `---`-separated appended records (partial
     iteration runs, a later full run, etc. — `ci_local.sh` appends,
     never overwrites). Parse **all** blocks and accept if **any** block
     has both `ci-local: pass` (exact — `pass (partial)` does not match)
     and `scope: full` (exact — matches ADR-0018 decision 1 & 2 verbatim:
     full-scope pass is accepted; partial/dirty/`--no-attest` are not).
   - **`upstream.repos` completeness** (ADR-0018 Consequences addendum
     from #577): if the repo has an `upstream.repos` file, a matching
     block is valid only if it carries an `upstream-repo: <dir>@<sha>`
     line for **every** entry in `upstream.repos` (parsed the same way
     `ci_local.sh` does — `python3 -c` + `yaml.safe_load` reading the
     `repositories:` mapping's keys — reusing the parse shape, not
     importing `ci_local.sh` itself). A block missing any entry is
     rejected even if it says `ci-local: pass` / `scope: full` — per
     ADR-0018, that note doesn't describe the environment that would
     actually be verified.
   - Returns one of three verdicts (`attested` / `no-attestation` /
     `stale-attestation-on-ancestor` is not distinguished from
     `no-attestation` — see above) via exit code + a one-line message on
     stdout for the caller to echo.

3. **Wire the verdict into `merge_pr.sh`'s CI-wait block:**
   - `statusCheckRollup` non-empty → unchanged (step 1).
   - Empty + `ci_local_attestation_status` → attested → print `  ✅
     ci-local full-scope attestation found for <sha> (ADR-0018) —
     treating as merge verification.` and proceed (no exit).
   - Empty + no valid attestation → print `  ⚠️  WARNING: <gh_repo> has no
     CI checks configured and no ci-local attestation for <sha> — merging
     with NO automated verification.` to **stderr** (so it survives a
     piped/redirected run) and proceed (no exit). This is the honest
     "third state" the issue asks for: not a failure, not silently
     invisible — a named, loud statement of fact. The merge is **not**
     blocked and `--no-wait` is **not** required to reach this outcome —
     `--no-wait` keeps its one existing meaning (skip the wait when CI is
     known green) and is orthogonal to this whole block, which only runs
     when `--no-wait` is *not* passed.
   - The `no checks reported on` stderr string is no longer parsed or
     depended on anywhere in the new code. It only ever appeared because
     `gh pr checks --watch` was being called in a situation the new
     `statusCheckRollup` check now avoids calling it in at all.

4. **Update ADR-0018 via a cross-reference addendum, not a rewrite.**
   Per [ADR-0012](../../../docs/decisions/0012-permit-cross-reference-addendums-in-adrs.md),
   editing an existing Consequences bullet's *meaning* ("merge_pr.sh does
   not yet check...") is a substantive change, not a navigational one — it
   requires the addendum form, not an in-place rewrite. Add:
   - A Status-line note: *"merge_pr.sh attestation check landed in
     [#610](https://github.com/rolker/ros2_agent_workspace/issues/610)."*
   - An appended `### Addendum (#610)` subsection after Consequences (same
     shape as the precedent in
     [ADR-0013](../../../docs/decisions/0013-progress-md-entry-type-vocabulary.md#addendum-cross-reference-per-adr-0012))
     stating plainly that the named gap is now closed and pointing at the
     new behavior in `merge_pr.sh`, without editing the original
     Consequences bullet's text (it remains an accurate historical record
     of the gap that existed at the time ADR-0018 was written).

5. **Update `AGENTS.md`'s "Merge verification (ADR-0018)" section** (not an
   ADR — ordinary docs, edited directly) to describe all three states
   `merge_pr.sh` now handles: hosted checks configured (wait, gate as
   before), no checks + valid full-scope `ci-local` attestation (proceeds,
   ADR-0018 path), no checks + no attestation (proceeds with a named
   warning, not a block).

6. **Tests** — new `.agent/scripts/tests/test_ci_verification_helpers.sh`,
   following the `test_field_mode.sh` pattern (source the helper directly,
   no subprocess, no network):
   - Fabricate a temp git repo, commit, and exercise
     `ci_local_attestation_status`:
     - No note at all → `no-attestation` verdict.
     - Note present but `scope: partial` → `no-attestation`.
     - Note present but unattested marker / missing `ci-local: pass`
       exact string (e.g. only `pass (partial)`) → `no-attestation`.
     - Note present, `ci-local: pass` + `scope: full`, on the exact head
       commit → `attested`.
     - Multi-block note (partial run appended, then a full run appended)
       → `attested` (proves "any block" parsing, matching `ci_local.sh`'s
       append behavior).
     - Note exists only on an **ancestor** commit (commit, note, then one
       more empty commit) → head lookup finds nothing → `no-attestation`
       (exercises the ADR-0018 "must be the exact head" decision from
       Approach step 2 — this is the test coverage the issue's third
       bullet asks for).
     - Repo with an `upstream.repos` fixture (two entries) and a note with
       only one matching `upstream-repo:` line → `no-attestation` (or a
       distinguishable rejection) despite `ci-local: pass` / `scope:
       full`.
     - Repo with a fetched-from-origin-only attestation (note pushed to a
       bare `origin` remote, no local note) → `attested`, and the local
       `refs/notes/ci-local` ref is verified **untouched** afterward
       (proves the scratch-ref fetch never clobbers local state).
   - `mru_transform`-shaped case, matching the issue's own worked example:
     no `.github/workflows`, no `.agents/` config, and (simulating
     `ci_local.sh`'s documented inability to run there) no `ci-local` note
     at all → `no-attestation` verdict, which step 3's wiring turns into
     the "proceeds with a named warning" outcome. This is the test that
     stands in for "the third state," per the issue's request, since it's
     the path most likely to rot silently.
   - Extend the existing `.agent/scripts/tests/test_merge_pr.sh` with one
     stubbed-`gh` case proving the **detection** step alone: stub `gh pr
     view --json statusCheckRollup` to return `{"statusCheckRollup":[]}`
     and stub `gh pr checks` to a loud sentinel failure; assert the
     sentinel is **never** called (mirrors the existing field-mode
     "GH_WAS_CALLED must not appear" pattern) — proving the empty-rollup
     path never reaches the old stderr-dependent call. This test stops
     short of the live merge step, consistent with the existing file's
     documented scope boundary (comment at the top of `test_merge_pr.sh`).

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/_ci_verification_helpers.sh` (new) | `ci_local_attestation_status()` — note lookup (local + scratch-ref fetch fallback), multi-block parsing, `upstream.repos` completeness check |
| `.agent/scripts/merge_pr.sh` | Source the new helper; replace the CI-wait block with the `statusCheckRollup`-first three-way logic from Approach steps 1–3 |
| `docs/decisions/0018-local-first-ci-verification.md` | Cross-reference addendum only (Status line + appended `### Addendum (#610)` section) — no rewrite of existing Decision/Consequences text |
| `AGENTS.md` | Update "Merge verification (ADR-0018)" section to describe the three states |
| `.agent/scripts/tests/test_ci_verification_helpers.sh` (new) | Hermetic tests per Approach step 6 |
| `.agent/scripts/tests/test_merge_pr.sh` | One additional stubbed-`gh` case proving the empty-rollup detection path never calls `gh pr checks` |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Core fix: the operator now sees an accurate three-way state (checks failed / verified via ci-local / no verification at all) instead of a uniform false "failed." |
| Enforcement over documentation | Closes the exact gap ADR-0018's own Consequences section names — the accepted verification path becomes something `merge_pr.sh` actually consults, not just documents. |
| Capture decisions, not just implementations | ADR-0018 addendum (step 4) + AGENTS.md update (step 5) + this plan's explicit resolution of the three open design questions record the rationale, not just the diff. |
| Test what breaks | New hermetic helper tests (step 6) cover all three verdicts plus the ancestor-commit and upstream.repos edge cases named in the dispatch brief — including the "third state" (`mru_transform`-shaped, no attestation possible) that's most likely to rot silently. |
| Only what's needed | No change to the `statusCheckRollup`-non-empty (checks-configured) path; `--no-wait` semantics untouched; no new flag introduced (the considered `--no-ci-configured` flag is explicitly rejected per the dispatch brief — it would move the ambiguity up a level rather than remove it). |
| Improve incrementally | Single PR, one script + one new helper + docs; scoped to `merge_pr.sh`'s CI-wait block only. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0018 — Local-first CI verification | Yes | Implements the named-but-undone follow-up (attestation check in `merge_pr.sh`); full-scope/partial distinction and `upstream.repos` completeness rule applied exactly as decision 1, 2, and the #577 Consequences addendum specify. New behavior for the *third* state (no hosted checks AND no attestation) is not previously covered by ADR-0018's Decision section — recorded as an addendum (step 4), not asserted as something ADR-0018 already said. |
| 0012 — Permit cross-reference addendums in ADRs | Yes | Governs *how* ADR-0018 is updated (step 4): addendum form, not in-place edit of Consequences prose. |
| 0011 — Field mode | No | Unrelated; the field-mode guard in `merge_pr.sh` already runs before any `gh` call and is untouched. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `merge_pr.sh`'s CI-wait block | ADR-0018 Consequences (stale "does not yet check" bullet) | Yes — step 4, via addendum |
| ADR-0018's documented-but-unenforced gap | `AGENTS.md` "Merge verification (ADR-0018)" section | Yes — step 5 |
| Adds a new sourced helper file | `.agent/scripts/tests/` regression coverage (`make test-scripts` / `make validate`) | Yes — step 6, auto-discovered by `run_script_tests.sh`'s `test_*.sh` glob, no Makefile wiring needed |
| Third-state ("no verification available") becomes a supported, non-blocking outcome | Any downstream tooling or docs that assume `merge_pr.sh` either merges-on-green-CI or refuses | Checked — no other script in `.agent/scripts/` calls `merge_pr.sh` or parses its output programmatically (only human/CI invocation via `make merge-pr`); no follow-up needed |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): `AGENTS.md`'s "Merge verification
  (ADR-0018)" section currently describes only the full-scope-attestation
  path and hosted-CI-required-for-workspace-repo rule; it doesn't mention
  `merge_pr.sh`'s no-checks-configured behavior at all (because that
  behavior didn't exist). Updated in step 5.
- **Agent-instruction candidates**: None — this is a bug fix to an
  existing script's error handling, not a new pattern that generalizes
  beyond `merge_pr.sh` itself.

## Open Questions

None — the three items flagged in the dispatch brief are resolved above:
"full-scope and valid" = ADR-0018's exact `ci-local: pass` + `scope: full`
match plus `upstream.repos` completeness (step 2); the note must be on the
PR's exact head commit, with an ancestor-only note treated as no
attestation, no fallback (step 2); the third state is exercised via a
`mru_transform`-shaped fixture with no note at all (step 6), which is the
honest "no verification was available" outcome from step 3.

## Estimated Scope

Single PR.
