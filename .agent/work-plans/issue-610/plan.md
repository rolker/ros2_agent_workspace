# Plan: merge_pr.sh reports "CI checks failed" when a repo has no CI configured

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/610

> **Revision (post-review)**: this plan was revised after the
> [Plan Review](progress.md) returned **changes-requested** with four
> must-fixes and five suggestions. All nine are folded in below; the
> revision landed as its own commit before implementation started, per the
> plan-first workflow. The material changes from the first draft: the
> empty-`statusCheckRollup` case is no longer treated as "no CI configured"
> on its own (Approach step 2), the workspace repo is excluded from the
> substitution path entirely (step 1b), `gh` failures are errors rather than
> empty results (step 1a), the scratch-ref fetch is forced and trapped
> (step 3), the note is pushed at merge time (step 5), the ADR-0018
> addendum stays strictly navigational with the new policy landing in
> `AGENTS.md` (steps 6–7), and the tests assert the three-way *outcomes*,
> not just the absence of the old call (step 8).

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
merge verification for **project-repo** PRs, and its own Consequences
section names the exact gap this issue closes: *"`merge_pr.sh` does not yet
check for the attestation before merging."* `merge_pr.sh` never references
`ci-local` today.

The [Issue Review](progress.md) entry additionally found that the
`no checks reported on` string is an **unversioned `gh` CLI message** — not
a documented exit code or `--json` field — so depending on it for the
pass/fail distinction would itself be fragile. This plan avoids that
dependency.

**What the Plan Review changed about the framing.** An empty
`statusCheckRollup` is *not* equivalent to "this repo has no CI". A repo
that does have CI presents `[]` when:

- the head was pushed moments ago and Actions has not registered check runs
  yet (the exact moment `merge_pr.sh` typically runs);
- every workflow is `paths:`-filtered and none matched this PR's diff
  (`cube_bathymetry` has such a workflow);
- a check suite is queued with no runs attached yet.

Today all three fail **closed** — a misleading message, but no merge. Any
design that reads `[]` as "no CI configured" converts them into a
**fail-open** merge on a warning, on the 42% of repos that *do* have CI.
That would be a worse bug than the one being fixed, so the plan below keeps
the ambiguous case fail-closed and only reaches the ADR-0018 substitution
path when a positive probe says the repo genuinely has no workflows at the
PR head.

## Approach

> **Scope widened after local review round 1 (operator decision).** The plan
> below consults the `ci-local` attestation only for a repo with *no workflows
> at all*. The operator decided to implement **ADR-0018 decision 1 in full**: a
> project-repo PR with a valid full-scope attestation on the exact head
> satisfies the merge gate whether or not the repo also has workflows. The
> precedence that widening rides on — **red hosted checks refuse regardless of
> any attestation**; pending-plus-attestation merges without waiting; pending
> without one still waits; passing is unchanged — is implemented in
> `merge_pr.sh` and documented in `AGENTS.md` § Merge verification, which also
> records that ADR-0018's own text does not settle the attestation-plus-red
> case. Read the states below with that widening applied.

### 1. Replace the CI-wait block with an explicit classification step

Before deciding whether to call `gh pr checks --watch`, query the PR once:

```bash
gh pr view "$PR_NUM" -R "$GH_REPO" --json statusCheckRollup,headRefOid
```

`statusCheckRollup` is a documented, stable `gh --json` field (verified
live: `[]` on `rolker/mru_transform#40`, a merged zero-workflow PR;
populated on `rolker/ros2_agent_workspace#600`). `headRefOid` gives the
PR's actual remote head, so a stale local checkout can't produce a false
verdict in either direction.

**1a. A `gh` failure is an error, never an empty result.** (must-fix 3)
Non-zero exit from `gh pr view`, empty output, or output that does not
parse as JSON → print `ERROR: could not read check status for PR #N
(<repo>) — gh failed; not merging.` and `exit 1`. Auth expiry, network
loss, and rate limiting all land here. This is checked *explicitly*: the
JSON is captured to a variable, `gh`'s exit status is tested, and the
rollup array is extracted with `jq -e` in a separate step so that "jq found
nothing" can never be confused with "the call failed" (`jq` on empty input
exits 4 — a distinction the first draft left implicit).

**1b. The workspace repo never takes the substitution path.** (must-fix 2)
ADR-0018 decision 4 exempts `ros2_agent_workspace`: its hosted checks stay
required, and a `ci-local` attestation is accepted **only** for project-repo
PRs. The first draft's logic was repo-agnostic, so on this very repo an
empty rollup — most likely right after a push, i.e. exactly when the race
fires — would have warned and merged. The gate is
`[[ "$BRANCH_REPO" == "$ROOT_DIR" ]]` (equivalently, an empty `REPO_SLUG`);
`BRANCH_REPO` is already resolved above the CI block for the field-mode
guard. For the workspace repo, an empty rollup goes to the settle/re-poll
of step 2 and then **fails closed** with the workspace-specific message:

```
ERROR: no checks have registered yet for PR #N (rolker/ros2_agent_workspace).
  The workspace repo's hosted checks are required (ADR-0018 decision 4) —
  ci-local attestations do not substitute here. Wait for Actions to start
  and re-run merge-pr.
```

**1c. Non-empty rollup → today's behavior, untouched.** `gh pr checks
"$PR_NUM" -R "$GH_REPO" --watch --fail-fast`; non-zero → `ERROR: CI checks
failed` → `exit 1`. This path is not what the issue is about and does not
change.

### 2. Empty rollup: probe for workflows at the PR head, then settle

(must-fix 1) An empty rollup is *ambiguous*. Resolve it with a positive
probe, not an inference:

```bash
gh api "repos/$GH_REPO/contents/.github/workflows?ref=$HEAD_SHA"
```

- **HTTP 404** → the directory does not exist at this commit → the repo
  genuinely has no CI. (Verified by the reviewer against
  `rolker/mru_transform`.) Proceed to step 3.
- **A file listing** → workflows exist but no check runs have registered.
  Re-poll `statusCheckRollup` a few times with a short sleep
  (default 3 attempts, 10 s apart — a registration race resolves in
  seconds). If checks appear, fall into 1c and watch them normally. If the
  rollup is still empty after the settle window, **fail closed** with an
  honest message that says what is actually happening:

  ```
  ERROR: <repo> has .github/workflows at <sha> but no checks have registered
    for PR #N after <n> polls. Either Actions has not started them yet, or
    every workflow is filtered out for this PR's paths/branches. Not merging:
    a repo with CI must be verified by it. Check the PR's Checks tab (<url>),
    then re-run merge-pr.
  ```

  The settle interval and attempt count come from
  `MERGE_PR_SETTLE_ATTEMPTS` / `MERGE_PR_SETTLE_SECONDS` (defaults 3 / 10)
  so the tests can drive the loop to zero delay without sleeping.
- **Any other failure** (non-zero exit that is not a 404 — auth, network,
  rate limit, 5xx) → error out per 1a. A probe we could not run is not a
  probe that said "no CI".

**Do not use `gh api repos/<repo>/actions/workflows`** for this probe. It
reports `total_count: 2` for `mru_transform`, a repo with no workflow files
at all, because dynamically-registered Copilot reviewer workflows appear
there. A code comment records this so nobody "simplifies" the probe into
the wrong endpoint. The `?ref=<head_sha>` form is what makes the probe
answer the right question — whether *this commit* carries CI, not whether
the default branch does.

### 3. Genuinely no CI: look for a full-scope `ci-local` attestation

New sourced helper `.agent/scripts/_ci_verification_helpers.sh` (alongside
the existing `_worktree_helpers.sh` / `field_mode.sh` sourcing convention),
exposing `ci_local_attestation_status <repo_path> <head_sha>`:

- **Note lookup is local-first.** `git -C <repo_path> notes --ref=ci-local
  show <head_sha>`. Worth stating because it removes a temptation: a
  worktree shares its ref store with `BRANCH_REPO` (the main project
  checkout), so a note `ci_local.sh` wrote inside the feature worktree is
  *already visible* here — the common case needs no fetch at all.
- **Fallback fetch uses a forced refspec and a per-call scratch ref.**
  (must-fix 4) If no local note matches, best-effort fetch from `origin`
  into a scratch ref rather than onto `refs/notes/ci-local` (which could
  clobber a local, not-yet-pushed attestation — `ci_local.sh` treats the
  ref as append-only, and the read side must too):

  ```bash
  scratch=$(_ci_local_scratch_ref)   # per invocation: prefix + PID + RANDOM + ns
  git -C "$repo" update-ref -d "$scratch" 2>/dev/null || true
  git -C "$repo" fetch -q origin "+refs/notes/ci-local:$scratch"
  # ... read the note ...
  git -C "$repo" update-ref -d "$scratch" 2>/dev/null || true
  ```

  **As shipped** (local review round 1): the `trap ... RETURN` became an
  explicit delete after use, and the scratch ref name is built **per call**
  rather than being a constant — with a constant name a concurrent
  `merge_pr.sh` in the same repo deletes the ref mid-fetch and an *attested*
  PR falls through to merge-with-no-verification (reproduced 10/10).

  The `+` and the up-front delete are both load-bearing: the reviewer
  reproduced (rc=1) a leftover scratch ref from an interrupted run making
  the next fetch fail non-fast-forward, which would have yielded a false
  `no-attestation` → a merge on a warning instead of on evidence. The
  non-clobbering property of the scratch ref itself is verified (a local
  unpushed `refs/notes/ci-local` survives the fetch intact).
- **Exact head only.** `git notes --ref=<ref> show <sha>` is keyed by the
  sha path and resolves even when the commit object is absent locally
  (verified), so no ancestor can be matched by accident and no commit
  fetch is needed. An attestation on an ancestor is not evidence for a head
  that may carry unverified changes; there is no ancestor-walk fallback.
- **Multi-block parsing.** A note can hold several `---`-separated appended
  records (`ci_local.sh` appends, never overwrites). Parse **all** blocks
  and accept if **any** block has both `ci-local: pass` (exact — `pass
  (partial)` must not match) and `scope: full` (exact). This is ADR-0018
  decisions 1 and 2 verbatim.
- **`upstream.repos` completeness** (ADR-0018's #577 Consequences
  addendum): a matching block is valid only if it carries an
  `upstream-repo: <dir>@<sha>` line for **every** entry in
  `upstream.repos`. Which copy is parsed matters (review suggestion):
  `BRANCH_REPO` is the main checkout, typically sitting on the default
  branch, so its working-tree file may not be the PR head's. Read it at the
  head commit — `git -C <repo> show <head_sha>:upstream.repos` — parsed the
  same way `ci_local.sh` does (`python3 -c` + `yaml.safe_load` over the
  `repositories:` mapping keys; the parse shape is reused, `ci_local.sh`
  itself is not imported). The completeness check is required whenever
  either the head tree has `upstream.repos` or the candidate note carries
  `upstream-repo:` lines. If the head commit object is not present locally,
  `git show` cannot read it (unlike the note lookup, which can) — the
  verdict is then `no-attestation` with a message saying to
  `git fetch origin` and re-run, never a silent pass.
- **Verdicts**: `attested` (return 0) or `no-attestation` (return 1), each
  with a one-line explanation on stdout for the caller to echo. Ancestor-only
  notes, partial/dirty records, and incomplete `upstream-repo:` coverage all
  land in `no-attestation` — they are not separate verdicts, but the message
  names which condition applied so the operator knows what to do.

### 4. Wire the verdict into `merge_pr.sh`

For a **project repo** whose probe returned 404 (genuinely no CI):

- `attested` → print
  `  ✅ ci-local full-scope attestation found for <sha> (ADR-0018) — treating as merge verification.`
  and proceed.
- `no-attestation` → print to **stderr** (so it survives a piped run):

  ```
  ⚠️  WARNING: <repo> has no CI workflows at <sha> and no full-scope ci-local
     attestation for that commit — merging with NO automated verification.
     To merge on evidence instead: .agent/scripts/ci_local.sh <repo_path>
     on THIS head commit (<sha>), then re-run merge-pr. Note: an attestation
     for an earlier commit does not count; re-run it after every push.
  ```

  and proceed. (Review suggestion: the recourse line is the point. Without
  it, the operator's next move is `--no-wait` — the blunt flag this whole
  issue exists to stop people reaching for.)

  **As shipped** (local review round 1): the recourse names a checkout that
  is actually *on* the head commit, never `BRANCH_REPO` (the main checkout,
  which sits on the default branch — following that would attest the wrong
  commit). `--no-wait`'s meaning did widen: it now skips the attestation
  lookup and its publication as well as the hosted-check wait, and both the
  script header and `make help` say so. And this state now exits **42** with
  a banner (operator decision): it stays warn-and-merge, but must be
  greppable in a transcript.

The `no checks reported on` stderr string is not parsed anywhere in the new
code. It only ever appeared because `gh pr checks --watch` was being called
in a situation this classification now avoids calling it in.

### 5. Push `refs/notes/ci-local` when the note authorized the merge

(Review suggestion; ADR-0018 decision 5.) If the merge proceeds *because
of* a local attestation, the evidence must not stay on the merging machine.
Immediately before the `gh pr merge` step, when and only when step 4
returned `attested`:

```bash
git -C "$BRANCH_REPO" push origin refs/notes/ci-local
```

Failure is a **warning, not an abort** — the verification itself already
happened and the human is watching the merge; the message says to push the
ref manually. Pushing before the merge (rather than after) means a
push failure is visible while the merge is still in the operator's hands.

### 6. ADR-0018: a strictly navigational addendum

(Review suggestion — the first draft's reasoning inverted ADR-0012.)
[ADR-0012](../../../docs/decisions/0012-permit-cross-reference-addendums-in-adrs.md)
permits addendums **because they are navigational**; *substantive* changes
require superseding. So the addendum must not assert new policy inside
ADR-0018. It gets:

- A Status-line note: *"`merge_pr.sh` attestation check landed in
  [#610](https://github.com/rolker/ros2_agent_workspace/issues/610)."*
- An appended `### Addendum (#610)` subsection (same shape as the precedent
  in [ADR-0013](../../../docs/decisions/0013-progress-md-entry-type-vocabulary.md#addendum-cross-reference-per-adr-0012))
  that points at #610 and at `AGENTS.md` for the current behavior — and
  says nothing more.

The original Consequences bullet ("`merge_pr.sh` does not yet check…") is
left untouched: it is an accurate historical record of the gap as it stood
when ADR-0018 was written.

The new **third-state policy** — that a merge may proceed, loudly, when
nothing verifies it — is a consequence ADR-0018 never recorded. It is not
asserted inside ADR-0018; it lands in `AGENTS.md` (step 7). If it later
proves contentious enough to need decision status, that is a superseding
ADR, not an edit.

### 7. `AGENTS.md` — "Merge verification (ADR-0018)"

Extend the section to describe what `merge_pr.sh` now does, in four states:

1. hosted checks present → wait and gate on them, as before;
2. workflows present at the head but no checks registered → **refuse**,
   with the settle/re-poll first (and the workspace repo always lands here
   rather than in 3/4 — its hosted checks are required, ADR-0018
   decision 4);
3. project repo, no workflows at the head, full-scope `ci-local`
   attestation on that exact commit → merge on the attestation, and the
   note is pushed at merge time;
4. project repo, no workflows at the head, no attestation → merge with a
   named warning that no automated verification exists, and how to get one.

### 8. Tests

**New `.agent/scripts/tests/test_ci_verification_helpers.sh`**, following
the `test_field_mode.sh` pattern (source the helper directly; temp git
repos; no network). Fabricate a repo and exercise
`ci_local_attestation_status`:

- no note at all → `no-attestation`;
- `scope: partial` → `no-attestation`;
- `ci-local: pass (partial)` (no unqualified `pass`) → `no-attestation`;
- `ci-local: pass` + `scope: full` on the exact head → `attested`;
- multi-block note (partial appended, then full) → `attested`;
- note on an **ancestor** only → `no-attestation`;
- `upstream.repos` with two entries, note carrying one `upstream-repo:`
  line → `no-attestation` despite `pass`/`full`;
- `upstream.repos` with two entries, note carrying both → `attested`;
- attestation reachable only from a bare `origin` (note pushed there, none
  local) → `attested`, **and** the local `refs/notes/ci-local` is verified
  untouched afterward (the scratch-ref fetch never clobbers local state).
  The bare-`origin` fetch works over a local path with no network
  (verified by the reviewer);
- **leftover scratch ref** at `refs/notes/ci-local-merge-check` pointing at
  unrelated content, plus a valid attestation on `origin` → still
  `attested` (regression test for must-fix 4: the forced refspec and the
  up-front delete are what make this pass; without them the fetch fails
  non-fast-forward and the verdict is a false `no-attestation`).

**Extend `.agent/scripts/tests/test_merge_pr.sh` with the three-way
outcomes**, not merely the absence of the old call. (Review suggestion: the
existing file only asserts `gh pr checks` is never called; the *outcomes*
are what will rot.) The existing fixtures already prove the shape works —
a temp repo under `layers/main/<layer>_ws/src/<slug>` with a github origin,
reached via `--pr <N> --repo-slug <slug>`, with `gh` stubbed on `PATH`. The
stub answers `pr view` (state/headRefName/headRefOid/statusCheckRollup),
`api` (the workflows probe), `pr checks`, and `pr merge`; its behavior is
driven by env vars the test sets, and `gh pr merge` prints a
`MERGE_ATTEMPTED` sentinel and exits non-zero — so each case terminates
right after the CI block, without a real merge, worktree removal, or `make
sync`. `MERGE_PR_SETTLE_ATTEMPTS=1 MERGE_PR_SETTLE_SECONDS=0` keeps the
settle loop instant. Cases:

- empty rollup + probe 404 + no note → warning text on stderr **and**
  `MERGE_ATTEMPTED` present (it proceeds);
- empty rollup + probe 404 + valid full-scope note on the head → the
  attestation message **and** `MERGE_ATTEMPTED` present;
- empty rollup + probe returns a workflow listing → fail-closed: the
  "no checks have registered" error, `MERGE_ATTEMPTED` **absent**;
- **workspace repo** (`--repo-slug workspace`) + empty rollup →
  fail-closed with the ADR-0018-decision-4 message, `MERGE_ATTEMPTED`
  absent (must-fix 2's regression test);
- `gh pr view --json statusCheckRollup` exits non-zero → error, no merge,
  and the output does **not** claim "no CI configured" (must-fix 3's
  regression test);
- non-empty rollup → `gh pr checks` **is** called (the untouched path stays
  untouched).

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/_ci_verification_helpers.sh` (new) | `ci_local_attestation_status()` — local-first note lookup, forced+trapped scratch-ref fetch fallback, multi-block parsing, `upstream.repos` completeness read at the head commit |
| `.agent/scripts/merge_pr.sh` | Source the new helper; replace the CI-wait block with the classification of Approach 1–4; push `refs/notes/ci-local` when the note authorized the merge (5) |
| `docs/decisions/0018-local-first-ci-verification.md` | Navigational addendum only (Status line + `### Addendum (#610)`) — no policy asserted, no existing text rewritten |
| `AGENTS.md` | "Merge verification (ADR-0018)" section → the four states, incl. the third-state policy |
| `.agent/scripts/tests/test_ci_verification_helpers.sh` (new) | Hermetic helper tests per Approach 8 |
| `.agent/scripts/tests/test_merge_pr.sh` | Three-way (six-case) outcome tests per Approach 8 |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Core fix: the operator sees an accurate state — checks failed / checks not registered yet / verified via ci-local / no verification at all — instead of a uniform false "failed", and each message says what to do next. |
| Enforcement over documentation | Closes the exact gap ADR-0018's own Consequences names: the accepted verification path becomes something `merge_pr.sh` consults, and decision 5's note push becomes something it performs. |
| Capture decisions, not just implementations | Navigational ADR-0018 addendum + the third-state policy recorded where policy belongs (`AGENTS.md`) + this plan's revision note. |
| Test what breaks | The fail-open paths the review found are each covered by a named regression test (workspace-repo gate, `gh`-failure, workflows-present-but-empty-rollup, leftover scratch ref). The three-way *outcomes* are asserted, not just the absence of the old call. |
| Only what's needed | The checks-configured path is untouched; `--no-wait` semantics unchanged; no new flag (a `--no-ci-configured` flag would move the ambiguity up a level rather than remove it). |
| Improve incrementally | Single PR: one script + one new helper + docs + two test files. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0018 — Local-first CI verification | Yes | Implements the named-but-undone follow-up (decision 1/2 attestation check, incl. the #577 `upstream.repos` completeness rule); honors **decision 4** by excluding the workspace repo from the substitution path entirely; performs **decision 5**'s note push when the note is what authorized the merge. The third state (no workflows AND no attestation) is new policy ADR-0018 never recorded — landed in `AGENTS.md`, not asserted inside the ADR. |
| 0012 — Permit cross-reference addendums in ADRs | Yes | Governs step 6: the addendum stays navigational (that is *why* it is permitted); substantive change would require superseding, which is why the new policy goes to `AGENTS.md`. |
| 0011 — Field mode | No | Unrelated; the field-mode guard runs before any `gh` call and is untouched. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `merge_pr.sh`'s CI-wait block | ADR-0018's stale "does not yet check" bullet | Yes — step 6, navigational addendum (bullet itself left as history) |
| ADR-0018's documented-but-unenforced gap | `AGENTS.md` "Merge verification (ADR-0018)" | Yes — step 7 |
| Adds a new sourced helper file | `.agent/scripts/tests/` coverage (`make test-scripts` / `make validate`) | Yes — step 8; auto-discovered by `run_script_tests.sh`'s `test_*.sh` glob, no Makefile wiring |
| Third state becomes a supported, non-blocking outcome | Downstream tooling that assumes merge-on-green-or-refuse | Checked — no other script calls `merge_pr.sh` or parses its output (human/`make merge-pr` invocation only) |
| `merge_pr.sh` now pushes `refs/notes/ci-local` | Nothing else — the ref is already the documented push target (ADR-0018 decision 5, `AGENTS.md`) | N/A |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): `AGENTS.md`'s "Merge verification
  (ADR-0018)" section describes the attestation path and the workspace-repo
  exemption but says nothing about `merge_pr.sh`'s behavior when a repo has
  no CI — because that behavior didn't exist. Updated in step 7.
- **Agent-instruction candidates**: one, small — the third-state policy in
  `AGENTS.md` is the instruction; no separate `.agent/knowledge/` page is
  warranted for a single script's error handling.

## Open Questions

None. The review's four must-fixes are resolved in Approach steps 1a, 1b,
2, and 3; the five suggestions are folded into steps 3 (`upstream.repos`
copy, exact-head rationale), 4 (recourse text), 5 (note push), 6 (ADR-0012
reasoning), and 8 (outcome tests).

## Estimated Scope

Single PR.
