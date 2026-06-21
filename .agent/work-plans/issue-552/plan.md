# Plan: dispatch_subagent.sh — host-injected read context + exit-contract robustness on entry replace

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/552

## Context

Two dispatch-contract gaps surfaced driving #466 through `/run-issue` with
container dispatch. Both live in `.agent/scripts/dispatch_subagent.sh` and ship
in one PR (the Issue Review confirms no split).

- **Gap 1 (read-side).** Container dispatch runs headless with no GitHub auth, so
  a phase whose first step is `gh issue view <N>` (e.g. `review-issue`) fails
  outright. Worked around on #466 by hand: `--prompt-file` with the issue body
  embedded — which *abandons* `--skill` (losing auto entry-type + auto model).
  The fix makes host-side context injection first-class and **composable with
  `--skill`**.
- **Gap 2 (exit-contract).** The gate at `dispatch_subagent.sh:440`
  (`POST_COUNT -le PRE_COUNT` → FAILED) assumes append-only growth. A re-dispatch
  that *replaces* a typed entry (e.g. a prior `failed ## Issue Review` → a
  `complete` one) keeps the count flat (1→1) and reports a false FAILED even
  though the phase succeeded. Observed on the #466 review-issue re-run.

Key current facts grounding the design:
- The handoff prompt is composed at `dispatch_subagent.sh:308-354` (`TASK_BODY` →
  `HANDOFF`). `--skill` and `--prompt-file` are mutually exclusive (`:241-246`).
- The gate computes `entry_count()` (`:384-400`) for PRE/POST, then — only after
  passing the count gate — already reads the **last matching entry's**
  `status`/`when`/`by` via `progress_read.py` (`:452-485`). The status routing
  (complete/partial/failed) at `:476-495` is correct; only the **count-delta
  precondition** is wrong.
- `progress_read.py` already exposes `status`, `when`, `when_has_offset`, `by`
  per entry — the signals the new gate needs.
- `entry_count()` lives **below** the source-guard (`:198`), so it is not
  unit-testable; `resolve_progress_file()` lives **above** it and the guard
  comment already anticipates a sourced regression test.

## Approach

### Gap 1 — first-class host-injected context (`--context-file`)

**Where it lives — decision.** The *flag/mechanism* lives in
`dispatch_subagent.sh`; the *fetch* (`gh issue view`) stays in the **caller**
(`run-issue` / the operator). Rationale: keeping `gh` out of the dispatcher
preserves the local-first auth boundary (#532) — the script never needs GitHub
auth, it only injects a file the host already fetched — and a flag benefits
*every* caller (manual dispatch + `run-issue`) and is unit-testable, whereas
fetch logic buried in the `run-issue` markdown would help only that one path.

1. **Add `--context-file <path>`** to arg parsing (`:202-216`) and defaults
   (`:40-48`). Validate existence like `--prompt-file` (`:247-249`).
   **Composable with `--skill`** (unlike `--prompt-file`): you keep
   `--skill review-issue` (auto entry-type, auto model) *and* inject the body.
2. **Inject into the handoff.** When `CONTEXT_FILE` is set, splice a clearly
   delimited section into `HANDOFF` (after `TASK_BODY`, before the handoff
   contract) — e.g. `## Injected GitHub context (issue/PR body, fetched
   host-side)` followed by the file contents and a one-line instruction: *use
   this instead of `gh issue view`; you have no GitHub read auth.*
3. **Transparency (human-control principle).** Log to stderr that context was
   injected and its size / first heading (never the raw token-bearing env), so
   the operator can see what went into the handoff.
4. **Caller docs.** `run-issue` documents fetching the body host-side for
   context-needing phases (`review-issue`, post-PR `triage-reviews`) and passing
   `--context-file`. Pre-push `review-code` / `review-plan` need no GitHub read
   (they work from the diff / local `plan.md`) — explicitly out of scope.

### Gap 2 — gate on last-entry freshness, not raw count delta

1. **Add a sourceable `last_entry_signature()`** helper (above the source-guard,
   next to `resolve_progress_file`) that returns the last matching entry's
   `when|status` (operator decision: `by` dropped) using the *same* match
   predicate (`type == want || base_type == want || startswith(want)`) the count
   gate uses. Empty when no match. Carry a comment that a same-minute/same-status
   re-dispatch yields an identical signature → reads as FAILED (fail-closed,
   accepted — see Open Questions).
2. **Capture PRE signature** alongside `PRE_COUNT`; **capture POST signature**
   alongside `POST_COUNT`.
3. **Replace the count-delta precondition** (`:440`) with a freshness test:
   an entry is *fresh* iff `POST_COUNT > PRE_COUNT` (append case — unchanged) **or**
   `POST_COUNT == PRE_COUNT && POST_SIG != PRE_SIG && POST_SIG non-empty`
   (replace case — the newest typed entry changed `when`/`status`). Only a
   flat count **and** identical signature reads as FAILED ("died before
   reporting"). The existing fail-closed numeric guards (`:403-405`, `:436-439`)
   stay.
4. **Reuse the existing status routing** (`:476-495`) unchanged — once an entry
   is judged fresh, complete/partial/failed reporting already does the right
   thing. The #466 case (`failed`→`complete`, count 1→1) now passes the gate
   because `status` flipped, then reports `OK` via the `complete` branch.
5. **Extract the freshness decision into a small pure function**
   (`is_fresh_entry pre_count post_count pre_sig post_sig`, above the guard) so
   the regression test can exercise it without docker.

### Gap 3 — regression test for the replace scenario

Add sourced unit tests to `test_dispatch_subagent.sh` (it currently only execs
the no-docker surface; the gate is in the container path after the `docker_run`
call, so it can't be reached by execution without docker). Source the script
(the guard at `:198` already supports this), then:
- Write a fixture `progress.md` with **1** `## Issue Review` entry, `**Status**:
  failed`, `**When**: T0`. Capture `pre = last_entry_signature(...)`.
- Overwrite it with **1** `## Issue Review` entry, `**Status**: complete`,
  `**When**: T1` (count stays 1→1). Capture `post`.
- Assert `is_fresh_entry 1 1 "$pre" "$post"` returns true (OK), **not** FAILED.
- Keep an append-case assertion (0→1 fresh) and a genuine no-write assertion
  (1→1, identical signature ⇒ not fresh) so all three branches are covered.

### Gap 4 — docs

- **`review-issue/SKILL.md`** — the read-path auth-dependency note (`:35-42`) and
  step 1 (`:63-67`): note that when the host injects the issue body via
  `dispatch_subagent.sh --context-file`, the phase reads the injected section
  instead of `gh issue view`, satisfying the read path with zero container auth.
- **`AGENTS.md`** script table row (`:470`): add `--context-file <path>`
  (host-injected issue/PR body; container needs no GitHub read auth).
- **`run-issue/SKILL.md`** — two edits: (a) "How phases are dispatched": for
  context-needing phases the host fetches the body and passes `--context-file`
  (the fetch lives in the caller, not the dispatcher); (b) **refresh the
  exit-gate description** at `:102` (and the `startswith` note at `:125`), which
  still say "PRE/POST entry-count delta" — update to the freshness gate
  (count-grow **or** same-count-but-changed-`when|status` signature), per the
  review-plan suggestion. Otherwise the SKILL describes a gate the script no
  longer uses.

### Gap 5 — record the decision (ADR)

Write **a new ADR** (`docs/decisions/0015-dispatch-handoff-context-contract.md`)
capturing the *host-injects-context (read) / container-produces, host-publishes
(write)* dispatch handoff contract — the read-side complement to #532's
write-side framing. See Open Questions for the addendum-vs-new-ADR call.

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/dispatch_subagent.sh` | `--context-file` flag + handoff injection (Gap 1); `last_entry_signature()` + `is_fresh_entry()` helpers above the source-guard and freshness gate replacing the count-delta precondition (Gap 2) |
| `.agent/scripts/test_dispatch_subagent.sh` | Sourced unit tests for the replace / append / no-write gate branches (Gap 3); plus an in-process assertion that `--context-file` content lands in the handoff |
| `.claude/skills/review-issue/SKILL.md` | Read-path note + step 1: host-injection bypass of the `gh issue view` auth dependency |
| `.claude/skills/run-issue/SKILL.md` | Host fetches body + passes `--context-file` for context-needing phases |
| `AGENTS.md` | Script-table row for `dispatch_subagent.sh`: `--context-file` |
| `docs/decisions/0015-dispatch-handoff-context-contract.md` | New ADR: read-side host-injection + write-side host-publish dispatch contract |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control & transparency | Gap 1 logs *what* context was injected; Gap 2 makes the dispatch outcome track reality, so `run-issue` routing isn't poisoned by a false FAILED |
| Enforcement over documentation | Gap 2 hardens a mechanical gate; Gap 3 adds the regression test that locks the replace case |
| Capture decisions, not just code | Gap 5 records the host-injects-context contract as an ADR |
| A change includes its consequences | Docs (review-issue, run-issue, AGENTS.md) + the ADR land in the same PR as the script change |
| Only what's needed | Fetch stays in the caller; the dispatcher gains one flag + one gate fix — no GitHub client added to the script |
| Test what breaks | Gap 3 targets the exact 1→1 false-FAILED that broke #466 |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0001 (adopt ADRs) | Yes | The new dispatch-contract decision is recorded as ADR-0015 |
| ADR-0004 / ADR-0005 (enforcement hierarchy / layered) | Watch | Gap 2 strengthens the script-level gate (the right layer); no reliance on docs alone |
| ADR-0012 (cross-ref addendums) | Yes | Its carve-out is *navigational notes only*; a substantive new decision needs its own ADR — supports the new-ADR recommendation over an ADR-0013 addendum |
| ADR-0013 (progress.md vocabulary) | Yes | Gap 2 consumes `**Status**`/`**When**` per the schema; entry types and the consume-by-type rule are unchanged (no superseding change) |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `dispatch_subagent.sh` flags | AGENTS.md script table | Yes (Gap 4) |
| `--context-file` injection path | `run-issue` (caller fetches + passes it) | Yes (Gap 4) |
| read-path auth story | `review-issue/SKILL.md` read-path note | Yes (Gap 4) |
| exit-contract gate logic | `test_dispatch_subagent.sh` regression | Yes (Gap 3) |
| new dispatch contract | `docs/decisions/0015-*.md` | Yes (Gap 5) |

## Open Questions (RESOLVED by operator, 2026-06-21)

- [x] **ADR placement → new ADR-0015** (not an ADR-0013 addendum). Operator
  confirmed; matches the plan's recommendation (ADR-0012's carve-out is
  navigational-only, so a substantive new decision needs its own ADR).
- [x] **Gap-2 fingerprint → `when|status`** (drop `by`). Operator chose the
  simpler signal. The residual edge — two re-dispatches in the same minute that
  both produce the *same* status — collapses to an identical `when|status`
  signature and so reads as **not fresh (FAILED)**: this is **fail-closed** (the
  safe direction; a genuinely stuck re-dispatch is the far more likely cause of a
  same-minute/same-status flat count). The implementation must carry a code
  comment documenting this accepted trade-off.

## Estimated Scope

Single PR. Suggested atomic commits: (1) Gap-1 `--context-file`; (2) Gap-2 gate
refactor; (3) Gap-3 test; (4) Gap-4 docs; (5) Gap-5 ADR — plus the plan and
progress commits.
