# Plan: #470 phase B: triage-reviews as progress.md integrator (## Integrated Review)

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/485

## Context

Phase A (#473, merged) added a `## 8. Persist to progress.md` step to the
review/plan skills and established ADR-0013's entry-type vocabulary.
`triage-reviews` still writes single-source `## External Review` entries and
reads only the live GitHub PR reviews. Phase B turns it into an **integrator**:
it reads the prior `progress.md` timeline (local reviews, plan reviews, etc.),
merges those with the GitHub-side reviews, and emits one unified
`## Integrated Review` entry that flags cross-source confirmations. The golden
output shape is the hand-written entry in
`.agent/work-plans/issue-468/progress.md`.

## Approach

1. **Add `.agent/scripts/progress_read.py`** — extract entries from a
   `progress.md` by `## <Entry Type>` heading, emitting JSON (one record per
   entry: type, correlation key, status, when, findings). Python (not bash) for
   robust markdown parsing and native JSON; matches `sync_repos.py`'s precedent.
   Correlation key per ADR-0013's table: issue# (`## Issue Review`), plan-commit
   SHA (`## Plan *`), PR/branch head SHA (review/impl entries). Recognize
   `## External Review` as the predecessor of `## Integrated Review`. JSON shape
   pairs with `fetch_pr_reviews.sh` so the integrator and phase C (#481) consume
   both inputs in one idiom.
2. **Add pytest tests for `progress_read.py`** (run via `.venv`, per ADR-0009) —
   malformed entries, missing correlation key, `Z` vs `+00:00` offset,
   `## External Review` predecessor recognition, multi-entry files. Use
   `.agent/work-plans/issue-468/progress.md` as a golden-input fixture.
3. **triage-reviews step 3** — after fetching PR reviews, also run
   `progress_read.sh` on the issue's progress.md and select prior entries by
   **entry type + correlation key** (head SHA for review entries).
4. **triage-reviews steps 5/6** — add a `sources` column; flag a finding present
   in both a local entry and a GitHub review at the same correlation key as a
   **cross-source confirmation** (keep both, don't collapse). False positives
   listed separately with justification (per the golden reference).
5. **triage-reviews step 7** — rename the persisted entry `## External Review`
   → `## Integrated Review`; add `**Sources**:` and
   `**Cross-source confirmations**:` header fields. Retire the old name for new
   writes; still *read* historical `## External Review` entries.
6. **Update `AGENTS.md`** Script Reference table: add `progress_read.sh`.
7. **Verify** no framework adapter describes triage-reviews' now-stale
   `## External Review` output name.
8. **Add an ADR-0012 cross-reference addendum to ADR-0013** — one line noting
   phase B landed in #485 and new writes are `## Integrated Review`. Do **not**
   alter ADR-0013's Decision table (that would require a superseding ADR). Also
   refresh the predecessor wording in the review guide's ADR-0013 row if it
   reads stale ("transitional, until phase B retires it").

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/progress_read.py` | New — Python entry extractor by type + correlation key, emits JSON |
| `.agent/scripts/tests/test_progress_read.py` | New — pytest parser tests (run via `.venv`) |
| `.claude/skills/triage-reviews/SKILL.md` | Steps 3, 5, 6, 7 (integrate prior entries; sources column; rename entry) |
| `AGENTS.md` | Add `progress_read.py` to Script Reference table |
| `docs/decisions/0013-*.md` | ADR-0012 cross-reference addendum (rename live, links #485); Decision table untouched |
| `.agent/knowledge/principles_review_guide.md` | Refresh ADR-0013 row wording if the predecessor note reads stale |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Capture decisions, not just implementations | The integrator consumes the durable timeline rather than re-doing work — the principle's payoff. |
| A change includes its consequences | AGENTS.md script-table + adapter check are in the plan. |
| Test what breaks | Parser tests authored alongside the helper (step 2), not deferred. |
| Only what's needed | The helper is shared with phase C, so it's justified — keep it a thin extractor, not a framework. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0013 — progress.md entry-type vocabulary | Yes (central) | Implements the consumer side: filter by entry type **+ correlation key**; predecessor recognition; `## Integrated Review` rename; `sources` column is an already-sanctioned skill-specific field. |
| 0012 — cross-reference addendums | Yes | No new entry type/field added (no superseding ADR needed). Step 8 adds a cross-reference addendum to ADR-0013 noting the rename is live — permitted by ADR-0012 because it does not touch the Decision table. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Add a script (`progress_read.py`) | `AGENTS.md` Script Reference table | Yes (step 6) |
| A framework skill (`triage-reviews`) | Framework adapters if they describe its output | Yes — verify (step 7) |
| Edit ADR-0013 (addendum) | Review guide's ADR-0013 row wording | Yes (step 8) |

## Resolved Decisions

- **`progress_read` output → JSON, implemented in Python (`progress_read.py`).**
  Matches `fetch_pr_reviews.sh`'s JSON + the integrator's `jq` pipeline; Python
  gives robust markdown parsing and phase-C reuse. (Was an open question;
  decided 2026-05-25.)
- **ADR-0013 → add an ADR-0012 cross-reference addendum** noting the rename is
  live (links #485), Decision table untouched. Prevents a future reader of the
  ADR from writing the retired `## External Review` name. (Decided 2026-05-25.)

## Estimated Scope

Single PR. One new Python helper + pytest tests, a focused multi-step edit to one
SKILL.md, a one-line AGENTS.md table addition, and an ADR-0013 addendum.
