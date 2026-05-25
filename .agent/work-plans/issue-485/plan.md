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

1. **Add `.agent/scripts/progress_read.sh`** — extract entries from a
   `progress.md` by `## <Entry Type>` heading, emitting JSON (one record per
   entry: type, correlation key, status, when, findings). Correlation key per
   ADR-0013's table: issue# (`## Issue Review`), plan-commit SHA (`## Plan *`),
   PR/branch head SHA (review/impl entries). Recognize `## External Review` as
   the predecessor of `## Integrated Review`. JSON to match `fetch_pr_reviews.sh`
   and so phase C (#481) can reuse it.
2. **Add `.agent/scripts/tests/test_progress_read.sh`** (mirrors downstream's
   test-script pattern) — malformed entries, missing correlation key, `Z` vs
   `+00:00` offset, `## External Review` predecessor recognition, multi-entry files.
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

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/progress_read.sh` | New — entry extractor by type + correlation key (JSON) |
| `.agent/scripts/tests/test_progress_read.sh` | New — parser tests |
| `.claude/skills/triage-reviews/SKILL.md` | Steps 3, 5, 6, 7 (integrate prior entries; sources column; rename entry) |
| `AGENTS.md` | Add `progress_read.sh` to Script Reference table |
| `docs/decisions/0013-*.md` | Only if the status-note open question resolves "yes" (addendum, not Decision-table edit) |

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
| 0012 — cross-reference addendums | Watch | No new entry type/field added, so no superseding ADR needed. If implementation needs one, that's a superseding-ADR gate — not a quiet edit. The status-note (open question) must stay an addendum, not a Decision-table change. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Add a script (`progress_read.sh`) | `AGENTS.md` Script Reference table | Yes (step 6) |
| A framework skill (`triage-reviews`) | Framework adapters if they describe its output | Yes — verify (step 7) |
| ADR-0013 status (rename now live) | Review guide's ADR table (only if ADR edited) | Open question |

## Open Questions

- **ADR-0013 status note**: leave as-is (the ADR already anticipated "post phase B"),
  or add a short ADR-0012 cross-reference addendum noting the rename is live? Must
  not alter the Decision table either way.
- **`progress_read.sh` output**: JSON (proposed, matches `fetch_pr_reviews.sh` + phase-C reuse) vs line-oriented. Confirm JSON.

## Estimated Scope

Single PR. One new script + its test, a focused multi-step edit to one SKILL.md,
and a one-line AGENTS.md table addition.
