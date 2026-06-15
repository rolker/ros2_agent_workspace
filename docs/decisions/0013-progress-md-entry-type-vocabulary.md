# ADR-0013: `progress.md` Entry-Type Vocabulary

## Status

Accepted.

## Context

`progress.md` files at `.agent/work-plans/issue-<N>/progress.md` are
the workspace's per-issue lifecycle timeline. The
[review-code](../../.claude/skills/review-code/SKILL.md) and
[triage-reviews](../../.claude/skills/triage-reviews/SKILL.md) skills
already write entries to these files (step 8 and step 7
respectively). [#470](https://github.com/rolker/ros2_agent_workspace/issues/470)
generalizes the pattern: every workflow skill should write a typed
entry, and downstream skills (notably the integrator redesign of
`triage-reviews`) should consume prior entries as input rather than
re-doing work or losing signal.

Two failure modes motivate this ADR:

1. **Vocabulary drift.** With six skills (`review-issue`, `plan-task`,
   `review-plan`, `review-code`, `triage-reviews`, future `implement`)
   each able to write entries, the entry names have to be stable. Today
   the names live only in skill bodies — review-code writes
   `## Local Review`, triage-reviews writes `## External Review`. A
   future skill author with no central reference is free to invent
   `## Code Review`, `## Triage`, `## Adversarial`, etc., and the
   timeline becomes ungrep-able.

2. **Consume vs re-do.** [#469](https://github.com/rolker/ros2_agent_workspace/issues/469)
   first proposed read-side dedup ("skip already-covered comments")
   and was closed as superseded once the right semantic emerged: keep
   duplicates, surface them as **cross-source confirmations** at
   integration time. That semantic requires consumers to filter prior
   entries by entry type and source.
   `.agent/work-plans/issue-468/progress.md`'s `## Integrated Review`
   entry from PR [#471](https://github.com/rolker/ros2_agent_workspace/pull/471)
   is the worked example.

## Decision

Canonicalize the `progress.md` entry-type vocabulary as a workspace
decision. Workflow skills writing to `progress.md` MUST use one of the
following entry types:

| Entry type | Writer | Purpose |
|---|---|---|
| `## Issue Review` | `review-issue` | Result of evaluating an issue against principles + ADRs before work begins. |
| `## Plan Authored` | `plan-task` | Marker that a plan was committed and a draft PR opened. References the plan-commit SHA. |
| `## Plan Review` | `review-plan` | Independent evaluation of a committed plan; verdict + findings checklist. |
| `## Local Review` | `review-code` post-PR | Local pre-merge review of an existing PR. |
| `## Local Review (Pre-Push)` | `review-code` pre-push | Local review of unpushed commits before opening / updating a PR. |
| `## Integrated Review` | `triage-reviews` (post phase B of #470) | Unified findings table integrating local-review entries + GitHub-side reviews; flags cross-source confirmations. |
| `## External Review` | `triage-reviews` (pre phase B of #470 — *transitional predecessor* of `## Integrated Review`) | Per-PR findings table from a single source. Phase B of [#470](https://github.com/rolker/ros2_agent_workspace/issues/470) renames new writes to `## Integrated Review`; see Predecessor recognition below. |
| `## Implementation` | future `implement` skill | Marker that implementation completed; references final commit SHA. |

### Schema

Every entry begins with:

```markdown
## <Entry Type>
**Status**: complete | partial | failed
**When**: <YYYY-MM-DD HH:MM ±HH:MM>
**By**: <agent name> (<model>)
```

The `**When**` value MUST carry an explicit numeric UTC offset
(e.g. `2026-05-20 01:45 -04:00`, `2026-05-20 05:45 +00:00`). Local-time
display is preferred — agents write the wall-clock time of the host
that produced the entry, and the offset records the rule (including
DST) that was in effect. Bare `HH:MM` without an offset is not
schema-conformant: cross-host handoff and post-DST reads both depend
on the offset being present. Use a numeric offset here rather than a
TZ abbreviation (`EDT`, `PST`, etc.); abbreviations are ambiguous
across regions and silently shift across DST boundaries.

`Z` is accepted as a synonym for `+00:00` (per RFC 3339 / ISO 8601),
so `2026-05-20 05:45 Z` and `2026-05-20 05:45 +00:00` are equivalent.

In the entry's header section (typically near the top, before any
`### Findings` / `### Actions` / `### Notes` sub-sections), every
entry MUST include the canonical correlation-key field for its entry
type, so consumers (notably the phase-B `triage-reviews` integrator)
can filter by entry-type + correlation-key without parsing prose.
Exact position within the header is not constrained — skill-specific
fields like `**Verdict**` may appear before or after the canonical
field; consumers locate by field name, not by line offset.

| Entry type | Required field |
|---|---|
| `## Issue Review` | `**Issue**: #<N>` |
| `## Plan Authored`, `## Plan Review` | `` **Plan**: `<path>` at `<plan-commit-sha>` `` (commit SHA, not blob SHA — see [Consume by entry-type filter](#consume-by-entry-type-filter)) |
| `## Local Review`, `## Local Review (Pre-Push)`, `## Integrated Review`, `## External Review`, `## Implementation` | `` **PR**: #<N> at `<sha>` `` (or `` **Branch**: <name> at `<sha>` `` if no PR exists yet) |

After that, additional skill-specific fields are permitted but should
follow existing precedents (verdict, must-fix/suggestion counts,
sources column for integrated entries, etc.). Findings, actions, or
open questions go in a `### Findings`, `### Actions`, or
`### Open questions` sub-section as a checkbox list — sources for
each item should be inline (e.g., `- [ ] (suggestion, Copilot R2) …`)
when known. `### Open questions` is reserved for decisions that need
human input before implementation (used by `## Plan Authored`);
`### Findings` and `### Actions` are the general-purpose forms.

### Consume by entry-type filter

Downstream skills that aggregate or react to prior work (notably the
phase-B redesign of `triage-reviews`) MUST consume prior entries by
**filtering on entry type and the correlation key appropriate to that
type**, not by re-classifying:

| Entry type | Correlation key |
|---|---|
| `## Issue Review` | issue number (no SHA — issue body is the reference) |
| `## Plan Authored`, `## Plan Review` | plan-commit SHA — the SHA of the commit that last updated `plan.md` (not the PR head, not the file's blob SHA) |
| `## Local Review`, `## Local Review (Pre-Push)`, `## Integrated Review`, `## External Review`, `## Implementation` | PR / branch head SHA |

A finding present in both a `## Local Review` at head `<sha>` and a
GitHub-side Copilot review at the same head is a **cross-source
confirmation** — keep both with a sources column, do not collapse.
More generally, cross-source confirmation is keyed by entry type +
the entry's correlation key (per the table above): issue number for
`## Issue Review`, plan-commit SHA for `## Plan Authored` / `## Plan
Review`, PR/branch head SHA for the others. E.g., a `## Plan Review`
and a `## Plan Authored` for the same plan-commit SHA agree on a
finding.

### Predecessor recognition

`## External Review` is the recognized predecessor of
`## Integrated Review`. Until [#470](https://github.com/rolker/ros2_agent_workspace/issues/470)'s
phase B retires the name, `triage-reviews` continues to write
`## External Review` and existing entries (e.g.,
`.agent/work-plans/issue-452/progress.md`,
`.agent/work-plans/issue-460/progress.md`,
`.agent/work-plans/issue-461/progress.md`) remain as historical
artifacts. Phase B's `triage-reviews` redesign retires the predecessor
name for new entries; no migration of historical files.

## Consequences

**Positive:**
- Vocabulary lives in one place reviewers and skill authors actually
  look (`docs/decisions/`) rather than spread across five SKILL.md
  files where it would silently drift.
- The "consume by entry-type filter" rule unlocks
  [#470](https://github.com/rolker/ros2_agent_workspace/issues/470)
  phase B (triage-reviews-as-integrator) without each consumer
  re-inventing the filter logic.
- Cross-source confirmations are an explicit signal class. The
  worked example in
  `.agent/work-plans/issue-468/progress.md`'s `## Integrated Review`
  shows the shape downstream skills should produce.

**Negative:**
- Adds one decision document to `docs/decisions/`. Mitigation: the
  document is short and the canonical-vocab decision is what makes
  the timeline composable.
- A future workflow skill author has to either pick an existing
  entry type or introduce a new one via a **superseding ADR**.
  [ADR-0012](0012-permit-cross-reference-addendums-in-adrs.md) does
  *not* apply here: adding an entry type changes this ADR's Decision
  table and Consequences, which ADR-0012's
  [§ Still requires a new/superseding ADR (substantive changes)](0012-permit-cross-reference-addendums-in-adrs.md#still-requires-a-newsuperseding-adr-substantive-changes)
  explicitly excludes from the cross-reference-addendum carve-out.
  The supersession-only rule is the intended friction.

## References

- [ADR-0001](0001-adopt-architecture-decision-records.md) — Adopt
  Architecture Decision Records (parent; this ADR captures a workspace
  decision per ADR-0001's rule).
- Issue
  [#470](https://github.com/rolker/ros2_agent_workspace/issues/470) —
  Composable timeline umbrella (this ADR is phase A commit 1).
- Issue
  [#469](https://github.com/rolker/ros2_agent_workspace/issues/469) —
  superseded; this ADR encodes the corrected
  "duplicates-as-cross-source-confirmations" semantics.
- PR
  [#471](https://github.com/rolker/ros2_agent_workspace/pull/471) /
  `.agent/work-plans/issue-468/progress.md` — worked example of
  `## Integrated Review` output format.
- [Principles review guide](../../.agent/knowledge/principles_review_guide.md) — references this ADR in the ADR-applicability table.

### Addendum (cross-reference, per [ADR-0012](0012-permit-cross-reference-addendums-in-adrs.md))

- Issue [#485](https://github.com/rolker/ros2_agent_workspace/issues/485) / PR
  [#486](https://github.com/rolker/ros2_agent_workspace/pull/486) — #470 **phase
  B** landed: `triage-reviews` is now the integrator and **writes
  `## Integrated Review`**. The `## External Review` entry type is now a
  read-only predecessor (recognized on consume; no longer written). This note is
  navigational only — the Decision table and the "Predecessor recognition"
  section are unchanged (a substantive change would require a superseding ADR).
- **Historical-state marker**: the "Predecessor recognition" section above is
  written in the pre-phase-B present tense ("`triage-reviews` continues to write
  `## External Review` … until phase B retires it"). As of #485 that is
  **historical** — phase B has landed and `triage-reviews` writes
  `## Integrated Review`. Read that paragraph as describing the transitional
  state, not current write behavior; this addendum is the current source of
  truth for what gets written.
- Issue [#491](https://github.com/rolker/ros2_agent_workspace/issues/491) — the
  `address-findings` skill (#481 phase C) is now an **additional writer of
  `## Implementation`**: after `triage-reviews`, it works the open
  `## Integrated Review` actions and records a `## Implementation` entry. The
  Decision table's `## Implementation` Writer cell ("future `implement` skill")
  is the original intent and is **unchanged**; this note registers the second
  writer navigationally (no new entry type, no semantics change — a substantive
  change would require a superseding ADR). Consumers filtering by entry type are
  unaffected: `## Implementation` keeps its meaning ("implementation work
  completed; references the commit SHA(s)").
