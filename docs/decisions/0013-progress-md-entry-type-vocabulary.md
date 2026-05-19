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
generalises the pattern: every workflow skill should write a typed
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

Canonicalise the `progress.md` entry-type vocabulary as a workspace
decision. Workflow skills writing to `progress.md` MUST use one of the
following entry types:

| Entry type | Writer | Purpose |
|---|---|---|
| `## Issue Review` | `review-issue` | Result of evaluating an issue against principles + ADRs before work begins. |
| `## Plan Authored` | `plan-task` | Marker that a plan was committed and a draft PR opened. References the plan file SHA. |
| `## Plan Review` | `review-plan` | Independent evaluation of a committed plan; verdict + findings checklist. |
| `## Local Review` | `review-code` post-PR | Local pre-merge review of an existing PR. |
| `## Local Review (Pre-Push)` | `review-code` pre-push | Local review of unpushed commits before opening / updating a PR. |
| `## Integrated Review` | `triage-reviews` (post phase B of #470) | Unified findings table integrating local-review entries + GitHub-side reviews; flags cross-source confirmations. |
| `## Implementation` | future `implement` skill | Marker that implementation completed; references final commit SHA. |

### Schema

Every entry begins with:

```markdown
## <Entry Type>
**Status**: complete | partial | failed
**When**: <YYYY-MM-DD HH:MM>
**By**: <agent name> (<model>)
```

After that, skill-specific fields are permitted but should follow
existing precedents (PR/Branch + SHA, verdict, must-fix/suggestion
counts, sources column for integrated entries, etc.). Findings or
actions go in a `### Findings` or `### Actions` sub-section as a
checkbox list — sources for each item should be inline (e.g.,
`- [ ] (suggestion, Copilot R2) …`) when known.

### Consume by entry-type filter

Downstream skills that aggregate or react to prior work (notably the
phase-B redesign of `triage-reviews`) MUST consume prior entries by
**filtering on entry type and the correlation key appropriate to that
type**, not by re-classifying:

| Entry type | Correlation key |
|---|---|
| `## Issue Review` | issue number (no SHA — issue body is the reference) |
| `## Plan Authored`, `## Plan Review` | plan-file SHA (the plan, not the PR head) |
| `## Local Review`, `## Local Review (Pre-Push)`, `## Integrated Review`, `## Implementation` | PR / branch head SHA |

A finding present in both a `## Local Review` at head `<sha>` and a
GitHub-side Copilot review at the same head is a **cross-source
confirmation** — keep both with a sources column, do not collapse.
For entries keyed by issue or plan-file SHA, cross-source
confirmation is by entry type + issue number (e.g., a `## Plan Review`
and a `## Plan Authored` for the same plan SHA agree on a finding).

### Predecessor recognition

`## External Review` is the recognised predecessor of
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
  table and Consequences, which §52-58 of ADR-0012 explicitly excludes
  from the cross-reference-addendum carve-out. The supersession-only
  rule is the intended friction.

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
