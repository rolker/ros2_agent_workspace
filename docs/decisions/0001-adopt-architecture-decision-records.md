# ADR-0001: Adopt Architecture Decision Records

## Status

Accepted

## Context

Design decisions in this workspace are buried in issue comments, PR descriptions, and
commit history. They are hard to find and at risk of being accidentally reverted. During
the documentation consolidation (#181), intentional per-framework instruction duplication
was nearly undone because the rationale for that decision wasn't captured in a durable,
discoverable location.

The workspace has evolved through many small, isolated fixes. Each solved a real problem,
but without captured rationale the cumulative result is a system where nobody can answer
"why was it done this way?" without archaeology through git history.

## Decision

Adopt Architecture Decision Records using a minimal MADR-inspired format.

- Store in `docs/decisions/`, numbered sequentially (0001, 0002, ...). This is the
  conventional location for ADRs — no reason to deviate from convention here.
- Plain markdown files, no tooling
- Each ADR is short: Context, Decision, Consequences
- ADRs are immutable once accepted — supersede rather than edit
- Workspace-level ADRs live in this repo; project-level ADRs live in their respective
  project repos

## Consequences

**Positive:**
- Design decisions become discoverable — agents and humans can check `docs/decisions/`
  before proposing changes that might conflict with existing decisions
- Reduces the risk of accidental reversion during cleanup or consolidation efforts
- Provides a natural place to record new decisions as the workspace evolves

**Negative:**
- One more thing to maintain — ADRs can go stale if the practice isn't sustained
- Risk of over-documenting trivial decisions (mitigated by keeping the bar at
  "would reverting this cause problems?")
