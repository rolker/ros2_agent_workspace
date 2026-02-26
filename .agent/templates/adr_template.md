# ADR Template — Reference for Project Repos

When a project repo needs its first ADR, create `docs/decisions/` and use
this template. The PR template's architecture impact checklist naturally
triggers this — when someone checks "ADR created," they need a place to
put it.

## Bootstrapping a new project

1. Create `docs/decisions/` in the project repo
2. Copy the ADR-0001 bootstrap below as `0001-adopt-architecture-decision-records.md`
3. Create the actual decision as ADR-0002+

There's no need to retroactively capture past decisions. Create ADRs going
forward, and backfill specific ones only when someone needs to understand
a past decision.

## When to write an ADR

Ask: "would reverting this cause problems if someone didn't know why it
was done this way?" If yes, write an ADR. If it's a routine implementation
choice, skip it.

---

## ADR-0001 bootstrap (copy into project repo)

```markdown
# ADR-0001: Adopt Architecture Decision Records

## Status

Accepted

## Context

Design decisions in this project risk being lost in issue comments, PR
descriptions, and commit history. When the rationale behind a decision
isn't captured in a discoverable location, future contributors may
accidentally revert it or make conflicting choices.

## Decision

Adopt Architecture Decision Records using a minimal format.

- Store in `docs/decisions/`, numbered sequentially (0001, 0002, ...)
- Plain markdown files, no tooling
- Each ADR is short: Context, Decision, Consequences
- ADRs are immutable once accepted — supersede rather than edit

## Consequences

**Positive:**
- Design decisions become discoverable
- Reduces the risk of accidental reversion

**Negative:**
- One more thing to maintain (mitigated by keeping the bar at
  "would reverting this cause problems?")
```

## Blank ADR template

```markdown
# ADR-NNNN: Title

## Status

Proposed | Accepted | Superseded by ADR-NNNN

## Context

<!-- What is the issue? Why does a decision need to be made? -->

## Decision

<!-- What was decided and why? -->

## Consequences

**Positive:**
-

**Negative:**
-
```
