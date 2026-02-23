# ADR-0004: Enforcement Hierarchy for Agent Compliance

## Status

Accepted

## Context

AI agents don't reliably follow written rules. The workspace learned this incrementally
(issues #103, #125, #178, #241): documentation was ignored, prominent warnings were
marginally better, pre-commit hooks were bypassed. Each enforcement layer was added after
the previous one proved insufficient.

But enforcement isn't only about preventing violations from reaching merge. The workspace
uses a review workflow where PRs are reviewed remotely (e.g., by GitHub Copilot), and a
local agent responds to review comments. Each review round-trip is expensive — it takes
time, costs tokens, and adds noise. The more issues caught locally before pushing, the
fewer cycles wasted in the remote review loop. The enforcement hierarchy serves both
directions: catching violations before they slip through *and* pulling checks closer to
the point of work to shorten feedback loops.

## Decision

Rules that matter must be enforced at multiple layers. No single layer is sufficient.

The enforcement hierarchy from weakest to strongest:

| Layer | Bypassable? | Purpose |
|-------|-------------|---------|
| Instruction files | Yes | Set expectations |
| Pre-commit hooks | Yes | Fast local feedback |
| Environment guardrails | Yes | Prevent common mistakes mechanically |
| CI + branch protection | No | Enforcement at merge time |
| Container isolation | No | Prevention by construction |

Two principles guide where a rule is implemented:

1. Every rule enforced by a weaker layer should also be enforced by a stronger one.
   If a rule only exists in an instruction file, it will eventually be violated.
2. Anything that can be checked locally should be, to avoid discovering it in remote
   review. The tighter the feedback loop, the less time wasted.

Container isolation has been implemented but not yet tested in practice. Its role is
undecided — it may become an optional tool for specific scenarios or part of every
workflow. That decision will be informed by real usage.

## Consequences

**Positive:**
- Defense in depth — no single bypass defeats all enforcement
- Shorter review cycles — local checks catch issues before they reach remote review
- Clear model for where to implement new rules: instruction file for awareness, hook
  for fast feedback, CI for enforcement

**Negative:**
- Some rules are implemented in multiple places, creating maintenance burden
- Adding new enforcement requires changes at multiple layers
- Container isolation adds infrastructure complexity and its role in the workflow is
  still being determined
