---
name: work-plan-review
description: Instructions for reviewing work plans committed as the first step of feature branches
applies_to:
  - ".agent/work-plans/**"
---

# Work Plan Review Instructions

Files in `.agent/work-plans/` are implementation plans, not code. They follow the
naming convention `PLAN_ISSUE-<N>.md` and are committed as the first step on a
feature branch — often in a draft PR before implementation begins.

## What to Review

- **Scope**: Does the plan address the linked issue? Are there gaps or overreach?
- **Principle alignment**: Check against `docs/PRINCIPLES.md`. Key principles:
  - "Only what's needed" — is the plan adding unnecessary complexity?
  - "Human in the loop" — does the plan preserve human oversight?
  - "Observable behavior" — will the result be testable/verifiable?
- **ADR compliance**: Check `docs/decisions/` for relevant Architecture Decision Records.
- **Consequences**: Does the plan account for downstream effects (tests, docs, references)?
- **Feasibility**: Are the proposed steps concrete enough to implement?

## What NOT to Review

- Code style, formatting, or linting — these are plans, not code
- Missing implementation — the plan is the deliverable at this stage
- Test coverage — tests come during implementation
