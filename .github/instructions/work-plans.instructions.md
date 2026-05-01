---
name: work-plan-review
description: Instructions for reviewing work plans committed as the first step of feature branches
applies_to:
  - ".agent/work-plans/**"
---

# Work Plan Review Instructions

Files in `.agent/work-plans/` are implementation plans, not code. The
canonical layout is `.agent/work-plans/issue-<N>/plan.md`, with
companion artifacts (progress logs, review output, adversarial findings)
landing alongside under the same per-issue directory. Plans authored
before the directory convention landed remain at the legacy
`PLAN_ISSUE-<N>.md` path; each one has a matching symlink at
`issue-<N>/plan.md` so consumer skills find every plan at the new
path. Treat the new `issue-<N>/plan.md` form as the authoritative path
for new work; the flat `PLAN_ISSUE-<N>.md` filename is legacy
compatibility.

Plans are committed as the first step on a feature branch — often in a
draft PR before implementation begins.

This pattern applies to both the workspace repo and project repos.
Plans live in whichever repo owns the issue being worked on — project
repos maintain their own `.agent/work-plans/` directories following the
same conventions.

## What to Review

- **Scope**: Does the plan address the linked issue? Are there gaps or overreach?
- **Principle alignment**: Check against `docs/PRINCIPLES.md`. Key principles:
  - "Only what's needed" — is the plan adding unnecessary complexity?
  - "Human control and transparency" — does the plan preserve human oversight?
  - "Test what breaks" — will the result be testable/verifiable?
- **ADR compliance**: Check `docs/decisions/` for relevant Architecture Decision Records.
- **Consequences**: Does the plan account for downstream effects (tests, docs, references)?
- **Feasibility**: Are the proposed steps concrete enough to implement?

## What NOT to Review

- Code style, formatting, or linting — these are plans, not code
- Missing implementation — the plan is the deliverable at this stage
- Test coverage — tests come during implementation
