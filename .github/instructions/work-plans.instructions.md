---
name: work-plan-review
description: Instructions for reviewing work plans committed as the first step of feature branches
applies_to:
  - ".agent/work-plans/**/plan.md"
  - ".agent/work-plans/PLAN_ISSUE-*.md"
---

# Work Plan Review Instructions

These instructions apply to **plan files** under `.agent/work-plans/`:
the canonical `.agent/work-plans/issue-<N>/plan.md` form for new work,
and the legacy `PLAN_ISSUE-<N>.md` form for plans authored before the
per-issue directory convention landed. Each legacy plan has a matching
symlink at `issue-<N>/plan.md` so consumer skills find every plan at
the new path; treat the nested form as authoritative for new work.

Companion artifacts (`progress.md`, review output, adversarial
findings, etc.) live in the same per-issue directory but are **not**
plan files — they have their own conventions and are not in scope for
this review skill. The `applies_to` glob above intentionally excludes
them.

Plan files are implementation plans, not code. They are committed as
the first step on a feature branch — often in a draft PR before
implementation begins.

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
