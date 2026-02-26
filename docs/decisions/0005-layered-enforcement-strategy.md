# ADR-0005: Layered enforcement — framework-agnostic backbone, framework-native fast feedback

## Status

Proposed

## Context

ADR-0004 established that rules must be enforced at multiple layers — no single layer
is sufficient. The open question was: where do framework-specific mechanisms fit
relative to framework-agnostic ones?

This workspace uses multiple AI agent tools — a primary CLI agent for development,
a separate agent for PR reviews, and potentially others as the landscape evolves.
Enforcement cannot depend on any single framework being present.

## Decision

Three layers, each with a clear role:

1. **CI + branch protection** is the enforcement layer. Every rule that matters must
   be checked here. This is framework-agnostic and non-bypassable — it applies equally
   to any agent or a human. If a rule isn't in CI, it's a suggestion, not a rule.

2. **Pre-commit hooks** are the local feedback layer. They catch mistakes at commit
   time for any git user or agent. They're bypassable, so they complement CI rather
   than replacing it.

3. **Framework-native hooks** are the early feedback layer. They catch mistakes closer
   to the point of work — before a commit, before a push, sometimes before a file is
   even written. They're valuable but not load-bearing. If the active framework changes,
   these are rewritten; the rules they check survive in CI.

The relationship: CI is the source of truth. Pre-commit hooks are a fast local mirror
of CI checks. Framework hooks are an even faster mirror of the subset that can be
checked at edit time. Each layer is independently useful; none depends on the others
being present.

New rules should be implemented CI-first. If the feedback loop is too slow, add a
pre-commit hook. If it can be caught even earlier, add a framework hook. Never
implement a rule *only* in a framework hook.

## Consequences

**Positive:**
- Framework changes don't break enforcement — only fast feedback degrades
- All contributors — human or AI, any tool — get the same enforcement
- Clear decision process for new rules: CI first, then pull checks closer to the
  point of work as needed

**Negative:**
- Some checks exist in three places (CI, pre-commit, framework hook), creating
  maintenance overhead
- Framework hooks require framework-specific investment that may not transfer
- CI and branch protection currently mean GitHub Actions and GitHub branch rules.
  This is a platform dependency, but one shared by all other workflows — not specific
  to this enforcement decision
