# ADR-0017: Extend AGENTS.md to project repos (reference, never fork)

## Status

Accepted. Extends [ADR-0006](0006-adopt-agents-md-as-shared-instruction-file.md)
to project-repo level.

## Context

ADR-0006 adopted a two-tier instruction structure at the **workspace** level:
a shared `AGENTS.md` plus thin framework adapters. Project repos were left
with only the `.agents/README.md` agent guide — an onboarding encyclopedia,
not behavioral rules.

Two developments changed the calculus (2026-07 research digest):

1. **GitHub Copilot code review reads a repo's root `AGENTS.md`**
   (2026-06-18) and applies it when generating review feedback. Our project
   repos receive Copilot PR reviews with no instructions — a measured
   baseline of ~200 false-positive review findings across 18 repos, much of
   it noise a short instruction file can address (e.g. recurring plan-drift
   flags).
2. **Closest-AGENTS.md-to-file wins** is the cross-framework convention, so
   a project-repo root `AGENTS.md` is also the first instruction file any
   coding agent sees when working inside a layer worktree.

Without a rule, per-repo files would accumulate copies of workspace rules —
recreating, one level down, exactly the duplication/drift problem ADR-0006
eliminated.

## Decision

1. Project repos in this workspace get a thin root `AGENTS.md` (~40–60
   lines), instantiated from `.agent/templates/project_agents_md.md`.
2. **Reference, never fork**: the per-repo file points to the workspace
   `AGENTS.md` for shared rules and adds only repo-specific content
   (review context, Quality Standard excerpt, pointers). Restating or
   rewording workspace rules in a project AGENTS.md is a defect.
3. The file must carry a standalone context block (Quality Standard,
   plan-sync convention) because one consumer — Copilot code review — has
   no workspace access.
4. `onboard-project` offers the file; `audit-project` checks presence and
   currency (keyed on the template's stable `## Quality Standard` heading).

## Consequences

### Positive

- Copilot code review on project repos is steered by the same standards the
  workspace enforces; false-positive review noise becomes measurable
  (progress.md triage baseline → re-measure after rollout).
- Coding agents in layer worktrees see repo-specific guidance first.

### Negative

- One more per-repo file to keep current; mitigated by the `audit-project`
  currency check rather than manual vigilance.

### Neutral

- Rollout is incremental (pilot repos first, tracked per-repo); repos
  without the file simply behave as before.

## References

- [ADR-0006](0006-adopt-agents-md-as-shared-instruction-file.md) — the
  workspace-level pattern this extends
- [ADR-0003](0003-workspace-infrastructure-is-project-agnostic.md) — the
  template stays generic; repo-specific content is filled per-repo
- Issue [#563](https://github.com/rolker/ros2_agent_workspace/issues/563)
