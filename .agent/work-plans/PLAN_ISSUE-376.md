# Plan: Add parent-issue linking rule to Issue-First Policy

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/376

## Context

When agents create sub-task issues during work on an existing issue, there is no
rule requiring them to reference the parent. This produces orphan issues with no
traceability. The fix is a small addition to the Issue-First Policy in `AGENTS.md`.

## Approach

1. **Add parent-linking paragraph to `AGENTS.md`** — Insert a new paragraph in the
   Issue-First Policy section (after the "Trivial fixes" paragraph, before
   "Verify before committing") with the wording from the issue, plus a note that
   the reference must appear in the issue body (not just a comment).

2. **Verify framework adapters are unaffected** — Confirmed: `.github/copilot-instructions.md`,
   `.agent/instructions/gemini-cli.instructions.md`, and `.agent/AGENT_ONBOARDING.md`
   do not duplicate the Issue-First Policy — they reference `AGENTS.md` by link.
   No adapter changes needed.

## Files to Change

| File | Change |
|------|--------|
| `AGENTS.md` | Add parent-linking rule paragraph to Issue-First Policy section |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Enforcement over documentation | This is documentation-only. Acceptable as an incremental first step; enforcement (e.g., `gh_create_issue.sh` check) can follow in a separate issue. |
| Only what's needed | One paragraph addition solving a concrete pain point. |
| Improve incrementally | Small, single-file change. |
| A change includes its consequences | `AGENTS.md` change → check framework adapters (done, no changes needed). |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0004 — Enforcement hierarchy | Watch | Instructions-layer only for now. Review comment recommends follow-up enforcement. |
| 0006 — Shared AGENTS.md | Yes | Change targets `AGENTS.md` (shared file), not a framework adapter. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `AGENTS.md` | Framework adapters if affected | Yes — verified no changes needed |

## Open Questions

None — the issue is well-specified and the review had no blocking concerns.

## Estimated Scope

Single PR, single file change.
