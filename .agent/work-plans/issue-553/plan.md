# Plan: Self-contained checkpoints in run-issue

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/553

## Context

The `run-issue` orchestrator raises `AskUserQuestion` checkpoints at key lifecycle
junctures. Currently the Checkpoints section of `.claude/skills/run-issue/SKILL.md`
describes *when* to pause but says nothing about *what context* the dialog must carry.
An operator returning to a checkpoint dialog sees the choice but not the triggering
finding — the *why* lives only in prose above the dialog. During concurrent operation
(e.g. #550 and #466 running together) the operator must scroll back to reconstruct
context that should have been in the dialog itself.

The fix is a targeted guidance update: two new requirements in the Checkpoints section,
no changes to the decision table or the set of checkpoints.

## Approach

1. **Read the current Checkpoints section** — confirm exact line range (~lines
   242-256 in run-issue/SKILL.md; the Publish step begins ~258). Verify against
   the live file since #552 shifted line numbers; do not trust a stale number.

2. **Update the Checkpoints section** — replace the existing section with an expanded
   version that adds, after the introductory line and checkpoint list:

   - **Re-orientation header requirement**: every `AskUserQuestion` call must begin
     with a one-line header **in the `question` field text** (NOT the `header`
     field — that chip is capped at ~12 chars and can't hold this):
     `Issue #N: <title> — phase X of Y; <one-line state>`
     This reloads the operator's context when attention returns without scrollback.
     The SKILL.md guidance must state the `question`-field-not-`header`-chip
     placement explicitly so agents don't try to cram it into the chip.

   - **Finding-embedding requirement**: when a checkpoint is triggered by a review
     finding (`## Plan Review` / `## Local Review (Pre-Push)` / `## Integrated Review`),
     the finding's severity and condensed text must appear verbatim in the `question`
     or option `description` fields — not only in surrounding prose. Option labels stay
     short; the *why* rides in the `description`.

   Both requirements apply to all four mandatory checkpoints. A short worked example
   (or inline template) makes each requirement concrete enough to produce a
   judgment-free dialog.

3. **Verify no collateral change** — confirm the decision table, the checkpoint list
   itself (4 items), the publish step, and all other sections are untouched.

## Files to Change

| File | Change |
|------|--------|
| `.claude/skills/run-issue/SKILL.md` | Expand Checkpoints section with re-orientation header + finding-embedding requirements |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | This change directly strengthens it: operators can adjudicate checkpoints without scrollback |
| Only what's needed | Single section update; no new mechanism, no new checkpoint, no schema change |
| Improve incrementally | Minimal targeted edit; the decision table and checkpoint set are explicitly out of scope |
| A change includes its consequences | Pure guidance doc; the acceptance criterion ("dialog readable in isolation") is human-observable — no automated test applies |
| Enforcement over documentation | This is a doc refinement; enforcement is by human review of live run-issue traces |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0001 (Adopt ADRs) | No | Behavioral refinement to existing skill, not a new design decision |
| ADR-0006 (Shared AGENTS.md) | No | The checkpoint-format change doesn't add/remove skills or change skill descriptions in adapter files |
| ADR-0013 (progress.md vocabulary) | No | No new entry type; change is to checkpoint prose in SKILL.md |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Checkpoints section wording | Adapter files (AGENTS.md, copilot-instructions.md) | No — adapters reference run-issue only at a high level and don't describe checkpoint format |
| Checkpoints section wording | Other sibling skills (#531, #537, #545) | No — scope is run-issue only per issue statement |

## Open Questions

- [x] None. Review-plan's 2 suggestions folded in: corrected the line ref
  (~242-256, verify live post-#552); the re-orientation header goes in the
  `question` field, not the 12-char `header` chip.

## Estimated Scope

Single PR — one file, one section, ~20-30 lines added.
