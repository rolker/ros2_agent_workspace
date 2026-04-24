# Plan: plan-task — add guidance on updating plan during implementation

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/449

## Context

The `plan-task` skill generates a work plan up through "commit the plan" and
"open draft PR" (steps 6 and 7) but says nothing about what happens to that plan
during implementation. In practice, implementation deviates from the plan
constantly — types change, modules get renamed, a function originally spec'd as
"pure logic" picks up a ROS dependency. Without explicit guidance, the plan
rots, and Copilot's plan-file review flags the drift as a finding, which has to
be triaged as a false positive each round.

This was concretely observed during `rolker/rqt_operator_tools#22`: across
review rounds 1–8, Copilot repeatedly flagged `PLAN_ISSUE-20.md` wording that
had been deliberately superseded by implementation choices. Round 8 (comment on
`PLAN_ISSUE-20.md:108`: "pure logic, no Qt/ROS" vs. actual `rclcpp::Time`) was
the most visible — the dogfooded fix in commit `9680d3e` landed four inline
plan edits alongside the code changes that motivated them, and that pattern is
what the skill should describe.

## Approach

Three small, tightly related doc edits. Single PR.

1. **Add "During implementation" subsection to `.claude/skills/plan-task/SKILL.md`** —
   insert between current step 7 (create/update draft PR) and step 8 (report to
   user). Content: four rules (inline default, inline-plus-notes for
   rationale-bearing deviations, never append-only, commit discipline), one
   short worked example pair.

2. **Add a one-line reference in `AGENTS.md` § Post-Task Verification**,
   alongside the existing work-plan bullet in that section, pointing at
   the new skill subsection so non-Claude agents who adopt plan-first
   later discover the rule. This prevents the cascade barrier flagged in
   the review-issue comment.

3. **Acknowledge the enforcement-layer choice in the skill prose** — one
   sentence stating plan freshness is enforced by review feedback
   (Copilot, `review-code`), not a local hook. Prevents future maintainers
   from "fixing the gap" with a premature check.

## Files to Change

| File | Change |
|------|--------|
| `.claude/skills/plan-task/SKILL.md` | Insert new "During implementation" subsection with four rules + worked example pair + enforcement-layer note |
| `AGENTS.md` | One-line reference to the new subsection, added under `## Post-Task Verification` alongside the existing work-plan bullet |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Only what's needed | Two files, ~40 lines of new prose total. No new enforcement layer, no new skill file, no restructuring of existing content. |
| Capture decisions, not just implementations | The rule codifies a decision reached on PR #22; worked example points at commit `9680d3e` rather than an invented diff. |
| Enforcement over documentation | Acknowledged explicitly in the skill prose: this is instruction-layer guidance, with Copilot's plan-file review acting as the de-facto signal. No hook is appropriate (checking "was the plan updated when it should be" is not mechanically tractable). |
| A change includes its consequences | AGENTS.md update included; no other adapter files need updating (they reference the skill by name, not its internal steps). `AGENT_ONBOARDING.md` / `gemini-cli.instructions.md` already list `plan-task` as a skill name — no content edit needed there. |
| Workspace improvements cascade to projects | AGENTS.md entry addresses the cascade barrier — the rule becomes discoverable from shared instructions, not locked inside a Claude-only skill. |
| Improve incrementally | Single small PR, one feature (the rule), no refactoring. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0004 — Enforcement hierarchy | Yes | Instruction-layer only, by explicit design. Skill prose will state this so the choice isn't mistaken for an oversight. Review feedback (Copilot, `review-code`) is the de-facto enforcement signal; a hook isn't practical. |
| 0006 — Shared AGENTS.md | Yes | AGENTS.md gets the one-line reference so the rule is discoverable framework-agnostically, even though the detailed guidance lives in the Claude skill. |
| 0001, 0002, 0003, 0005, 0007, 0008, 0009, 0010 | No | Not applicable — no design decision requiring a new ADR, no worktree / project-agnostic / Make / ROS / Python / git-bug scope. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| A framework skill (`.claude/skills/plan-task/`) | That framework's adapter file; regenerate skills if needed | N/A — no non-Claude adapter references this skill's internal steps; they reference it by name only. `make generate-skills` regenerates Makefile-driven slash commands, not skill content — not applicable here. |
| `AGENTS.md` | Framework adapters if affected | Yes — checked. The one-line reference is pointer-only (no new rule), so adapters don't need mirrored content. |

## Open Questions

Both items below were resolved during implementation; annotations reflect
the decisions that landed.

- **Name of the new section inside `.claude/skills/plan-task/SKILL.md`** —
  Resolved: `## During implementation` top-level section placed after
  `## Steps` (i.e. after step 8), peer to `## Guidelines`, with
  `### Worked example` as a subsection. Keeps the 8 numbered steps
  under `## Steps` internally consistent (no mixed numbered/unnumbered
  peers) and reads honestly as guidance for the post-skill
  implementation phase. (Landed through several iterations: first
  `##` inserted between steps which broke nesting; then `###`
  unnumbered inside Steps which Copilot flagged as inconsistent with
  the numbered peers; finally moved to its current after-Steps
  location.)
- **Depth of the worked example** — Resolved: short before/after excerpt
  pasted inline, plus a SHA reference to `rolker/rqt_operator_tools@9680d3e`.
  Readers see the pattern without clicking through; the SHA anchors the
  example to a real commit for anyone who wants full context.

## Estimated Scope

Single PR. ~40 lines of new/changed content across two files. No tests
(instruction doc). No dependency regeneration needed.

---
**Authored-By**: `Claude Code Agent`
**Model**: `Claude Opus 4.7 (1M context)`
