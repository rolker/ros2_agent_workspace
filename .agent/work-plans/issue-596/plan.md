# Plan: Wire documentation and agent-instruction update review into the per-issue lifecycle

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/596

## Context

The per-issue lifecycle reviews code and governance but never asks "did this
change make documentation stale?" or "did implementation surface a pattern that
belongs in `.agent/knowledge/` or `.agents/README.md`?" The Consequences Map
has no project-code rows for parameters/topics/services → package README, and
neither `plan-task` nor `review-code` explicitly prompt for doc-impact coverage.

The operator-settled design: inject at three existing seams (plan-task template,
review-code governance specialist 5b, Consequences Map), with a verify step on
review-plan. No new lifecycle phase, no auto-editing instruction files, no new
ADR required.

## Approach

1. **Add `## Documentation & Instruction Impact` section to the plan template**
   (`plan-task/SKILL.md` step 5) — required subsection with an explicit "none"
   obligation; lists docs the change makes stale (must go in the same PR) and
   candidate agent-instruction updates (proposals only, operator decides).

2. **Add doc-impact rows to `principles_review_guide.md` Consequences Map** —
   two new rows covering project-code interface changes → package README, and
   implementation surfacing a new pattern → `.agent/knowledge/` / `.agents/README.md`
   candidate (operator approval required before edits).

3. **Update review-code governance specialist (5b)** — extend the consequence
   check description with explicit doc-impact instructions: parameters/topics/
   services changed → check package README and `.agents/review-context.yaml`;
   new pattern surfaced → flag instruction-update candidates the plan missed.

4. **Update review-plan evaluation dimensions (step 4)** — add "Documentation &
   instruction impact" as a dimension to verify the new required plan section
   exists and is non-silent.

5. **Verify framework adapters** — the consequence map row "framework skill →
   adapter file" requires checking `.github/copilot-instructions.md`,
   `.agent/instructions/gemini-cli.instructions.md`, `.agent/AGENT_ONBOARDING.md`.
   All three reference review-code by SKILL.md URL, not by behavior — no
   updates needed. (Confirmed during planning; record for review-plan.)

## Files to Change

| File | Change |
|------|--------|
| `.claude/skills/plan-task/SKILL.md` | Add `## Documentation & Instruction Impact` section to plan template in step 5. The plan line-count guidance that a companion note would touch lives in the **Guidelines** section of `plan-task/SKILL.md` (the 30–80-line bullet; cited by name — numeric line refs went stale twice in this PR alone), not "step 6" — one small required section keeps plans within the 30-80 line target, so no companion note is needed. |
| `.agent/knowledge/principles_review_guide.md` | Add two rows to Consequences Map: interface changes → package README; pattern surfaced → knowledge/README candidate |
| `.claude/skills/review-code/SKILL.md` | Extend 5b consequence check prose with explicit doc-impact and instruction-candidate instructions |
| `.claude/skills/review-plan/SKILL.md` | Add "Documentation & instruction impact" evaluation dimension to step 4 |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Instruction-update candidates are proposals only; operator decides before any edits land. Design explicitly rejects auto-drafting. |
| Enforcement over documentation | The new required plan section is **not** documentation-only — it is backed by the review layer: `review-plan` step-4 gains a "Documentation & instruction impact" dimension that verifies the section exists and is non-silent, and `review-code` specialist 5b gains doc-impact rows. The ADR-0004 enforcement posture is therefore **instruction + review layer** (no hook/CI check, which is the appropriate ceiling for the instruction-file domain). |
| A change includes its consequences | Framework adapters verified (no update needed). review-plan gains the new dimension check. |
| Only what's needed | Three targeted seams; no new lifecycle phase. document-package wiring is out of scope (kept as a "consider" item). |
| Improve incrementally | Additive changes to existing files; no rewrites. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0006 — Shared AGENTS.md | Watch | Changing `.claude/skills/` files. Framework adapters checked — reference skills by URL only, no behavior text needs updating. |
| ADR-0004/0005 — Enforcement hierarchy | Watch | New plan section is enforced at **instruction + review layer** (`review-plan` step-4 dimension + `review-code` 5b doc-impact rows), not documentation-only. No hook/CI check — instruction + review layer is the appropriate posture for the instruction-file domain. |
| ADR-0013 — progress.md entry types | No | No new entry types introduced. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `.claude/skills/plan-task/SKILL.md` | Framework adapters | Yes — verified, no update needed |
| `.claude/skills/review-code/SKILL.md` | Framework adapters | Yes — verified, no update needed |
| `.claude/skills/review-plan/SKILL.md` | Framework adapters | Yes — verified; adapters reference it by URL only, no update needed |
| `.agent/knowledge/principles_review_guide.md` | Skills that reference it (review-issue, review-plan, review-code) | Yes — additive rows; existing consumers are not broken |
| A framework skill | Adapter files | Yes — checked; adapters reference by URL, content unchanged |

## Open Questions

- [ ] No open questions — plan is review-plan-ready.

## Estimated Scope

Single PR. Four files, all additive changes. No code, no tests, no ROS packages.
