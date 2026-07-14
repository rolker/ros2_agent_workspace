# Plan: Project-repo root AGENTS.md for Copilot code review

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/563

## Context

GitHub Copilot code review reads root-level `AGENTS.md` (since 2026-06-18). Zero
first-party project repos currently have one, so Copilot reviews their PRs uninstructed.
A `triage-reviews` baseline counts ~200 false-positive classifications across 18 repos,
with the top 7 totaling 158 FPs. The closest-AGENTS.md-to-file-wins rule also means this
file becomes the first instruction any coding agent sees in a layer worktree.

This workspace PR delivers:
1. Template `.agent/templates/project_agents_md.md` — the canonical starting point
2. Skill updates — `onboard-project` adds AGENTS.md to its checklist; `audit-project`
   checks presence and Quality-Standard currency
3. New thin ADR-0017 recording the extension to project-repo level and the
   "reference, never fork" invariant, plus a purely navigational ADR-0012
   cross-reference addendum on ADR-0006 pointing to it

Pilot rollout (7 repos getting actual `AGENTS.md` files) is a separate follow-up stacked
on this issue — out of scope for this PR.

## Approach

1. **Create template** `.agent/templates/project_agents_md.md` — 40–60 lines with:
   - Workspace rules pointer (reference, never fork — points to workspace `AGENTS.md`)
   - Standalone context block so Copilot reviewers on project repos get useful guidance
     without workspace access (Quality Standard, plan-sync convention, ROS 2 field-deployment context)
   - Repo-specific review context slot (placeholder filled per-repo in pilot PRs)
   - Pointer to `.agents/README.md` for the deep agent guide

2. **Add ADR-0017** (`docs/decisions/0017-extend-agents-md-to-project-repos.md`) — thin
   ADR recording the substantive decision: extend the two-tier AGENTS.md pattern
   (ADR-0006) to project-repo level; per-repo files **reference, never fork**, workspace
   rules; the file serves both coding agents (closest-file-wins) and Copilot code review.
   Then add a **purely navigational** cross-reference addendum on ADR-0006 (Status note +
   References entry pointing at ADR-0017) per ADR-0012.

3. **Update `onboard-project` skill** (`.claude/skills/onboard-project/SKILL.md`):
   - Add "Root `AGENTS.md`" row to the checklist table with check = file exists at repo root,
     fix approach = copy from `.agent/templates/project_agents_md.md`
   - Insert in ordered presentation (after "Agent guide", before "License headers")
   - Add an apply-fixes bullet in step 5

4. **Update `audit-project` skill** (`.claude/skills/audit-project/SKILL.md`):
   - Add `AGENTS.md` row to the governance coverage table (step 2 of the skill) and the
     report-format table
   - In the recommended actions section, flag absence as a distinct action item
   - Add a currency check keyed on the template's stable `## Quality Standard` heading
     (not a prose string search)

5. **Sync `.agent/templates/project_governance.md`** — add root `AGENTS.md` to the suggested
   structure, file table, and adoption levels so the governance template and `audit-project`
   stay consistent

## Files to Change

| File | Change |
|------|--------|
| `.agent/templates/project_agents_md.md` | New template (40–60 lines) |
| `docs/decisions/0017-extend-agents-md-to-project-repos.md` | New thin ADR (the substantive decision) |
| `docs/decisions/0006-adopt-agents-md-as-shared-instruction-file.md` | Navigational cross-reference addendum (ADR-0012) pointing at ADR-0017 |
| `.claude/skills/onboard-project/SKILL.md` | Add root `AGENTS.md` checklist item |
| `.claude/skills/audit-project/SKILL.md` | Add `AGENTS.md` governance row + currency check |
| `.agent/templates/project_governance.md` | Add root `AGENTS.md` to structure/table/adoption levels |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Rationale is explicit and quantified (FP baseline). Template must include standalone context — Copilot reviewers on project repos should know the field-deployment stakes. |
| Enforcement over documentation | `audit-project` check is the nearest thing to enforcement — flagged prominently as a recommended action when absent. No mechanical hook; adoption relies on skill use. |
| Capture decisions, not just implementations | ADR-0017 records the "reference, never fork" invariant and the extension to project-repo level; a navigational addendum makes it discoverable from ADR-0006. |
| A change includes its consequences | Template changes propagate to both consuming skills and the governance template in the same PR. |
| Only what's needed | Template is purposely thin (~40–60 lines). ADR-0017 is a thin decision record, not a rewrite. Pilot rollout deferred. |
| Workspace vs. project separation | Template lives in workspace; the AGENTS.md files themselves live in project repos. Template references, never forks, workspace rules. |
| Workspace improvements cascade to projects | This is the explicit intent — workspace skill + template, then project-repo adoption. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0001 — Adopt ADRs | Yes — extending AGENTS.md pattern is a design decision | New thin ADR-0017 records the decision (immutability of ADR-0006 preserved) |
| ADR-0003 — Project-agnostic workspace | Yes — template in workspace repo | Template is generic (no project-specific content); placeholder slots filled per-repo in pilots |
| ADR-0006 — Shared AGENTS.md | Directly relevant — pattern extended to project-repo level | ADR-0017 extends it; ADR-0006 gets only a navigational pointer |
| ADR-0012 — Cross-reference addendums | Governs the ADR-0006 addendum | Addendum is purely navigational (Status note + References pointing at ADR-0017) — no decision content added |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Template in `.agent/templates/` | Docs that reference it; skills that use it | Yes — `onboard-project` + `audit-project` + `project_governance.md` all updated (steps 3–5) |
| `docs/decisions/` (new ADR) | Older ADRs it qualifies | Yes — navigational addendum on ADR-0006 (step 2) |
| `AGENTS.md` (workspace) | Framework adapters | Not triggered — we are not changing workspace AGENTS.md |
| `.claude/skills/` SKILL.md files | Nothing — `make generate-skills` only regenerates `/make_*` commands from Makefile `.PHONY`, not SKILL.md content | N/A (removed no-op step per plan review) |

## Open Questions

- [ ] No open questions — plan is review-plan-ready.

## Estimated Scope

Single PR. All changes are in workspace infrastructure (template + skill updates + ADR addendum).
Pilot rollout is a separate follow-up.
