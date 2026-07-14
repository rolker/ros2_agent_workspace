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
3. ADR-0006 cross-reference addendum (via ADR-0012) recording the extension to project-repo level

Pilot rollout (7 repos getting actual `AGENTS.md` files) is a separate follow-up stacked
on this issue — out of scope for this PR.

## Approach

1. **Create template** `.agent/templates/project_agents_md.md` — 40–60 lines with:
   - Workspace rules pointer (reference, never fork — points to workspace `AGENTS.md`)
   - Standalone context block so Copilot reviewers on project repos get useful guidance
     without workspace access (Quality Standard, plan-sync convention, ROS 2 field-deployment context)
   - Repo-specific review context slot (placeholder filled per-repo in pilot PRs)
   - Pointer to `.agents/README.md` for the deep agent guide

2. **Add ADR-0006 cross-reference addendum** to `docs/decisions/0006-adopt-agents-md-as-shared-instruction-file.md`:
   - Append a References entry noting the extension of the pattern to project-repo level (issue #563)
   - Record the "reference, never fork" invariant as a one-line note in Status
   - This is a cross-reference addendum (navigational) per ADR-0012 — not a substantive rewrite

3. **Update `onboard-project` skill** (`.claude/skills/onboard-project/SKILL.md`):
   - Add "Root `AGENTS.md`" row to the checklist table with check = file exists at repo root,
     fix approach = copy from `.agent/templates/project_agents_md.md`
   - Insert in ordered presentation (after "Agent guide", before "License headers")

4. **Update `audit-project` skill** (`.claude/skills/audit-project/SKILL.md`):
   - Add `AGENTS.md` row to the governance coverage table (step 2 of the skill)
   - In the recommended actions section, flag absence as a distinct action item
   - Add a currency check: does the file reference the Quality Standard? (string search for "nits")

5. **Run `make generate-skills`** — required because `.claude/skills/` SKILL.md files changed
   (CLAUDE.md § Makefile targets)

## Files to Change

| File | Change |
|------|--------|
| `.agent/templates/project_agents_md.md` | New template (40–60 lines) |
| `docs/decisions/0006-adopt-agents-md-as-shared-instruction-file.md` | Cross-reference addendum (ADR-0012) — Status note + References entry |
| `.claude/skills/onboard-project/SKILL.md` | Add root `AGENTS.md` checklist item |
| `.claude/skills/audit-project/SKILL.md` | Add `AGENTS.md` governance row + currency check |
| (generated) `.claude/skills/make_generate-skills/` or equivalent | Updated by `make generate-skills` |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Rationale is explicit and quantified (FP baseline). Template must include standalone context — Copilot reviewers on project repos should know the field-deployment stakes. |
| Enforcement over documentation | `audit-project` check is the nearest thing to enforcement — flagged prominently as a recommended action when absent. No mechanical hook; adoption relies on skill use. |
| Capture decisions, not just implementations | ADR-0006 addendum records the "reference, never fork" invariant and the extension to project-repo level so future reviewers can find it. |
| A change includes its consequences | `make generate-skills` is included in the plan. The consequences map entry for template changes (docs + skills) is covered. |
| Only what's needed | Template is purposely thin (~40–60 lines). No new ADR file — addendum suffices. Pilot rollout deferred. |
| Workspace vs. project separation | Template lives in workspace; the AGENTS.md files themselves live in project repos. Template references, never forks, workspace rules. |
| Workspace improvements cascade to projects | This is the explicit intent — workspace skill + template, then project-repo adoption. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0001 — Adopt ADRs | Yes — extending AGENTS.md pattern is a design decision | ADR-0006 addendum records it via ADR-0012's cross-reference-addendum mechanism |
| ADR-0003 — Project-agnostic workspace | Yes — template in workspace repo | Template is generic (no project-specific content); placeholder slots filled per-repo in pilots |
| ADR-0006 — Shared AGENTS.md | Directly relevant — pattern extended to project-repo level | Addendum to ADR-0006; "reference, never fork" invariant recorded there |
| ADR-0012 — Cross-reference addendums | Governs the ADR-0006 addendum | Addendum is purely navigational (Status note + References) — passes the "would it mislead a reader?" test |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `.claude/skills/` SKILL.md files | Run `make generate-skills` | Yes — step 5 |
| Template in `.agent/templates/` | Docs that reference it; skills that use it | Yes — `onboard-project` + `audit-project` both reference it in their SKILL.md |
| `AGENTS.md` (workspace) | Framework adapters | Not triggered — we are not changing workspace AGENTS.md |

## Open Questions

- [ ] No open questions — plan is review-plan-ready.

## Estimated Scope

Single PR. All changes are in workspace infrastructure (template + skill updates + ADR addendum).
Pilot rollout is a separate follow-up.
