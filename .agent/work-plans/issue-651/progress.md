---
issue: 651
---

# Issue #651 — Sweep health doc: make audit coverage explicit and mandatory; never render Resolved for a section not re-examined

## Issue Review
**Status**: complete
**When**: 2026-09-22 08:15 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #651
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Actions
- [ ] Name a verification approach for the coverage-as-data behavior in the plan — these are prose skill files with no automated test harness; propose replaying the 2026-09-14 vs 2026-09-21 sweep contrast from the issue body as a manual check that the shallower run would now render `Not re-examined` instead of silently omitting findings.
- [ ] Clarify in the plan which `audit-project` sections genuinely need a new coverage line (item 4) vs. which already report coverage implicitly (its SKILL.md already frames itself as producing a "coverage report").
- [ ] Distinguish the new per-check coverage line (item 2) from the sweep's existing top-line `Checks: X of 4 completed` metric in `janitor-sweep` SKILL.md, so the plan doesn't read as duplicating that existing line.

## Plan Authored
**Status**: complete
**When**: 2026-09-22 08:19 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-651/plan.md` at `0e24bdb`
**Branch**: feature/issue-651 at `0e24bdb`
**Phases**: single

### Open questions
- [ ] Whether the audit-workspace Coverage line goes in per-section tables (new column) or a separate summary table — implementation detail, either satisfies the issue.

## Plan Review
**Status**: complete
**When**: 2026-09-22 08:22 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-651/plan.md` at `0e24bdb`
**PR**: PR-less
**Verdict**: approve-with-suggestions

### Findings
- [ ] (suggestion) `audit-project`'s § 3 "Check agent guide quality" is silently dropped from the coverage-line convention — the Files to Change table (line 164) lists only "§ 2, 4, 5, 6, 8" and the Approach step-4 text (lines 133–157) never mentions § 3, with no stated rationale (unlike § 7, which explicitly says why it needs no change). § 3's checks (expected sections present, no empty sections, listed paths exist, package inventory matches) are a fixed checklist like §§ 2/6/8, so it's unclear whether the omission is deliberate (conditional on `.agents/README.md` existing) or an oversight — `plan.md:133-164`
- [ ] (suggestion) The claim that an empty `Not re-examined` subsection is omitted "matching the existing rule for empty subsections" (`plan.md:126-131`) overstates current `janitor-sweep` behavior: the skill only omits an entire *tier* when it has zero findings across all subsections (§ 6, "A tier with no findings this run is omitted... not rendered as an empty heading"); there is no existing rule for omitting one of `New`/`Resolved`/`Unchanged` individually while its siblings still have entries — the closest precedent (lines ~573–577/716) is the special-cased "first run" narrative, not a general per-subsection rule. Restate this as new, deliberate behavior for the 4th state, and consider whether `New`/`Resolved`/`Unchanged` should get the same per-subsection omission for consistency, rather than citing a precedent that doesn't exist in the current text.

### Verification notes
Cross-checked the plan's section/step references against the current text of
all three target files — `.claude/skills/audit-workspace/SKILL.md` (7
checklist sections, matches plan's 1–2 sampling / 3–7 exhaustive split),
`.claude/skills/janitor-sweep/SKILL.md` (§ 3/5/6 references and the
`Checks: X of 4 completed` vs. per-check coverage distinction both check
out), and `.claude/skills/audit-project/SKILL.md` (all sections are
exhaustive-by-construction, supporting the plan's "workspace scope only"
restriction on the new `Not re-examined` diff state). Also verified against
the actually-committed `docs/health.md` (2026-09-21 run): its coverage
narrative already carries counts (`2 of 10 principles, 1 of 19 ADRs`) but not
item *names*, and its "Not re-examined this run" note is exactly the
hand-written workaround the issue and plan describe — confirming the plan's
factual premise and the value of the plan's "name which items were examined"
requirement (sections 1–2), which is what makes the coverage-gated diff
matchable at all.

No must-fix findings. The core mechanism — coverage as named, structured
data; a 4th diff state gated on whether the specific item a finding
originated from was re-examined; workspace-scope-only restriction — composes
correctly with the existing New/Resolved/Unchanged logic and the `Checks: X
of 4` line. The two suggestions above are implementation-detail gaps, not
scope or approach problems.
