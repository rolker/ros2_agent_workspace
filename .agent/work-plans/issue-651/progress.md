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

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-09-22 08:45 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-651 at `65ba7de`
**Mode**: pre-push
**Depth**: Standard (reason: 236 lines across three `.claude/skills/**/SKILL.md` governance files — governance-path override; no security/ADR trigger for Deep)
**Must-fix**: 3 | **Suggestions**: 10
**Round**: 1 | **Ship**: continue — must-fix 1 is a design question (extend the coverage gate to project scope, or restate the limit honestly), not a mechanical edit

### Findings
- [ ] (must-fix) Project-scope exemption rests on a false premise: `audit-project`'s coverage is NOT always `N of N` — its own new lines admit `0 of 1`, `0 of 4 — SKIPPED`, `2 of 3 — layer: SKIPPED`, `— run: M of N`. A repo audited in `layer` mode then re-audited in `clone` mode drops the layer check, and the prior finding reads `Resolved` — the exact failure #651 closes, left live in project scope while the text asserts it cannot occur. Cross-confirmed by both adversarial lenses and the lead read — `.claude/skills/janitor-sweep/SKILL.md:614-619`, `.claude/skills/audit-project/SKILL.md:200-206`, `.agent/work-plans/issue-651/plan.md:123-129`
- [ ] (must-fix) § 5's decision procedure is unfollowable for check-4 (research-digest) findings: they are workspace scope and tier 5 per § 4, but have no `audit-workspace` section, and check 4's Detail string ("the one digest file, read") matches none of the four bullets' patterns — no rule says whether an absent prior digest finding is `Resolved` or `Not re-examined` — `.claude/skills/janitor-sweep/SKILL.md:583-619`
- [ ] (must-fix) The fenced Report-Format template hard-codes `principles X of 10` / `ADRs X of 19`, directly contradicting `audit-workspace/SKILL.md:40-41`'s "count what is there, never assume these numbers"; every other placeholder in both files is generic `X of Y` — `.claude/skills/janitor-sweep/SKILL.md:777`
- [ ] (suggestion) The section→tier mapping parenthetical enumerates sections 1,2,3,4,5,7 and omits section 6 (instruction/adapter consistency, tier 4 by the § 4 table), silently downgrading it to the always-uncertain fallback — `.claude/skills/janitor-sweep/SKILL.md:589-592`
- [ ] (suggestion) `janitor-sweep` is the skill that now makes a *classification decision* on self-reported coverage, but never restates `audit-workspace`'s "reportable, not self-verifying" caveat; a reader of `janitor-sweep` alone is not told an inflated `10 of 10` silently converts a real gap to `Resolved` — `.claude/skills/janitor-sweep/SKILL.md:583-619`
- [ ] (suggestion) Never stated which subsections of the previous `docs/health.md` parse back as "the previous run's findings"; a `Not re-examined` entry carried forward verbatim must count as a prior finding next run or it flips to `New` — `.claude/skills/janitor-sweep/SKILL.md:570-582`
- [ ] (suggestion) "`audit-workspace` **ends with** a `### Coverage` table" — it sits between `### Summary` and `### Findings`, not at the end — `.claude/skills/janitor-sweep/SKILL.md:445-446`
- [ ] (suggestion) "makes the check `FINDINGS` at worst, never `OK`" (441) and "the check is `FINDINGS` at best" (453) are opposite idioms ten lines apart for the same class of condition — `.claude/skills/janitor-sweep/SKILL.md:441,453`
- [ ] (suggestion) `audit-workspace`'s `all N` sections have no partial form; `audit-project` defines `N-1 of N — <item>: <reason>` for an unreadable item but `audit-workspace` can only say `all N` or whole-section SKIPPED, and `all N` resolves prior findings — `.claude/skills/audit-workspace/SKILL.md:46-55`
- [ ] (suggestion) The `## Projects` check-status `Detail` cell still reads only `<n> repos audited` — none of `audit-project`'s new coverage signal surfaces at rollup level, unlike the Workspace row — `.claude/skills/janitor-sweep/SKILL.md:812-815`
- [ ] (suggestion) The `#### Not re-examined` worked example sits under tier 1 (stale worktrees, section 7 — never samples), where the state can essentially never arise; tier 3 (principles) is its natural home — `.claude/skills/janitor-sweep/SKILL.md:793`
- [ ] (suggestion) The Coverage table keys rows on section names ("Script references") while the per-section lines and the relayed Detail use kind names ("scripts"); the table's own comment says a consumer keys on the section name — `.claude/skills/audit-workspace/SKILL.md:170-186`
- [ ] (suggestion) The plan's Manual Verification (the hand walkthrough of the shallow-run and full-run replay) is not recorded anywhere yet — it is the only verification this change has, and the plan says it lands in the PR description — `.agent/work-plans/issue-651/plan.md` § Manual Verification

### Governance
Principles — Enforcement over documentation: **Watch** (honesty caveat present in `audit-workspace`, absent from the consumer that acts on the number). A change includes its consequences: **Concern** (the project-scope consequence is asserted away rather than handled — must-fix 1). Capture decisions: **Pass** (deliberate no-ADR while the mechanism is still in flux). Only what's needed / Improve incrementally / Workspace vs. project separation: **Pass**. Test what breaks: **Watch** (walkthrough not yet recorded). Workspace improvements cascade to projects: **Watch** (`audit-project` gained the convention but not the protection).
ADRs — ADR-0013 not triggered (not a `progress.md` entry type); ADR-0017 N/A. Consequences map: framework-skill row satisfied (no skill added or removed, so adapter skill lists are unaffected); health-document row N/A (kind and expected location unchanged).
Static analysis: no linter profile for Markdown (per review-code § 4); `git diff --check` clean; `pre-commit` not installed in this worktree.
Plan adherence: in sync. Both `Plan Review` suggestions were addressed in the implementation (the `agent guide` coverage line was added; the empty-subsection rule is restated as new, deliberate behavior). The plan's own step-3 rationale carries the same false premise as must-fix 1 and must be corrected with it.
Out of scope, noted only: `docs/design/planning_document_vocabulary.md:347` still says "no `docs/health.md` exists anywhere yet" — stale since the first health doc was committed on 2026-09-21.
