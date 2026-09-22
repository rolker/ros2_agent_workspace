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
- [x] (must-fix) Project-scope exemption rests on a false premise: `audit-project`'s coverage is NOT always `N of N` — its own new lines admit `0 of 1`, `0 of 4 — SKIPPED`, `2 of 3 — layer: SKIPPED`, `— run: M of N`. A repo audited in `layer` mode then re-audited in `clone` mode drops the layer check, and the prior finding reads `Resolved` — the exact failure #651 closes, left live in project scope while the text asserts it cannot occur. Cross-confirmed by both adversarial lenses and the lead read — `.claude/skills/janitor-sweep/SKILL.md:614-619`, `.claude/skills/audit-project/SKILL.md:200-206`, `.agent/work-plans/issue-651/plan.md:123-129`
- [x] (must-fix) § 5's decision procedure is unfollowable for check-4 (research-digest) findings: they are workspace scope and tier 5 per § 4, but have no `audit-workspace` section, and check 4's Detail string ("the one digest file, read") matches none of the four bullets' patterns — no rule says whether an absent prior digest finding is `Resolved` or `Not re-examined` — `.claude/skills/janitor-sweep/SKILL.md:583-619`
- [x] (must-fix) The fenced Report-Format template hard-codes `principles X of 10` / `ADRs X of 19`, directly contradicting `audit-workspace/SKILL.md:40-41`'s "count what is there, never assume these numbers"; every other placeholder in both files is generic `X of Y` — `.claude/skills/janitor-sweep/SKILL.md:777`
- [x] (suggestion) The section→tier mapping parenthetical enumerates sections 1,2,3,4,5,7 and omits section 6 (instruction/adapter consistency, tier 4 by the § 4 table), silently downgrading it to the always-uncertain fallback — `.claude/skills/janitor-sweep/SKILL.md:589-592`
- [x] (suggestion) `janitor-sweep` is the skill that now makes a *classification decision* on self-reported coverage, but never restates `audit-workspace`'s "reportable, not self-verifying" caveat; a reader of `janitor-sweep` alone is not told an inflated `10 of 10` silently converts a real gap to `Resolved` — `.claude/skills/janitor-sweep/SKILL.md:583-619`
- [x] (suggestion) Never stated which subsections of the previous `docs/health.md` parse back as "the previous run's findings"; a `Not re-examined` entry carried forward verbatim must count as a prior finding next run or it flips to `New` — `.claude/skills/janitor-sweep/SKILL.md:570-582`
- [x] (suggestion) "`audit-workspace` **ends with** a `### Coverage` table" — it sits between `### Summary` and `### Findings`, not at the end — `.claude/skills/janitor-sweep/SKILL.md:445-446`
- [x] (suggestion) "makes the check `FINDINGS` at worst, never `OK`" (441) and "the check is `FINDINGS` at best" (453) are opposite idioms ten lines apart for the same class of condition — `.claude/skills/janitor-sweep/SKILL.md:441,453`
- [x] (suggestion) `audit-workspace`'s `all N` sections have no partial form; `audit-project` defines `N-1 of N — <item>: <reason>` for an unreadable item but `audit-workspace` can only say `all N` or whole-section SKIPPED, and `all N` resolves prior findings — `.claude/skills/audit-workspace/SKILL.md:46-55`
- [x] (suggestion) The `## Projects` check-status `Detail` cell still reads only `<n> repos audited` — none of `audit-project`'s new coverage signal surfaces at rollup level, unlike the Workspace row — `.claude/skills/janitor-sweep/SKILL.md:812-815`
- [x] (suggestion) The `#### Not re-examined` worked example sits under tier 1 (stale worktrees, section 7 — never samples), where the state can essentially never arise; tier 3 (principles) is its natural home — `.claude/skills/janitor-sweep/SKILL.md:793`
- [x] (suggestion) The Coverage table keys rows on section names ("Script references") while the per-section lines and the relayed Detail use kind names ("scripts"); the table's own comment says a consumer keys on the section name — `.claude/skills/audit-workspace/SKILL.md:170-186`
- [x] (suggestion) The plan's Manual Verification (the hand walkthrough of the shallow-run and full-run replay) is not recorded anywhere yet — it is the only verification this change has, and the plan says it lands in the PR description — `.agent/work-plans/issue-651/plan.md` § Manual Verification

### Governance
Principles — Enforcement over documentation: **Watch** (honesty caveat present in `audit-workspace`, absent from the consumer that acts on the number). A change includes its consequences: **Concern** (the project-scope consequence is asserted away rather than handled — must-fix 1). Capture decisions: **Pass** (deliberate no-ADR while the mechanism is still in flux). Only what's needed / Improve incrementally / Workspace vs. project separation: **Pass**. Test what breaks: **Watch** (walkthrough not yet recorded). Workspace improvements cascade to projects: **Watch** (`audit-project` gained the convention but not the protection).
ADRs — ADR-0013 not triggered (not a `progress.md` entry type); ADR-0017 N/A. Consequences map: framework-skill row satisfied (no skill added or removed, so adapter skill lists are unaffected); health-document row N/A (kind and expected location unchanged).
Static analysis: no linter profile for Markdown (per review-code § 4); `git diff --check` clean; `pre-commit` not installed in this worktree.
Plan adherence: in sync. Both `Plan Review` suggestions were addressed in the implementation (the `agent guide` coverage line was added; the empty-subsection rule is restated as new, deliberate behavior). The plan's own step-3 rationale carries the same false premise as must-fix 1 and must be corrected with it.
Out of scope, noted only: `docs/design/planning_document_vocabulary.md:347` still says "no `docs/health.md` exists anywhere yet" — stale since the first health doc was committed on 2026-09-21.

## Implementation
**Status**: complete
**When**: 2026-09-22 08:55 -04:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-651 at `2ef161f`
**Addressed**: `## Local Review (Pre-Push)` (2026-09-22 08:45 -04:00, branch at `65ba7de`), verdict changes-requested — 3 must-fix, 10 suggestions
**Commits**: `8679427`, `08803a2`, `db18c95`, `026704d`, `de744b6`, `77da22f`, `2ef161f`

**Design decision taken from the operator** (must-fix 1): extend the
coverage-gated `Not re-examined` state to **project scope** rather than
restate the limit honestly. The issue states the gate generally — "renders
Resolved only for a finding whose section was fully covered this run" — so
project scope gets the same rule, rendered as a fourth value on its existing
inline per-finding tag (`[Not re-examined]`), keeping its per-finding tag
shape rather than gaining subsections.

### Actions
- [x] (must-fix) Project-scope exemption rests on a false premise — gate now applies in both scopes: `janitor-sweep` § 5 gains a project-scope "a miss is `Resolved` only when this run looked" block keyed on `audit-project`'s `### Coverage` table (`0 of 1`, `N-1 of N`, `2 of 3 — layer: SKIPPED`, `0 of 4 — SKIPPED`, `— run: M of N`), § 6's tag placeholders and rendering prose carry the fourth value, `audit-project` § 2's "never applies" paragraph is replaced with "not sampling is not the same as always complete", and the plan's step 3 / step 4 / Files-to-Change / Consequences / Only-what's-needed rows are rewritten to match — `.claude/skills/janitor-sweep/SKILL.md` § 5, § 6; `.claude/skills/audit-project/SKILL.md:200`; `.agent/work-plans/issue-651/plan.md`
- [x] (must-fix) § 5 unfollowable for check-4 findings — own bullet added: check 4 reads the one digest file whole every run, so its gate is the check's own status (`Resolved` when check 4 completed, `Not re-examined` when SKIPPED/FAILED), with an explicit instruction not to route these through the section bullets or the can't-determine bullet — `.claude/skills/janitor-sweep/SKILL.md` § 5
- [x] (must-fix) Fenced template hard-codes `X of 10` / `X of 19` → generic `X of Y` — `.claude/skills/janitor-sweep/SKILL.md` report template
- [x] (suggestion) Section→tier mapping omitted section 6 — all seven sections now named, with a note that nothing falls through to the can't-determine bullet by omission — `.claude/skills/janitor-sweep/SKILL.md` § 5
- [x] (suggestion) `audit-workspace`'s "reportable, not self-verifying" caveat restated in the consumer that acts on the number, with an implausible coverage row made a `FINDINGS` condition rather than a licence to resolve — `.claude/skills/janitor-sweep/SKILL.md` § 5
- [x] (suggestion) Which subsections of the previous `docs/health.md` are "the previous run's findings" — stated as New + Unchanged + Not re-examined, minus Resolved, with the reason carrying `Not re-examined` forward is load-bearing (otherwise it resets to `New`) — `.claude/skills/janitor-sweep/SKILL.md` § 5
- [x] (suggestion) "`audit-workspace` **ends with** a `### Coverage` table" → "carries a `### Coverage` table, between its `### Summary` and `### Findings` sections"; same correction applied to the new `audit-project` relay text — `.claude/skills/janitor-sweep/SKILL.md` § 3
- [x] (suggestion) "`FINDINGS` at worst" vs "at best" — unified on "at best" (an incomplete section can also leave the check FAILED, so FINDINGS is the ceiling) — `.claude/skills/janitor-sweep/SKILL.md` § 3
- [x] (suggestion) `audit-workspace`'s never-sampling sections had no partial form — added `<kind>: M of N — <item>: <reason>`, the same form `audit-project` uses, because `all N` is what a consumer resolves prior findings against; § 5's bullets widened from "sampled" to "partially covered" to consume it — `.claude/skills/audit-workspace/SKILL.md` § Coverage
- [x] (suggestion) `## Projects` `Detail` cell now carries per-repo coverage (checkout mode + each repo's folded `### Coverage` table), and check 2's § 3 bullet gains a "relay each repo's coverage too" paragraph — it is the data the project-scope gate keys on — `.claude/skills/janitor-sweep/SKILL.md` § 3, § 6
- [x] (suggestion) `#### Not re-examined` worked example moved from tier 1 to tier 3; tier 1 keeps a short note saying why it is essentially always empty there — `.claude/skills/janitor-sweep/SKILL.md` report template
- [x] (suggestion) Coverage table keyed on section names while the lines use kind names — table gains a `Kind` column carrying the `<kind>` token, and its comment now says a consumer may key on either — `.claude/skills/audit-workspace/SKILL.md` § Report Format
- [x] (suggestion) Manual Verification walkthrough not recorded — written out as `.agent/work-plans/issue-651/verification.md` (case A workspace sampled, case B full coverage + `M of N` shortfall, case C check 4, case D project scope clone mode, plus what it does not establish); the plan's Manual Verification section now points at it — `.agent/work-plans/issue-651/verification.md`

### Ride-along (out-of-scope defect, fixed in the PR where found)
- [x] `docs/design/planning_document_vocabulary.md:347` said no `docs/health.md` exists anywhere yet — stale since the first health doc was committed 2026-09-21. Corrected to state when the first one landed and that the sweep's project scope is still report-only — `docs/design/planning_document_vocabulary.md`

### Deferred actions
- None. All 13 findings were actioned.

### Checks run
Pre-commit hooks ran clean on every commit (no `--no-verify`). No automated
test harness exists for these prose skill files; the hand walkthrough in
`verification.md` is the verification of record. Post-fix consistency greps
confirm no surviving "workspace scope only" / "never occurs there" /
three-state claims about the diff outside the deliberate one about
publishing `docs/health.md`.
