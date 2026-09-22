# Plan: Sweep health doc: make audit coverage explicit and mandatory; never render Resolved for a section not re-examined

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/651

## Context

`audit-workspace` runs inside `janitor-sweep` as a fresh-context sub-agent
each time, and its depth varies run to run — on 2026-09-14 it read all 10
principles and 3 ADRs and found gaps; on 2026-09-21 it spot-checked 2
principles and 1 ADR and reported "no gaps found". The committed
`docs/health.md` diffs each run's findings against the last committed run
and renders `New` / `Resolved` / `Unchanged`. A finding from a section that
simply wasn't looked at this run currently has no distinct rendering — it
either silently vanishes (would misread as `Resolved`) or, today, only
avoids that misreading because a human sweep operator hand-wrote a
"Not re-examined this run" paragraph into `docs/health.md` after the fact
(visible in the current committed file, 2026-09-21 run, "Rules that have
bitten with no enforcement" section). That workaround is exactly the gap
this issue closes: coverage needs to be **data the skills produce**, not a
note an operator remembers to add.

Three files are in scope, all `.claude/skills/` prose/procedure files with
no automated test harness:

- `.claude/skills/audit-workspace/SKILL.md` — report coverage as data,
  per section (issue item 1).
- `.claude/skills/janitor-sweep/SKILL.md` — surface a per-check coverage
  line in the `## Workspace` section (item 2), and use coverage data to
  drive a fourth run-over-run diff state, `Not re-examined` (item 3).
- `.claude/skills/audit-project/SKILL.md` — add the same coverage-line
  convention for symmetry (item 4).

## Approach

1. **`audit-workspace`: define which sections may sample, and require a
   coverage line for every section.**

   Of the 7 checklist sections, only two ever sample rather than examine
   everything: **1. Principles enforcement** (10 principles in
   `docs/PRINCIPLES.md`) and **2. ADR accuracy** (19 ADRs in
   `docs/decisions/`) — both large enough that a full pass is a real time
   cost, which is exactly why depth varied run to run. The other five
   (script reference table, template validity, consequences map, instruction
   file consistency, stale worktrees) each iterate a small, fully-enumerable
   set (`.agent/scripts/`, `.agent/templates/`, the consequences map's own
   item list, a fixed 3-file adapter list, `worktree_list.sh`'s output) —
   sampling is never appropriate there, since covering all of them costs no
   more than sampling would.

   Change the checklist so each section states its coverage explicitly:
   - Sections 1–2 report `<kind>: X of Y examined` (e.g.
     `principles: 2 of 10 examined`, `ADRs: 1 of 19 spot-checked`) **and
     name which items** were examined — a bare fraction with no names is not
     verifiable as a later run's diff input.
   - Sections 3–7 report `<kind>: all N` (e.g. `scripts: all 58`,
     `templates: all 12`) — restating the fixed set size makes an
     incomplete run visible (`all` claimed with a smaller N than the
     directory actually holds is itself a finding) rather than assumed —
     and, when a few items could not be read, the partial form
     `<kind>: M of N — <item>: <reason>` rather than a smaller `all N`
     (added in review round 1: `all N` resolves prior findings, so a
     never-sampling section needed a way to say "all but this one"; it is
     the same partial form `audit-project` already uses).
   - A section that could not be completed at all (an input file missing or
     unreadable) reports `0 of Y` / `SKIPPED(<reason>)`, per the existing
     "never OK for an incomplete section" rule already in the skill — this
     plan makes the *coverage number* the thing that proves incompleteness
     instead of prose.

   Add a dedicated `### Coverage` summary table between `### Summary` and
   `### Findings` — one row per section, always all seven, in a fixed
   order, each row carrying both the section name and the `<kind>` token its
   coverage line uses so the table and the per-section lines join without a
   glossary (the `Kind` column was added in review round 1) — so the coverage numbers are visible in the audit's own output
   as one block `janitor-sweep` can relay and key on, not just implied by
   findings text. (Settled at implementation: a separate table rather than
   a column on each findings table, because the consumer reads it as one
   unit and the Summary table answers a different question — what was
   *found* versus what was *looked at*.)

2. **`janitor-sweep`: per-check coverage line in `## Workspace`, distinct
   from the top-line `Checks: X of 4 completed`.**

   These are different grains and must stay visibly different:
   - **`Checks: X of 4 completed`** (existing, unchanged) — how many of the
     sweep's four *checks* (`audit-workspace`, `audit-project`,
     `issue-triage`, research-digest freshness) ran to completion at all.
   - **New: per-check coverage detail**, in the `## Workspace` section's
     check-status table `Detail` column, for the two rows that section
     already carries (`Workspace governance (audit-workspace)`,
     `Research-digest freshness`). For the `audit-workspace` row, this is
     step 1's coverage data folded into the `Detail` cell — e.g.
     `7 of 7 sections completed; principles 2 of 10 examined (...names...);
     ADRs 1 of 19 spot-checked (...name...); scripts all 58; templates all
     12; ...`. Research-digest freshness has no sampling (it reads one
     file's header every run), so its coverage is trivially "the one file,
     read" — state it for uniformity but this is not new mechanism, just a
     one-line addition to an already-`OK`/`FINDINGS` row.
   - This is a formatting/detail requirement on step 6 (render the report)
     of `janitor-sweep`, consuming step 3's per-check status data — no new
     data collection in `janitor-sweep` itself; it is relaying what
     `audit-workspace` now reports as data (step 1 above).

3. **`janitor-sweep`: fourth run-over-run diff state, `Not re-examined`,
   driven by coverage — not by a human note.**

   Today (§ 5, Run-over-run diff) a finding present in the previous
   committed `docs/health.md` but absent from this run's findings is
   implicitly `Resolved`. Change this: before classifying an absent prior
   finding as `Resolved`, check whether **this run's coverage data covered
   the section that finding originated from**.
   - Full coverage of that section this run (`all N`, or `X of Y` with
     `X == Y`) and the finding is genuinely absent → `Resolved`.
   - The section was only partially covered this run (`X of Y` with
     `X < Y` — a sample, or a never-sampling section's `M of N — <item>:
     <reason>` shortfall) and the specific item the prior finding came from
     was **not** among the items covered → `Not re-examined`, not
     `Resolved`. (If the item *was* covered and the finding is absent now,
     that's a genuine `Resolved` — partial coverage doesn't block resolution
     of the thing actually re-examined.)
   - Check 4 (research-digest freshness) has no `audit-workspace` section
     and so no coverage row. It reads the one digest file whole every run,
     so its gate is the check's own status: an absent prior check-4 finding
     is `Resolved` when check 4 completed, `Not re-examined` when it is
     SKIPPED or FAILED. Stated as its own bullet in `janitor-sweep` § 5 —
     review round 1 found the section-based procedure unfollowable for
     these findings.
   - A finding needs a stable enough description to know which named item
     (principle / ADR) it came from — the existing diff key
     (`tier + check + one-line description`, § 5) already includes the
     description; this plan does not change the key, only what a *miss* on
     that key resolves to when coverage is partial.

   Scope of this new state: **both scopes**, because the gate the issue
   states is general — "renders `Resolved` only for a finding whose section
   was fully covered this run" — and nothing about it is specific to the
   workspace section. A first draft of this plan scoped it to the workspace
   diff only, on the premise that `audit-project`'s sections, never
   sampling, are always `N of N`; review round 1 showed that premise is
   false. `audit-project` does not sample, but its own coverage lines
   legitimately report `0 of 1 — <reason>`, `N-1 of N — <item>: <reason>`,
   `2 of 3 — layer: SKIPPED (no layer checkout)` in `clone` mode, `0 of 4 —
   SKIPPED(<reason>)` for the planning-document probe, and `N of N packages
   — run: M of N` for a partial test pass. The clone-mode case is the
   likeliest of all: a repo audited in `layer` mode and re-audited in
   `clone` mode drops the layer check outright, and its prior finding would
   have read `Resolved` — the exact failure this issue closes, left live in
   project scope.

   The **rendering** still differs by scope, because the two scopes' diff
   histories differ: workspace scope gets a fourth subsection, project scope
   a fourth value on the inline per-finding tag
   (`[New/Resolved/Unchanged/Not re-examined]`), keeping its existing
   per-finding tag shape rather than gaining subsections. Project scope
   reads the gate off `audit-project`'s `### Coverage` table, section by
   section, exactly as the workspace scope reads `audit-workspace`'s.

   Render `Not re-examined` as a fourth subsection alongside `New` /
   `Resolved` / `Unchanged` under each tier in the `## Workspace` section
   (§ 6, report format), replacing today's ad hoc practice of a hand-written
   paragraph after the fact (visible in the current committed
   `docs/health.md`). A tier with no `Not re-examined` findings this run
   omits that subsection. This is **new behavior specific to this
   subsection**, not an existing rule: today only a whole *tier* is omitted
   when empty, and `New`/`Resolved`/`Unchanged` are always rendered inside a
   rendered tier (empty ones say why, as the first-run note does). The
   fourth subsection differs because on a full-coverage run it is empty by
   design, and an always-present empty `Not re-examined` heading would
   suggest partial coverage where there was none.

4. **`audit-project`: same coverage-line convention, for symmetry.**

   `audit-project`'s sections are already exhaustive over what they
   discover (every `package.xml` found, every fixed governance-checklist
   item, all 4 planning-document kinds) — it never samples. The delta here
   is narrower than for `audit-workspace`: no new sampling rule is needed,
   just stating the existing exhaustive counts as explicit coverage data so
   the report format matches `audit-workspace`'s new shape and a reader
   doesn't have to infer "was this exhaustive?" from prose. Add:
   - **Agent Guide** (step 3): a one-item section — `checked` when the
     guide exists and was read, `absent` when there is none (the absence
     is a § 2 finding, not a coverage gap), `0 of 1 — <reason>` when it
     exists but could not be read. Included so all seven sections carry a
     row; the plan-review round flagged its omission from the first draft.
   - **Package Metadata**: `N of N packages checked` (N = packages found by
     the same `package.xml` search step 4 already uses).
   - **Test Status**: `N of N packages checked` (same N).
   - **Governance Coverage** / **Documentation** / **Workspace
     Integration**: these are fixed-size checklists (6, ~4, 3 items) —
     state `all N items checked`, primarily so an item skipped due to a
     read failure shows up as `N-1 of N` rather than silently missing from
     the table.
   - **Planning Documents** (step 7): already reports "4 of 4" implicitly
     by design (the probe always emits all four kinds or is `SKIPPED`
     wholesale) — no format change needed there beyond noting the
     convention is already satisfied.

   The `Not re-examined` diff state **does** reach `audit-project`'s
   per-repo findings (see step 3 above): it is gated on coverage, not on
   sampling, and this audit's coverage is not always full — `0 of 1`,
   `N-1 of N`, `2 of 3 — layer: SKIPPED`, `0 of 4 — SKIPPED`, `— run: M of
   N`. `audit-project` states that in its § 2 coverage-convention paragraph
   so a reader of this skill alone is not told the state cannot apply here.

## Files to Change

| File | Change |
|------|--------|
| `.claude/skills/audit-workspace/SKILL.md` | Checklist sections 1–7 each state coverage (`X of Y examined`/`spot-checked`, naming items, for sections 1–2; `all N` for sections 3–7); add a Coverage column/table to the Report Format |
| `.claude/skills/janitor-sweep/SKILL.md` | § 3 (Run the four checks): note the coverage-line requirement is relayed, not re-collected; § 5 (Run-over-run diff): coverage-gated `Not re-examined` state in **both** scopes (a fourth subsection in workspace scope, a fourth inline tag value in project scope); § 6 (report format): `Detail` column carries per-check coverage; add `Not re-examined` subsection to the `## Workspace` tier template |
| `.claude/skills/audit-project/SKILL.md` | § 2–6, 8 and the Report Format: state exhaustive coverage counts (`N of N`) per section, with § 7's already-satisfied convention noted; a `### Coverage` table at the top of the report |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Enforcement over documentation | This plan makes coverage *reportable as structured data* so the diff mechanism can key off it instead of a human note — a real mechanical improvement over today's ad hoc paragraph. It does **not** make coverage self-verifying: nothing stops a future audit-writing agent from writing `principles: 10 of 10` without actually reading all 10, the same trust boundary every prose judgement-pass skill in this workspace already has. Named explicitly as a known limit below, not papered over. |
| Capture decisions, not just implementations | Skill-level refinement of the existing New/Resolved/Unchanged mechanism from #635, not a new cross-cutting policy — no ADR (matches the issue's own scope call and the review comment). |
| A change includes its consequences | All three touched skills change together in one PR so `audit-workspace` and `audit-project` don't diverge in coverage-reporting shape, and `janitor-sweep`'s diff and render logic are updated in the same PR as the data they consume. |
| Only what's needed | The gate reaches both scopes because the failure it prevents occurs in both (step 3), but nothing beyond it moves: project scope keeps its inline per-finding tag rendering rather than gaining subsections, and this does not attempt to make the audits themselves deterministic (issue's own "Not in scope"). |
| Improve incrementally | Extends the diff mechanism #635 introduced with one new state, rather than redesigning it. |
| Test what breaks | See Documentation & Instruction Impact / manual verification below — no automated harness exists for these prose skills; this plan proposes a concrete replay-based manual check instead of an untested claim. |
| Workspace vs. project separation | All three files are workspace-repo governance tooling; `audit-project`'s change stays generic (no project-specific content). |
| Workspace improvements cascade to projects | `audit-project` picks up the same coverage-line convention as `audit-workspace`, keeping the two audits' report shapes consistent for anyone reading both. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0013 — progress.md entry-type vocabulary | No | `docs/health.md`'s *content* changes (a new subsection, new coverage text in existing cells), but it is not a `progress.md` entry and this issue doesn't add a new entry type. |
| ADR-0017 — extend AGENTS.md to project repos | No | Not touched — `audit-project`'s changes are report-shape only, not the `AGENTS.md` template contract. |
| ADR-0001 — adopt ADRs | No | Consistent with the issue's own framing and the review comment: this is an in-flux skill-behavior refinement, not a settled cross-cutting decision. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `audit-workspace`'s report format (adds coverage data) | `janitor-sweep` § 3 and § 6, which relay `audit-workspace`'s per-check status/detail into the sweep report | Yes — step 2 |
| `janitor-sweep`'s diff mechanism (adds a 4th state) | The `## Workspace` report-format template block (§ 6) that currently only shows `New`/`Resolved`/`Unchanged` subsections | Yes — step 3 |
| `audit-project`'s report format | `janitor-sweep` § 3's check-2 rollup description and the `## Projects` check-status `Detail` cell, which relay per-repo coverage; § 5's project-scope diff, which now keys on it | Yes — step 4 covers `audit-project` itself; step 3 covers `janitor-sweep`'s project-scope gate, and the `## Projects` row's `Detail` cell carries the per-repo coverage signal so the rollup shows the same grain the Workspace row does |
| `docs/design/planning_document_vocabulary.md` | Does it need to document `New`/`Resolved`/`Unchanged`/`Not re-examined`? | No — confirmed by search (no mention of these states in the vocabulary doc today, before or after this change) and by the review comment's own check: the vocabulary doc describes planning-document *kinds and locations*, not the sweep's diff-state vocabulary. Out of scope; not a gap this PR introduces. |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): the current committed
  `.claude/skills/janitor-sweep/SKILL.md` § 6 report-format template block
  (the `## Workspace` example under "Report format") is stale the moment
  the 4th diff state ships — it must show `Not re-examined` alongside
  `New`/`Resolved`/`Unchanged` for the `## Workspace` example, or a future
  agent copies the old 3-state shape. Same for `audit-workspace`'s
  checklist-section descriptions and `audit-project`'s report-format
  block once the coverage lines are added.
- **Agent-instruction candidates** (proposals only — operator decides):
  None beyond the skill files themselves. This issue's fix *is* the
  instruction-file change; there's no separate `.agent/knowledge/` pattern
  to extract from it.

## Manual Verification

No automated test harness exists for these prose/procedure skill files
(confirmed in review-issue). Proposed verification, per the review comment's
suggestion: **replay the 2026-09-14 vs 2026-09-21 sweep contrast the issue
itself describes.**

1. After the skill edits land, take the 2026-09-21 committed `docs/health.md`
   (current file, the "shallow" run: `principles-enforcement and
   ADR-accuracy sections were spot checks this run (2 of 10 principles, 1 of
   19 ADRs)`) as the simulated "previous committed state."
2. Hand-construct a "next run" using the *same* shallow-run scope (2 of 10
   principles, 1 of 19 ADRs examined, matching or not matching the specific
   items previously found with gaps) and confirm, by walking the new § 5
   diff logic by hand against the two coverage snapshots, that:
   - A finding whose principle/ADR **was not** among the items re-examined
     this simulated run renders `Not re-examined`, not `Resolved` — the
     exact failure mode the issue reports (2026-09-21's "no gaps found"
     silently reading as resolution).
   - A finding whose principle/ADR **was** re-examined and is genuinely
     absent this run renders `Resolved`.
3. Separately, hand-construct a "full run" (10 of 10 principles, 19 of 19
   ADRs examined) and confirm every prior finding either shows up again
   (`Unchanged`) or is now legitimately `Resolved` — no `Not re-examined`
   entries should appear when coverage is complete.

This is a walkthrough against the new rules as written, not a live
sub-agent run (no CI harness triggers `audit-workspace`/`janitor-sweep`
automatically) — record the walkthrough's outcome in the PR description so
a reviewer can check the worked example against the rule text.

## Open Questions

- ~~Whether the Coverage line belongs in `audit-workspace`'s per-section
  report tables (adding a column) or as a separate `### Coverage` summary
  table before `### Findings`.~~ Settled at implementation: a separate
  `### Coverage` table (see Approach step 1).

## Implementation notes

- The publish step's PR body and the operator report both labelled the
  `X of 4 completed` line `Coverage:`. Renamed to `Checks:` in both places
  so the check-count grain and the new per-check coverage grain never share
  a word.
- A prior finding whose originating audit section cannot be inferred from
  its text fails toward `Not re-examined` whenever any section was less than
  fully covered this run — "not known to be fixed" is the safe default.

## Estimated Scope

Single PR — three `.claude/skills/*/SKILL.md` files, all instruction-text
edits with no code, plus the plan's manual-verification walkthrough
recorded in the PR description.
