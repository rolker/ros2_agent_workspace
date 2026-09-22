# Manual verification — issue #651

**When**: 2026-09-22 08:54 -04:00
**Against**: `.claude/skills/janitor-sweep/SKILL.md` § 5 (Run-over-run diff)
and § 6 (Render the report), `.claude/skills/audit-workspace/SKILL.md`
§ Coverage, `.claude/skills/audit-project/SKILL.md` § 2 / § Report Format,
as written on branch `feature/issue-651` through `9d503b9` (the last skill
and plan commit before this revision of the file — re-walked after the
round-2 review fixes).

These three skills are prose/procedure files with no automated test
harness (confirmed in review-issue), so the verification the plan proposes
is a **hand walkthrough of the rules as written** against a real pair of
runs — not a live sub-agent run. This file records that walkthrough so a
reviewer can check the worked examples against the rule text rather than
take "walked it through" on trust.

## The real pair of runs

The contrast the issue itself describes:

- **2026-09-14** — a local, uncommitted sweep report on the host that ran
  it. `audit-workspace` made a **full principles pass** and found five
  enforcement gaps (tier 3). They are quoted verbatim in the committed
  `docs/health.md`'s hand-written "Not re-examined this run" paragraph:
  issue-closing keywords have no hook or CI check; the AI-signature
  requirement has no check; `validate.yml`'s required-documentation step
  checks only README.md and ARCHITECTURE.md; ADR-0005/0008/0009 are still
  `Proposed` while AGENTS.md cites them as binding; the Consequences Map's
  ADR table omits 8 of 19 ADRs.
- **2026-09-21** — the first *committed* `docs/health.md`. Its
  `audit-workspace` row reads `7 of 7 sections completed; ... Principles-
  enforcement and ADR-accuracy sections were spot checks this run (2 of 10
  principles, 1 of 19 ADRs)`, and its tier 3 `New` subsection reads "(none
  found in this run's sample)".

That is the failure the issue reports, in the record: under the old
three-state diff, five gaps that were simply **not looked at** would have
rendered `Resolved`. On 2026-09-21 they did not, only because the operator
hand-wrote a paragraph after the fact.

Note what the 2026-09-21 run did **not** record: *which* 2 principles and
*which* ADR it examined. Nothing in this walkthrough invents those names —
where the rule turns on them, both branches are walked.

## Case A — workspace scope, sampled section (the issue's own failure)

Treat 2026-09-14's report as the previous state and 2026-09-21's as this
run. Take one prior finding: *"the AI-signature requirement has no check"*
(tier 3, from `audit-workspace` section 1, principles).

Walking § 5's workspace-scope procedure:

1. The finding is **absent** from this run's findings — tier 3 `New` is
   empty. Under the pre-#651 rule this is where it silently became
   `Resolved`.
2. Determine the section: tier 3 findings come from section 1 (principles)
   — named explicitly in § 5's section-to-tier mapping, which after round 1
   names all seven sections, so nothing falls through to the
   can't-determine bullet by omission.
3. Read section 1's coverage row: `principles 2 of 10`. `X < Y`, so the
   section is **partially covered**.
4. Bullet 2 vs bullet 3 turns on whether the item is among the 2 named:
   - If the coverage row's item list **does** name the AI-signature
     principle and the finding is still absent → **`Resolved`**. Partial
     coverage does not block resolution of the thing actually re-examined.
   - If it does **not** → **`Not re-examined`**, carried forward verbatim
     under tier 3's `Not re-examined` subsection.
5. The carried-forward entry is stamped with the date of the health document
   that first parked it — here `(since 2026-09-21)`, this being the first
   run to carry it. A later run that still cannot cover section 1 copies
   that same date rather than restamping, so an entry missed five times is
   visibly five runs old.
6. Tier 3 has **no** `New` findings in this run ("none found in this run's
   sample"). Its only content is the carried-forward entry — so § 6's
   omission rule matters here: a tier is omitted only when *every* one of
   its subsections is empty, `Not re-examined` included, so tier 3 renders.
   Under the "no findings this run" wording this tier would have vanished
   and taken the carried-forward gaps with it, reinstating the failure one
   level up.
7. With all five 2026-09-14 gaps and only 2 principles + 1 ADR examined, at
   most three could resolve and at least two must carry forward. The
   hand-written paragraph in the committed file becomes rendered output.

**Outcome**: the rules reproduce, mechanically, what the operator did by
hand — which is the whole point of the issue. ✅

Two details this case also exercises:

- The 2026-09-21 run **did not name** its 2 principles / 1 ADR, so under
  the new `audit-workspace` rule ("a nameless sample is not coverage data")
  that run would now be non-conforming, and its rows unusable for step 4
  above. The gate's own failure mode is safe: a row that names nothing
  matches no item, so every prior finding from it falls to
  `Not re-examined` — "not known to be fixed", never silently resolved.
- The prior findings carried forward are drawn from the previous file's
  `New` + `Unchanged` + `Not re-examined` entries, minus `Resolved` (§ 5).
  Had 2026-09-21 rendered these five under `Not re-examined`, the next run
  would still see them as prior findings — they would not reset to `New`
  when the section is finally examined in full.

## Case B — workspace scope, full coverage

Hand-construct a next run over the 2026-09-21 committed state, this time a
full pass: coverage rows `principles all 10` / `10 of 10`, `ADRs 19 of 19`,
`scripts all 58`, `templates all 12`, `consequences-map items all N`,
`adapters all 3`, `worktrees all N`.

- The two stale-worktree findings (tier 1, section 7): section 7 is
  `all N` — fully covered. Present again → `Unchanged`; genuinely gone (the
  worktree removed) → `Resolved`. Both legitimate.
- The script-table finding (tier 5, section 3, `all 58`): fully covered, so
  a miss is a real `Resolved`.
- A prior finding from section 6 — *"the Gemini adapter is missing"* — is
  gated on `adapters: all 3`, which is what that section reports **even when
  an adapter is missing**: the absence is the finding, not a coverage
  shortfall (the same rule `audit-project` § 3 applies to a missing agent
  guide). So the section is fully covered, and the finding resolves exactly
  when the adapter was actually added. Had a missing adapter been written as
  `2 of 3`, its own finding could never have resolved.
- **No tier renders a `Not re-examined` subsection**, because every section
  was fully covered and the subsection is rendered only when non-empty
  (§ 6). That is the deliberate asymmetry with `New`/`Resolved`/`Unchanged`,
  which are always present inside a rendered tier: an always-present empty
  `Not re-examined` would suggest partial coverage where there was none.
  A tier that now holds nothing at all — no `New`, no `Resolved`, no
  `Unchanged`, no carried-forward entry — is omitted outright, which is the
  only case § 6's omission rule still covers.

**Outcome**: full coverage produces the pre-#651 three-state report
unchanged. The new state costs nothing on a complete run. ✅

One variant worth walking, added in round 1: suppose the same run reads
`scripts: 57 of 58 — build_report_generator.py: unreadable`. Section 3 is
now *partially* covered even though it never samples. A prior script-table
finding about that one script → `Not re-examined`; a prior finding about
any other script → `Resolved`. Before round 1 this section could only say
`all 57`, which is indistinguishable from a workspace holding 57 scripts,
and the finding would have resolved. ✅

## Case C — check 4 (research-digest freshness)

Check 4's findings are workspace scope and tier 5, but they come from a
different check: there is no `audit-workspace` section for them and no row
in check 1's Coverage table. Walking a prior finding *"research digest 47
days stale (threshold 30)"*:

- The section bullets do not apply — there is no section. The
  can't-determine bullet does not apply either; § 5 says explicitly not to
  route check-4 findings through it. (Before round 1, neither did apply and
  the procedure simply stopped here.)
- Check 4 reads the one digest file, whole, every run — no sampling — so
  its coverage is full whenever it ran. Its gate is therefore the **check's
  own status**:
  - 2026-09-21's row is `Research-digest freshness | OK`. The check
    completed, the file was read, the finding is absent → **`Resolved`**.
    Correct: the digest was updated 2026-09-17, 4 days before the run.
  - Had the row been `SKIPPED(...)` or `FAILED(...)` — the digest file
    missing or unreadable — the same absence is → **`Not re-examined`**.
    Nothing was read, so nothing is known.

**Outcome**: the rule is decidable for every check-4 finding in both
directions. ✅

## Case D — project scope, clone mode (the round-1 must-fix)

Project scope has no committed history, so this walks a repo's prior
**local** report under `.agent/scratchpad/janitor/` against a later run.

Take a repo audited in **`layer`** mode in run 1. Its § 8 coverage row
reads `workspace integration: all 3`, and the audit found: *"repo is not in
its expected layer"* (tier 5).

Run 2 audits the same repo in **`clone`** mode — the host has no layer
checkout, so `resolve_repo_checkout.sh` returns a clone. The layer check
cannot run and is reported `SKIPPED (no layer checkout)`, so § 8's coverage
row reads `2 of 3 — layer: SKIPPED (no layer checkout)`.

- The prior finding is **absent** from run 2 — the check that produced it
  did not run.
- Under the shipped-before-round-1 text, project scope was three-state and
  the paragraph asserted this could not happen, so the finding rendered
  `[Resolved]` — **the exact failure #651 closes**, in the scope where it
  is likeliest, because checkout mode changes with the host rather than
  with the code.
- Under the current text, § 5's project-scope bullet reads § 8's coverage
  row, sees a section short of full, and renders
  **`- **<repo>** [Not re-examined]: repo is not in its expected layer`** —
  carried forward verbatim, in project scope's own inline-tag shape rather
  than as a subsection.

**Outcome**: the gate now holds in project scope, and the rendering shape
difference between the scopes is preserved. ✅

Step 2 of that walk — "which section did this finding come from?" — is
answerable because § 5's project-scope block now maps every tier to its
`audit-project` section: the finding above is tier 5, and tier 5 is § 2 /
§ 6 / § 8. Tiers 1 and 3 have no `audit-project` section at all, and a prior
finding sitting in one of them takes the conservative fallback —
`[Not re-examined]` unless every section of that repo was fully covered. § 7
(planning documents) never contributes a finding, so it never appears as a
tier's source.

The same walk succeeds on the other partial forms `audit-project` emits —
`0 of 1 — <reason>` (an agent guide that exists but could not be read),
`N-1 of N — <item>: <reason>`, `0 of 4 — SKIPPED(<reason>)` (the
planning-document probe), `N of N packages — run: M of N` (a partial
`colcon test` pass, for a finding that came from the test run). Each is a
section that did not fully re-examine what a prior finding may have come
from.

§ 3's two word-forms are walked the same way, and are the case the round-2
review caught. A prior finding *"the agent guide's package inventory omits
`foo_msgs`"* (tier 4, § 3):

- Run 2 reports `agent guide: checked` — § 3's **full-coverage** form. The
  finding is absent → **`[Resolved]`**. Read only against the numeric forms,
  `checked` would have matched neither the full nor the partial bullet and
  this finding could never have resolved at all.
- Run 2 reports `agent guide: 0 of 1 — unreadable` — the **partial** form
  → **`[Not re-examined since <date>]`**.
- Run 2 reports `agent guide: absent`. Nothing exists to carry the finding
  about. If run 2 also reports § 2's "`.agents/README.md` missing" finding,
  the audit looked and the guide's absence is what it found →
  **`[Resolved]`** (the content finding is moot). Without that § 2 finding,
  nothing was established → **`[Not re-examined since <date>]`**.

## What this walkthrough does not establish

- **Coverage remains reportable, not self-verifying.** Every case above
  trusts the coverage row the audit wrote. An audit that writes
  `principles: 10 of 10` without reading all ten converts a real gap to
  `Resolved` — the same failure by a different door. § 5 now says so at the
  point where the number becomes a classification, and makes an
  implausible row a `FINDINGS` condition on the check, but this is a
  documented trust boundary, not a closed one.
- **No live run.** Nothing here executes `janitor-sweep`; the first live
  sweep after this lands is the real test, and its `docs/health.md` is
  where a reviewer should check that the rendered shape matches § 6.
