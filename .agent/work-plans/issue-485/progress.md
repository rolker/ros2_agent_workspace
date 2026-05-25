---
issue: 485
---

# Issue #485 — #470 phase B: triage-reviews as progress.md integrator (## Integrated Review)

## Issue Review
**Status**: complete
**When**: 2026-05-25 14:19 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**Issue**: #485
**Comment**: https://github.com/rolker/ros2_agent_workspace/issues/485#issuecomment-4536264990
**Scope verdict**: well-scoped

### Actions
- [ ] (ADR-0013 gap) Write step 3 as "filter by entry type **+ correlation key**" (issue # / plan-commit SHA / PR-or-branch head SHA per ADR-0013's correlation-key table), not just "by entry type" — cross-source confirmation is keyed by both.
- [ ] (ADR-0013 gap) Make `## External Review` predecessor-integration explicit in the reader (integrate historical entries even as new writes switch names); cover with a test.
- [ ] (consequence) If `progress_read.sh` is added, update the Script Reference table in `AGENTS.md` (Makefile target only if warranted).
- [ ] (consequence) Verify no framework adapter (`copilot-instructions.md`, `gemini-cli.instructions.md`, `AGENT_ONBOARDING.md`) describes triage-reviews' now-stale `## External Review` output name.
- [ ] (test) Add parser/helper tests up front: malformed entries, missing correlation key, `Z` vs `+00:00` offset forms, predecessor recognition.
- [ ] (open question) Decide whether ADR-0013 needs a status note once new writes are `## Integrated Review`; if touched, stay within ADR-0012's addendum carve-out — do not alter the Decision table.
- [ ] (recommendation) Use `.agent/work-plans/issue-468/progress.md`'s `## Integrated Review` as the golden-output fixture; assert the redesigned skill reproduces its shape (sources column, cross-source flagging, separate false-positive justifications).

## Plan Authored
**Status**: complete
**When**: 2026-05-25 15:02 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**Plan**: `.agent/work-plans/issue-485/plan.md` at `239e9e2`
**PR**: https://github.com/rolker/ros2_agent_workspace/pull/486 (`[PLAN]` prefix)
**Phases**: single

### Open questions
- [x] ADR-0013 status note → **resolved (2026-05-25): add an ADR-0012 cross-reference addendum** noting the rename is live (links #485); Decision table untouched.
- [x] `progress_read` output format → **resolved (2026-05-25): JSON, implemented in Python** (`progress_read.py`).

## Plan Review
**Status**: complete
**When**: 2026-05-25 15:27 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context)) — fresh-context sub-agent (dispatched by plan author; independent context)

**Plan**: `.agent/work-plans/issue-485/plan.md` at `cbca052`
**PR**: https://github.com/rolker/ros2_agent_workspace/pull/486
**Verdict**: approve-with-suggestions

### Findings
- [ ] (must-fix) Half-rename: plan steps 3/5/6 still say `progress_read.sh` while step 1, the files table, and Resolved Decisions say `.py` — internal contradiction. Rename all to `progress_read.py`. — `plan.md`
- [ ] (must-fix) File targeting misses two in-body spots in `triage-reviews/SKILL.md` the rename leaves stale: the forward-looking self-reference note ("ADR-0013 will rename this … in phase B") and the commit-message string `"progress: external review for #<N>"` → `"progress: integrated review for #<N>"`. Add explicit line-level callouts. — `plan.md` step 5/7
- [ ] (suggestion) Tighten sources semantics: ADR-0013 keys cross-source confirmation by entry-type + correlation key for ANY two sources (not just local-entry vs GitHub-review); cite `0013:120-128`. — `plan.md:38-40`
- [ ] (suggestion) Parser tests must cover all three correlation-key forms incl. the `**Branch**: <name> at <sha>` alternative, and skill-specific fields appearing BEFORE the canonical field (golden fixture exercises both). — `plan.md:31-33`
- [ ] (suggestion) Predecessor-recognition test needs a fixture with an actual `## External Review` heading (issue-452/460/461) — issue-468 has none, so one fixture can't cover both golden-output and predecessor. — `plan.md:32-33`
- [ ] (suggestion) Make the review-guide ADR-0013 row edit definite, not conditional: post-merge the "transitional, until phase B retires it" wording is factually wrong. — `plan.md:52`
- [ ] (suggestion) State the Makefile-half of the script consequence explicitly ("no make target — library helper"), per the consequences map. — `plan.md:85`
- [ ] (suggestion) Phrase the adapter-verify step as expected no-op (grep confirmed adapters don't name skill outputs). — `plan.md:46-47`
- [ ] (suggestion) ADR-0012 addendum must land in References/Status, NOT edit the `## External Review` Decision-table parenthetical (that'd need supersession). — `plan.md:48-49`

## Implementation
**Status**: complete
**When**: 2026-05-25 16:53 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**Branch**: feature/issue-485 at `f468ac5` (local, pre-push; PR #486)
**Commits**: `ab5ab44` (progress_read.py + 19 unittest cases), `f468ac5` (triage-reviews integrator + docs)

### Findings
- [x] Plan steps 1–8 implemented: `progress_read.py` helper + tests; `triage-reviews` steps 3/5/6/7 (read prior entries, sources column, cross-source confirmations, `## Integrated Review` rename + in-body fixes); `AGENTS.md` script table; ADR-0012 addendum to ADR-0013; review-guide ADR-0013 row refresh.
- [x] Adapter check (step 7): no-op — no framework adapter names skill output entries.
- [ ] Next: review-code (pre-push) before pushing, per local-first.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-05-25 17:29 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context)) — fresh-context sub-agent (dispatched by author; independent context)
**Verdict**: approve-with-suggestions

**Branch**: feature/issue-485 at `25869e7` (local, pre-push)
**Sources**: 1 (fresh-context review-code sub-agent)
**Cross-source confirmations**: 0

### Findings
- [x] (fixed `eaa2c0d`) Fenced code blocks parse as real entries: `_split_entries` now tracks fence state and skips fenced lines, so a heading/checkbox quoted inside a ` ```markdown ` block can't become a phantom entry with a real-looking correlation SHA. — `progress_read.py`
- [x] (fixed `eaa2c0d`) Leading `---` horizontal rule mis-detected as frontmatter: `_split_frontmatter` now requires a `key:` line before treating the block as YAML, so a horizontal rule no longer swallows the body. — `progress_read.py`
- [x] (fixed `eaa2c0d`) Test gaps closed: added fenced-block, CRLF, no-frontmatter, leading-HR, and no-leading-paren cases (24 tests pass). — `test_progress_read.py`
- [x] (observation, not a defect) `_corr_plan` greedy-split handles paths containing " at " correctly (`$`-anchored SHA + non-greedy); no action.

### Notes
- Lint/tests green under the real hook config (flake8 plugin-less max-line 100, black 100). 19 tests pass. Plan drift: none (steps 1–8 as planned). Rename complete write-side; read-side predecessor retained. Verdict approve-with-suggestions; fixing #1–#3 before push per the Quality Standard (stale data is not a nit) since phase C (#481) will rely on this helper.

## Integrated Review
**Status**: complete
**When**: 2026-05-25 18:35 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #486 at `01700f5`
**Sources**: 2 (Copilot @ `01700f5`; prior `## Local Review (Pre-Push)` @ `25869e7`, findings already resolved in `eaa2c0d`)
**Cross-source confirmations**: 0
**CI**: all-pass (Lint, Validate Documentation, commit-identity Mechanism C)

<!-- Dogfooding the phase-B rename: this PR introduces ## Integrated Review, so
     the triage entry for the PR itself uses the new type. progress_read.py
     recognizes the predecessor ## External Review on read either way. -->

### Findings
- [ ] (valid, Copilot @ `01700f5`) `## Implementation` entry says "19 unittest cases" but the suite is 24 after the fence fix (`eaa2c0d`); the same entry's findings say 24 — stale count. Update to 24. — `progress.md`
- [ ] (valid, Copilot @ `01700f5`) `## Local Review (Pre-Push)` Notes say "19 tests pass" contradicting "24 tests pass" in its resolved finding. Update to 24. — `progress.md`
- [ ] (valid, Copilot @ `01700f5`) review-guide ADR-0013 row drops `## External Review` from the canonical-types list, but ADR-0013's Decision table still lists it (read-only predecessor) — reads as contradicting the ADR. Reword to distinguish "types to write" vs "recognized on read". — `principles_review_guide.md:36`
- [ ] (valid, substantive, Copilot @ `01700f5`) `--type "Integrated Review"` silently drops legacy suffixed headings like `## External Review (Round 5–6)` (`issue-452/progress.md:23`): `entry["type"]` is the full heading, so `predecessor_of` is null and `_matches_type` excludes it. Fix: normalize to a canonical base (strip a trailing ` (...)` when the base is canonical, but keep `Local Review (Pre-Push)` intact) for recognized/predecessor/correlation/filter; add a regression test. — `progress_read.py`

### False positives
- (none)

### Notes
- 5 older Copilot comments are against the plan-only commits (`3110a14`/`b3faa44`) and target `plan.md`/early `progress.md` — low priority per the plan-first guidance (plan is a pre-implementation artifact; reviews on plan-only commits go stale once implementation lands). Two are historical-record minutiae in `progress.md` (a `progress_read.sh` reference at line 19, adapter paths at line 20) — optional tidy, not blocking.
