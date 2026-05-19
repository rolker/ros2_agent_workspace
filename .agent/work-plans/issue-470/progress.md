---
issue: 470
---

# Issue #470 — Workflow skills as a composable timeline: progress.md as universal log, triage-reviews as integrator, handoff via fresh-context sub-agent

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-05-19 01:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))
**Verdict**: approved

**Branch**: feature/issue-470 at `c89cdfe` (fix commit incoming)
**Mode**: pre-push
**Depth**: Standard (reason: 6 files, governance-touching — new ADR + principles guide + 3 governance skills, markdown-only)
**Must-fix**: 1 (resolved in fix commit) | **Suggestions**: 2 (resolved in fix commit)

### Findings
- [x] (must-fix) Broken relative link to ADR-0013 — 3 SKILL.md files use `../../docs/...` but are 3 levels deep, need `../../../docs/...` — `review-issue/SKILL.md:147`, `plan-task/SKILL.md:191`, `review-plan/SKILL.md:249`
- [x] (suggestion) plan-task step 8 commits but doesn't push; "shows up on the draft PR" claim was wrong — added explicit `git push` after the commit — `plan-task/SKILL.md` step 8
- [x] (suggestion) review-plan `--in-context` flag referenced but undefined; replaced with behavioral detection (compare `$AGENT_NAME` to existing `## Plan Authored` entry's `**By**` field) — no new flag surface — `review-plan/SKILL.md` step 6

### Notes
- This is the first `## Local Review (Pre-Push)` entry written by review-code on a PR whose implementation directly motivated ADR-0013. The entry follows the schema the ADR canonicalizes — modest dogfooding.
- All findings came from the fresh-context Adversarial Specialist sub-agent. Governance + plan-drift specialists (run in-context by the lead reviewer) surfaced nothing additional — markdown-only governance change with no consequence gaps.
- Triage with Copilot's review left for a follow-up session per user pause request.

## External Review
**Status**: complete
**When**: 2026-05-19 01:30
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #473 — 3 review(s) from `copilot-pull-request-reviewer[bot]`, 12 valid (3 must-fix, 5 should-fix, 4 nice-to-fix), 5 false positives (addressed by pre-push fix commit `44343da`)
**CI**: all-pass

### Actions
- [x] (must-fix, #1) Add `-c user.name/user.email` overrides to the 3 new commit commands in `review-issue/SKILL.md:211`, `plan-task/SKILL.md:232`, `review-plan/SKILL.md:287` — CI-blocking via `check_pr_authors.py` (mechanism C from #468); 3-source confirmation across all reviews → landed in `2787d23` (also extended to `review-code` for consistency)
- [x] (must-fix, #2) Remove "ADR-0012-style cross-reference" option from ADR-0013 Consequences (lines 116-119); require supersession only. Update `principles_review_guide.md:49` to match — ADR-0012 §52-58 forbids substantive addendums; 2-source confirmation → landed in `4effba5`
- [x] (must-fix, #3) Resolve owning repo before `$WORKTREE_ISSUE` check in `review-issue/SKILL.md:153-157`; fix worktree-list regex to handle layer worktrees (`issue-<repo>-<N>`) + numeric boundary → landed in `8fff314`
- [x] (should-fix, #4) Rename `### Action items` → `### Actions` in `review-issue/SKILL.md:199` to match ADR-0013 §70-71 canonical vocab; 2-source confirmation → landed in `5525c42`
- [x] (should-fix, #5) Fix `progress.md:9` timestamp `2026-05-18 EOD` → `2026-05-19 01:00` per ADR-0013 schema → landed in `5525c42`
- [x] (should-fix, #6) Add locate-or-create flow to `review-plan/SKILL.md` step 6 for PR-number / file-path modes; 2-source confirmation → landed in `3833d6a`
- [x] (should-fix, #7) Fix invalid HTML comment marker on `review-plan/SKILL.md:269` (`<--` → `<!--`) → landed in `5525c42`
- [x] (should-fix, #8) Scope ADR-0013 "head SHA filter" rule to PR-correlated entry types (issue/plan entries use issue number + plan-file SHA) → landed in `5525c42`
- [x] (nice-to-fix, #9) Fix `review-issue` "no action items" condition to not drop Recommendations when Action-needed findings absent → landed in `5525c42` (folded into the `### Actions` rename — Recommendations now preserved under the same heading)
- [x] (nice-to-fix, #10) Update plan.md Estimated Scope (line 134-139) — actual landed diff is +505/-1 across 7 files → landed in `4126cb1`
- [x] (nice-to-fix, #11) Add SHA-citation bullet to `plan-task` step 9 (report should cite progress.md commit SHA per step 8) → landed in `4126cb1`
- [x] (nice-to-fix, #12) Add `## External Review` transitional row to `principles_review_guide.md:36` → landed in `4126cb1`

### False positives (addressed in `44343da`)
- ADR-0013 relative-link depth in 3 SKILL.md files — fixed
- plan-task step 8 missing `git push` — fixed
- review-plan `--in-context` flag undefined — replaced with behavioral detection
- review-issue fallback making persistence optional — addressed by worktree-on-demand pivot in `c89cdfe`
- Plan-only commit findings on `a1d67cd` — resolved by subsequent implementation

## External Review
**Status**: complete
**When**: 2026-05-19 02:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #473 — 4 review(s) (2 fresh comments on HEAD `8a4687f`), 2 valid, 0 false positives
**CI**: all-pass (incl. Mechanism C — field-validated the must-fix #1 fix from commit `2787d23`)

### Actions
- [x] (small, #1) Add `.agent/scripts/` path prefix to `worktree_enter.sh` invocation in `review-issue/SKILL.md:180` — as written it requires the script on PATH → landed in `ce1e85a`
- [x] (small, #2) Label review-issue's step 8a bullets as `**8a.1.**` through `**8a.5.**` so the cross-references from `review-plan/SKILL.md:269, 278` (introduced in commit `3833d6a`) resolve for readers → landed in `ce1e85a`

## External Review
**Status**: complete
**When**: 2026-05-19 17:30
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #473 — 5 review(s) from `copilot-pull-request-reviewer[bot]` (4 fresh comments on HEAD `5d57910`), 4 valid (2 must-fix, 2 should-fix), 0 false positives. Earlier-round comments addressed in the prior two `## External Review` entries.
**CI**: all-pass on Workspace Validation (Lint, Validate Documentation, Mechanism C). `Copilot code review / Cleanup artifacts` step failure is in Copilot's own artifact-deletion job — unrelated to PR code, not blocking.

### Actions
- [x] (must-fix, #1) Resolve ADR-0013 normative contradiction at `docs/decisions/0013-progress-md-entry-type-vocabulary.md:44-46` — added `## External Review` as a transitional row in the Decision table marked as predecessor of `## Integrated Review`, retired by phase B; Predecessor recognition section now cross-referenced from the table itself.
- [x] (must-fix, #2) Added `-c user.name="$AGENT_NAME" -c user.email="$AGENT_EMAIL"` overrides to `plan-task/SKILL.md` step 6 plan-commit example + a paragraph linking to AGENTS.md § Agent Commit Identity and noting the `check_pr_authors.py` (Mechanism C) failure mode.
- [x] (should-fix, #3) `review-issue/SKILL.md` step 8a.1 rewritten — dropped circular `--repo <owner/repo>`, now describes a 3-step probe (workspace first via cwd-based `gh issue view <N>`, then project repos under `layers/main/*/src/*`, then error). Corrected the "mirrors plan-task step 4" claim.
- [x] (should-fix, #4) `review-plan/SKILL.md` step 6 sub-step 1 rewritten with mode-split — PR-number mode reuses step-1 metadata, `--issue`/file-path mode uses the same cwd-based probe as #3. Mirror relationship now actually holds.

### Notes
- Findings #3 and #4 are the same underlying defect across two skill files — fix together to maintain the "mirror" relationship the docs claim.
- All 40 inline comments on earlier commits (a1d67cd / c89cdfe / 44343da / 8a4687f) were classified in the prior two `## External Review` rounds. Cross-checked: none reopened by HEAD changes.
