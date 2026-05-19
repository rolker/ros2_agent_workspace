---
issue: 470
---

# Issue #470 — Workflow skills as a composable timeline: progress.md as universal log, triage-reviews as integrator, handoff via fresh-context sub-agent

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-05-18 EOD
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
- [ ] (must-fix, #1) Add `-c user.name/user.email` overrides to the 3 new commit commands in `review-issue/SKILL.md:211`, `plan-task/SKILL.md:232`, `review-plan/SKILL.md:287` — CI-blocking via `check_pr_authors.py` (mechanism C from #468); 3-source confirmation across all reviews
- [ ] (must-fix, #2) Remove "ADR-0012-style cross-reference" option from ADR-0013 Consequences (lines 116-119); require supersession only. Update `principles_review_guide.md:49` to match — ADR-0012 §52-58 forbids substantive addendums; 2-source confirmation
- [ ] (must-fix, #3) Resolve owning repo before `$WORKTREE_ISSUE` check in `review-issue/SKILL.md:153-157`; fix worktree-list regex to handle layer worktrees (`issue-<repo>-<N>`) + numeric boundary
- [ ] (should-fix, #4) Rename `### Action items` → `### Actions` in `review-issue/SKILL.md:199` to match ADR-0013 §70-71 canonical vocab; 2-source confirmation
- [ ] (should-fix, #5) Fix `progress.md:9` timestamp `2026-05-18 EOD` → `2026-05-19 01:00` per ADR-0013 schema
- [ ] (should-fix, #6) Add locate-or-create flow to `review-plan/SKILL.md` step 6 for PR-number / file-path modes; 2-source confirmation
- [ ] (should-fix, #7) Fix invalid HTML comment marker on `review-plan/SKILL.md:269` (`<--` → `<!--`)
- [ ] (should-fix, #8) Scope ADR-0013 "head SHA filter" rule to PR-correlated entry types (issue/plan entries use issue number + plan-file SHA)
- [ ] (nice-to-fix, #9) Fix `review-issue` "no action items" condition to not drop Recommendations when Action-needed findings absent
- [ ] (nice-to-fix, #10) Update plan.md Estimated Scope (line 134-139) — actual landed diff is +505/-1 across 7 files
- [ ] (nice-to-fix, #11) Add SHA-citation bullet to `plan-task` step 9 (report should cite progress.md commit SHA per step 8)
- [ ] (nice-to-fix, #12) Add `## External Review` transitional row to `principles_review_guide.md:36`

### False positives (addressed in `44343da`)
- ADR-0013 relative-link depth in 3 SKILL.md files — fixed
- plan-task step 8 missing `git push` — fixed
- review-plan `--in-context` flag undefined — replaced with behavioral detection
- review-issue fallback making persistence optional — addressed by worktree-on-demand pivot in `c89cdfe`
- Plan-only commit findings on `a1d67cd` — resolved by subsequent implementation
