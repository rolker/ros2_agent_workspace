---
issue: 484
---

# Issue #484 — fix: layer worktree LD_LIBRARY_PATH shadowed by main tree install (C++ analog of #427)

## Plan Authored
**Status**: complete
**When**: 2026-05-25 14:50 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**Plan**: `.agent/work-plans/issue-484/plan.md` at `ddbef78`
**PR**: https://github.com/rolker/ros2_agent_workspace/pull/487 (`[PLAN]` prefix)
**Phases**: single

### Open questions
- [ ] No open questions — plan is review-plan-ready.

## Plan Review
**Status**: complete
**When**: 2026-05-25 15:20 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context)) (in-context — author self-review)

**Plan**: `.agent/work-plans/issue-484/plan.md` at `ddbef78`
**PR**: https://github.com/rolker/ros2_agent_workspace/pull/487
**Verdict**: approve-with-suggestions

### Findings
- [ ] (must-fix) Acceptance-gate timing collides with active unh_marine_navigation#26 worktree — defer validation to post-PR-#27 merge, or rephrase gate to "any C++ worktree with new symbols" — `plan.md:26`
- [ ] (suggestion) Makefile sanity-check row missing from consequences table — confirm `grep -F worktree_create Makefile` shows no target — `plan.md:57-64`
- [ ] (suggestion) Prepend ordering rationale (`<build>/<pkg>` first vs `<install>/<pkg>/lib`) should be noted in plan step 1 so the explanatory comment lands at implementation time — `plan.md:17-20`
- [ ] (suggestion) "Open Questions: None" contradicts must-fix finding above — update to capture the acceptance-gate timing question — `plan.md:66-68`
- [ ] (suggestion) Test 23 extension underspecified — note that python3.99/site-packages fabrication already exists; new step is just counting `lib` and `build/<pkg>` occurrences in `$LD_LIBRARY_PATH` — `plan.md:24, 35`

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-05-25 16:30 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))
**Verdict**: approved

**Branch**: `feature/issue-484` at `10f7be2`
**Mode**: pre-push
**Depth**: Light (reason: <50 code lines, ≤3 code files, no override-trigger files)
**Must-fix**: 0 | **Suggestions**: 0

### Findings
- [ ] No issues found. LGTM. (Copilot Adversarial surfaced 2 candidate findings — both verified as false positives: asymmetric guarding matches the sibling AMENT/PYTHONPATH pattern; `$pkg` is defined at test_worktree_create.sh:924 within the same function scope.)
