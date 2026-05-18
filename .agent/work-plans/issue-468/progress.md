---
issue: 468
---

# Issue #468 — Sub-agent commits drop agent git identity; commits get authored under human user

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-05-18 22:15
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))
**Verdict**: approved

**Branch**: feature/issue-468 at `04589d6`
**Mode**: pre-push
**Depth**: Standard (reason: 12 files, governance + hook + CI surface, identity-enforcement security-relevance)
**Must-fix**: 0 | **Suggestions**: 3

### Findings
- [ ] (suggestion) Coauthor semantics stricter than #468's primary-author failure mode — inspect `authors[0]` only or document intent — `.agent/hooks/check_pr_authors.py:97-105`
- [ ] (suggestion) Test gap: canonical `git -c user.email=…` + `GIT_CONFIG_PARAMETERS` propagation flow not pinned by regression test — `.agent/scripts/test_check_commit_identity.sh`
- [ ] (suggestion) `pr_number` arg unvalidated; add `.isdigit()` check + clearer usage error — `.agent/hooks/check_pr_authors.py:64`

### Dogfooding verification
All 5 commits on this branch authored as `Claude Code Agent <roland+claude-code@ccom.unh.edu>` (the agent pattern). The CI check this PR introduces accepts its own branch; if a commit were accidentally human-authored, this PR would self-reject — the exact dogfooding outcome the plan promised.

## Integrated Review
**Status**: complete
**When**: 2026-05-18 23:10
**By**: Claude Code Agent (Claude Opus 4.7 (1M context)) — emulating the future triage-reviews-as-integrator from [#470](https://github.com/rolker/ros2_agent_workspace/issues/470)

**PR**: #471 at `f5817e3` (before fix-pass)
**Sources**: 4 (Copilot R1 on `4e54c01`, Copilot R2 on `04589d6`, Local Review on `04589d6`, CI rollup)
**Cross-source confirmations**: 1
**CI**: all-pass (10 checks, including Mechanism C dogfooded against this PR)

### Findings (sources column)
- [x] (cross-confirmed) Coauthor handling — reject only on `authors[0]` primary author; resolves both **Copilot R2** + **Local Review** independently catching the same future-state bug + Copilot R2's offending-counter over-count complaint — `.agent/hooks/check_pr_authors.py`
- [x] (suggestion, Copilot R2) Docstring claimed bot allowlist that didn't exist — rewritten with primary-author semantics — `.agent/hooks/check_pr_authors.py:1-31`
- [x] (suggestion, Local Review) Test gap for `git -c user.email=…` + `GIT_CONFIG_PARAMETERS` propagation — added 2 new cases; regression test now 14-case — `.agent/scripts/test_check_commit_identity.sh`
- [x] (suggestion, Local Review) `pr_number` input validation — `.isdigit()` check with actionable error message — `.agent/hooks/check_pr_authors.py`
- [x] (suggestion, Copilot R1) "revertable" → "revertible" spelling — `.agent/work-plans/issue-468/plan.md`

### False positives (sources column, dismissed in this report)
- (Copilot R1 on stale plan-only commit) `feature/ISSUE-<N>-<description>` form not handled. **Stale** — current plan and implementation use case-insensitive regex `[iI][sS][sS][uU][eE]` from `identity_patterns.py`; regression test covers `feature/ISSUE-100-some-description`. Comment was on pre-revision plan wording.
- (Copilot R2) Drop `[PLAN]` prefix from PR title. **False positive by convention** — `plan-task` SKILL.md establishes the prefix as a stable marker through the PR lifecycle (PR #464 kept it through merge). Established workspace convention.

### Worked-example notes for #470
- triage-reviews-as-integrator emulated manually for this PR: read Copilot reviews from GitHub + Local Review from progress.md, surfaced 1 cross-source confirmation (coauthor handling) as the strongest signal. Two false-positive bot findings correctly dismissed with justifications. The shared `.agent/scripts/progress_read.sh` helper from #470 would have made the integration scripted instead of manual.
