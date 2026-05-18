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
