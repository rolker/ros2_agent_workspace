---
issue: 452
---

# Issue #452 — Port review-skill improvements from agent_workspace + encourage use

## External Review
**Status**: complete
**When**: 2026-05-15 17:30
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #453 — 4 review round(s), 5 valid, 1 needs discussion, 14 false positives / already addressed
**CI**: all-pass (Lint × 2, Validate Documentation × 2)

### Actions
- [x] Dropped ambiguous `<branch-name>` Usage form from `.claude/skills/review-code/SKILL.md` (R4-1, commit 33c3be0)
- [x] Default-branch resolution prefers local `git symbolic-ref refs/remotes/origin/HEAD` (R4-2, commit 3bc9d63)
- [x] Changed `mkdir` → `mkdir -p` in `.agent/work-plans/README.md` workflow snippet (R4-3, commit 8c5433d)
- [x] `review-plan` PR-number path uses `closingIssuesReferences` (R4-4, commit 3623c41)
- [x] Stripped stale `--depth=` from PLAN Open Questions (R4-5, commit a02b180)
- [x] R4-6 resolved Option A + fail-loud: documented branch-name extraction in `triage-reviews` as deliberate (worktree-aligned, local-first); added explicit error when branch name doesn't match pattern instead of silent fallback (commit 767a53e)
