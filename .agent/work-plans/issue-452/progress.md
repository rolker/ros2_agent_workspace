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
- [ ] Fix `<branch-name>` ambiguity in `.claude/skills/review-code/SKILL.md` Usage block (line 16) — either drop the form or document the non-numeric-arg → `--base` mapping (R4-1)
- [ ] Make pre-push default-branch resolution prefer local `git symbolic-ref refs/remotes/origin/HEAD` over `gh repo view` for field-mode / non-`main` defaults (`.claude/skills/review-code/SKILL.md:91`) (R4-2)
- [ ] Change `mkdir .agent/work-plans/issue-<N>` to `mkdir -p ...` in `.agent/work-plans/README.md:20` for idempotency (R4-3)
- [ ] Replace `grep -oE '#[0-9]+' | head -1` linked-issue resolution in `.claude/skills/review-plan/SKILL.md:56` with `gh pr view --json closingIssuesReferences` (matches review-code's round-2 fix) (R4-4)
- [ ] Update `.agent/work-plans/PLAN_ISSUE-452.md:188` Open Questions entry — strip `--depth=` (the positional form is the final interface) (R4-5)
- [ ] Decide: keep branch-name extraction in `.claude/skills/triage-reviews/SKILL.md:195` (worktree-aligned, what progress.md location follows), or switch to `closingIssuesReferences` (PR-aligned, may decouple from worktree)? (R4-6)
