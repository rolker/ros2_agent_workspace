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

## External Review (Round 5–6)
**Status**: complete
**When**: 2026-05-15 19:10
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #453 — 2 new review round(s) since b7013e9, 14 new valid, 1 addressed by prior commit (R5-7), 0 false positives, 2 minor Copilot hallucinations noted
**CI**: all-pass

### Actions
- [x] Fixed `git symbolic-ref | sed` pipefail bug at `.claude/skills/review-code/SKILL.md:95` (R5-1 / R6-1, commit 469ade8)
- [x] Fixed layer-worktree plan lookup glob in `.claude/skills/review-plan/SKILL.md` (R5-2, commit ade3f45)
- [x] Added `[--repo <owner/repo>]` to gh examples in `.claude/skills/review-plan/SKILL.md` (R5-3, commit ade3f45)
- [x] Added PR-less variant of no-findings template in `.claude/skills/review-plan/SKILL.md` (R5-4, commit ade3f45)
- [x] Rewrote `.agent/work-plans/README.md` plan-creation step — removed `write` non-command (R5-5, commit bc2d1d1)
- [x] Narrowed `applies_to` in `.github/instructions/work-plans.instructions.md` to plan files only; rewrote prose to exclude companion artifacts (R5-6 / R6-5, commit 3f81087)
- [x] Disambiguated PR/Branch placeholders in `.claude/skills/review-code/SKILL.md` progress template (R5-8, commit 4266a65)
- [x] Fixed `framework_config.sh` path in `.agent/knowledge/review_depth_classification.md` (R5-9, commit 2b2b873)
- [x] Updated `.agent/knowledge/skill_workflows.md` — review-code is the exception to "each step is optional" (R5-10, commit db21056)
- [x] Resolved fail-loud contradiction in `.claude/skills/triage-reviews/SKILL.md` — fail-loud wins per local-first decision (R6-2, commit c049048)
- [x] Added `.github/instructions/*.md` and `.agent/AGENT_ONBOARDING.md` to workspace governance trigger list (R6-3, commit 2b2b873)
- [x] Removed "and adversarial" claim from `.github/copilot-instructions.md` pre-push paragraph (R6-4, commit 1bdc093)
- [x] Reworded "offline plan review" to "PR-less plan review" in `.claude/skills/review-plan/SKILL.md` (R6-6, commit ade3f45)

### Notes
- R5-7 (progress.md unchecked items) was self-resolving — commit b7013e9 ticked them off before this round's review-fetch could see it. Already addressed.
- R6-1 claims the symbolic-ref|sed bug also appears at lines 143 and 232 of review-code/SKILL.md, but those lines contain unrelated code (`gh pr view` and "Dispatch specialists" prose). Treat as Copilot hallucination of additional locations; only line 95 needs the fix.
- R5-5 claims the `write` issue "also appears on line 33" but line 33 is the `git add ...` block which doesn't reference `write`. Same partial-hallucination pattern.
