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
- [ ] Fix `git symbolic-ref | sed` pipefail bug at `.claude/skills/review-code/SKILL.md:95` — pipeline exits with sed's status, so when `git symbolic-ref` fails the fallback chain is silently skipped and `DEFAULT_BRANCH` becomes empty. Use `REMOTE_HEAD=$(git symbolic-ref ...) && echo "${REMOTE_HEAD#refs/remotes/origin/}"` instead (R5-1 / R6-1)
- [ ] Fix layer-worktree plan lookup in `.claude/skills/review-plan/SKILL.md:87` — current glob `layers/worktrees/*/issue-*-<N>/.agent/work-plans/...` misses project-repo plans which live at `<layer>_ws/src/<project_repo>/.agent/work-plans/...` (R5-2)
- [ ] Add `[--repo <owner/repo>]` to `gh pr view` / `gh issue view` examples in `.claude/skills/review-plan/SKILL.md:53,60,104` — `--repo` is advertised but examples don't show its use (R5-3)
- [ ] Add PR-less variant of no-findings template in `.claude/skills/review-plan/SKILL.md:220-225` — currently uses `## Plan Review: PR #<N>` header which is wrong for `--issue`/file-path modes (R5-4)
- [ ] Replace `write .agent/work-plans/.../plan.md` in `.agent/work-plans/README.md:20` — `write` is a Unix messaging command, not a file-creation command. Rewrite step 3 to separate `mkdir -p` from "create plan.md using your editor/Write tool" (R5-5)
- [ ] Narrow `applies_to` in `.github/instructions/work-plans.instructions.md:5` from `.agent/work-plans/**` to plan files specifically, OR rewrite prose to distinguish plan files from companion artifacts (progress.md, review output, adversarial findings) (R5-6 / R6-5)
- [ ] Disambiguate `<#N>` placeholder in `.claude/skills/review-code/SKILL.md:504` template — currently same `<N>` used for issue path and PR ref. Use `<#PR-or-branch>` or split into separate placeholders (R5-8)
- [ ] Fix `framework_config.sh` path in `.agent/knowledge/review_depth_classification.md:130` — should be `.agent/scripts/framework_config.sh` to match actual location and other entries' path conventions (R5-9)
- [ ] Update `.agent/knowledge/skill_workflows.md:13` — "Each step is optional" contradicts AGENTS.md Post-Task Verification step 5 which makes `/review-code` expected pre-push. Qualify or restructure (R5-10)
- [ ] Resolve fail-loud contradiction in `.claude/skills/triage-reviews/SKILL.md`: line 210-220 says fail loud on branch-name mismatch, line 268-270 (preexisting) says skip persistence silently. Pick one. Per local-first direction, fail-loud wins — delete or rewrite the skip-persistence text (R6-2)
- [ ] Add `.github/instructions/*.md` and `.agent/AGENT_ONBOARDING.md` to workspace governance trigger list in `.agent/knowledge/review_depth_classification.md` (R6-3)
- [ ] Remove "and adversarial" from `.github/copilot-instructions.md:27` — pre-push pass cannot catch Adversarial findings because Adversarial is Claude-only (per caveat at line 36) (R6-4)
- [ ] Reword "offline plan review" in `.claude/skills/review-plan/SKILL.md:28` to "PR-less plan review" — `gh issue view` is still called in step 2, so it's not truly offline. Either reword, or add offline fallback using plan's embedded issue context (R6-6)

### Notes
- R5-7 (progress.md unchecked items) was self-resolving — commit b7013e9 ticked them off before this round's review-fetch could see it. Already addressed.
- R6-1 claims the symbolic-ref|sed bug also appears at lines 143 and 232 of review-code/SKILL.md, but those lines contain unrelated code (`gh pr view` and "Dispatch specialists" prose). Treat as Copilot hallucination of additional locations; only line 95 needs the fix.
- R5-5 claims the `write` issue "also appears on line 33" but line 33 is the `git add ...` block which doesn't reference `write`. Same partial-hallucination pattern.
