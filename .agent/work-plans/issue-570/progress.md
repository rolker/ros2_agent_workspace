---
issue: 570
---

# Issue #570 — review-code: default-on Local Model Adversarial Specialist (Ollama qwen3.5) with --no-local opt-out

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-16 15:58 -0400
**By**: Claude Code Agent (Claude Fable 5)
**Verdict**: approved

**Branch**: feature/issue-570 at `a7c7038`
**Mode**: pre-push
**Depth**: Standard (reason: new workspace script + governance-adjacent skill/knowledge docs)
**Must-fix**: 5 (all fixed) | **Suggestions**: 8 (all fixed)
**Round**: 1 | **Ship**: recommended — all 13 adversarial findings fixed and re-verified; static clean

**Specialists**: Static Analysis (shellcheck, full severity: clean);
Governance/consequence pass (found 2 stale specialist rosters in
knowledge docs — fixed in `d0fa8e1`); Claude Adversarial (1 fresh-context
pass, both lenses: 13 findings, several experimentally verified);
Local Adversarial (dogfooded on this branch's own diff — see below).

### Findings (all addressed in `21221a6` / `a7c7038`)
- [x] (must-fix) prompt passed as single argv arg — Linux MAX_ARG_STRLEN caps at ~128KB; large diffs never reached the model → `--rawfile` via temp file — `.agent/scripts/local_review.sh`
- [x] (must-fix) hardcoded `think:true` broke `LOCAL_REVIEW_MODEL` override for non-reasoning models → auto-retry without think on the server's 400; verified live with llama3.2:1b — `.agent/scripts/local_review.sh`
- [x] (must-fix) 5f snippet used undefined `$BASE_BRANCH` and diffed local base instead of `origin/$BASE` → matches step 1's diff now — `.claude/skills/review-code/SKILL.md`
- [x] (must-fix) no post-PR invocation path (default-on in both modes) → `gh pr diff` variant added — `.claude/skills/review-code/SKILL.md`
- [x] (must-fix) full + Light report templates had no home for the Local Adversarial skip notice → header lines added — `.claude/skills/review-code/SKILL.md`
- [x] (suggestion) `LOCAL_EXIT=$?` unreachable under errexit → `|| LOCAL_EXIT=$?` form
- [x] (suggestion) `||` handler misattributed every pipeline failure to curl → body-build and curl separated, per-cause messages
- [x] (suggestion) missing jq misdiagnosed as "model not pulled" → jq preflight, exit 2
- [x] (suggestion) exact-match model probe rejected bare names → also matches `<name>:latest`
- [x] (suggestion) silent truncation when prompt > num_ctx → fail-loud pre-check with required size
- [x] (suggestion) undocumented exit codes could escape 0/1/2 contract → guarded git/jq calls + documented catch-all
- [x] (suggestion) fixed `/tmp` stderr path in snippet → mktemp'd + trapped
- [x] (suggestion) `--help` hardcoded line range → extracts whole header block

### Dogfood calibration (Local Adversarial on its own diff)
Two failed runs were themselves the most valuable review input — each
exposed a real defect the fixes above address:
1. 600s timeout on a ~500-line diff (default was sized to the 104-line
   calibration diff) → default 900s + guidance.
2. Empty answer with `done_reason=length`: the model consumed the whole
   16k window reasoning before answering → default num_ctx 32768,
   overflow headroom 2048→12288, actionable error message.

Final validation run (fixed script, 32k ctx, full branch diff) was
interrupted by a host shutdown; rerun with:
`git diff --merge-base main HEAD | .agent/scripts/local_review.sh --context <ctx>`
The specialist's mechanics are verified end-to-end regardless (small-diff
run produced findings; llama3.2:1b fallback run produced findings).
