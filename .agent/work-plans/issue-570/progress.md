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

## Integrated Review
**Status**: complete
**When**: 2026-07-24 08:38 -0400
**By**: Claude Code Agent (Claude Fable 5)

**PR**: #571 at `86f26e1`
**Sources**: 3 (Copilot R1 @ `86f26e1`, Local Review (Pre-Push) @ `a7c7038`, CI rollup)
**Cross-source confirmations**: 0 (Copilot's finding refines the guard the
pre-push review's suggestion #10 introduced — related lineage, different
claims at different heads, so not a formal cross-confirmation)
**CI**: all-pass

### Findings
- [x] (medium, Copilot) Context-overflow guard's `~3 bytes/token` estimate is
  not conservative for dense tokenization; silent *input* truncation (model
  confidently reviews a tail fragment) can slip past both the pre-check and
  the `done_reason=length` post-check — `.agent/scripts/local_review.sh:189`.
  Scope verified: at default 32k ctx the slip requires <~1.9 bytes/token
  (pathological but possible — byte-BPE fallback on unusual bytes), and the
  guard *weakens as `LOCAL_REVIEW_NUM_CTX` grows* because the 12288-token
  headroom is fixed while the byte budget scales — and the overflow error
  message itself steers users to raise num_ctx. Root-cause fix: post-response
  check on the API's `prompt_eval_count` — fail loud when it reaches the
  num_ctx ceiling (actual measured tokens, no bytes/token guessing);
  optionally soften the "conservative" claim in the comment.

### False positives
- (Copilot, partial) "may ... still hit `done_reason=length` behavior" — the
  empty-answer/`done_reason=length` case is already caught loud post-hoc
  (`local_review.sh:257-264`, exit 1 with a raise-num_ctx message); only the
  silent-input-truncation half of the claim stands (tracked above).

## Implementation
**Status**: complete
**When**: 2026-07-24 08:46 -0400
**By**: Claude Code Agent (Claude Fable 5)

**PR**: #571 at `947465d`
**Addressed**: Integrated Review @ 2026-07-24 08:38 -0400 (PR #571 at `86f26e1`)
**Commits**: `947465d`

### Actions
- [x] Silent-input-truncation gap in the context-overflow guard — added a
  post-response check on the server's measured `prompt_eval_count` (fail
  loud, exit 1, when ingested tokens reach the num_ctx ceiling), softened
  the "conservative" claim in the pre-check comment, and documented the
  backstop in the header/--help block. Verified both the guard path
  (mock server reporting ceiling ingestion → exit 1 with actionable
  message) and the happy path (normal ingestion → review printed,
  exit 0) — `.agent/scripts/local_review.sh:189`

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-24 13:01 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-570 at `c7d6354`
**Mode**: pre-push
**Depth**: Standard (reason: governance files — SKILL.md, knowledge docs, AGENTS.md — plus a new workspace script)
**Must-fix**: 1 | **Suggestions**: 0
**Round**: 2 | **Ship**: recommended — single low, mechanical must-fix; count not rising vs. round 1's 5; address and ship

**Specialists**: Static Analysis (shellcheck --severity=warning: clean);
Governance/consequence pass (script→AGENTS.md row, 5f→roster docs,
--no-local→templates all present and consistent); Claude Adversarial
(2 disjoint-lens fresh-context passes — Lens A logic, Lens B
systemic/safety); Local Adversarial skipped (no Ollama server at
localhost:11434); Copilot off (default). Plan Drift skipped (no plan.md).

### Findings
- [x] (must-fix) mid-answer truncation silently accepted as complete — `done_reason == "length"` is only checked when `$CONTENT` is empty, so a non-empty answer truncated mid-stream (content + `done_reason: length`) prints as a clean review and exits 0; guard it independent of content emptiness — `.agent/scripts/local_review.sh:282`

## Implementation
**Status**: complete
**When**: 2026-07-24 09:55 -0400
**By**: Claude Code Agent (Claude Fable 5)

**PR**: #571 at `67b8ea5`
**Addressed**: Local Review (Pre-Push) @ 2026-07-24 13:01 +00:00 (round 2, `c7d6354`)
**Commits**: `67b8ea5`

### Actions
- [x] Mid-answer truncation silently accepted as complete — `done_reason`
  is now checked independent of content emptiness: `length` with
  non-empty content fails loud ("answer was cut off mid-stream ... a
  partial findings list is not trustworthy"), `length` with empty
  content keeps the reasoning-exhaustion message, and the plain
  empty-answer check follows. Verified all three response shapes
  against mock servers (mid-truncation → exit 1, empty+length →
  exit 1, normal → exit 0) — `.agent/scripts/local_review.sh`
