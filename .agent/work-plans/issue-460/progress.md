---
issue: 460
---

# Issue #460 — review-code: add --skip-static / --no-progress / --issue overrides + distinct pre-push progress header

## External Review
**Status**: complete
**When**: 2026-05-18 16:30
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #462 — 2 review(s), 3 valid, 5 plan-file/low-priority
**CI**: all-pass (8 checks)

### Actions
- [ ] Fix SKILL.md wording inconsistency: line 520 ("Progress persistence skipped (no linked issue)") vs line 585 ("Findings not persisted: no linked issue") — pick one canonical Summary string.
- [ ] Add `--skip-static` skip status line to Light condensed format (line 480–496) and No-findings format (line 498–506) so SA-skipped runs are never silent.
- [ ] Split progress.md template snippet at line 549–552 into two mode-tagged blocks (post-PR vs pre-push) for copy/paste safety.
- [ ] (Optional) Dismiss the four plan-file Copilot comments — implementation supersedes plan structure.
