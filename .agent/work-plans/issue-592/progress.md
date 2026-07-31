---
issue: 592
---

# Issue #592 — run-issue checkpoints: require repo-qualified re-orientation context in every AskUserQuestion

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-31 11:39 -04:00
**By**: Claude Code Agent (Claude Fable 5)
**Verdict**: approved

**Branch**: feature/issue-592 at `718498a`
**Mode**: pre-push
**Depth**: Standard (reason: `.claude/skills/*/SKILL.md` governance override trigger)
**Must-fix**: 0 | **Suggestions**: 3
**Round**: 1 | **Ship**: recommended — no must-fix; suggestions are mechanical and applied in-branch

### Findings
- [x] (suggestion) Pin "repo slug" to the GitHub repo name, not the local directory name (dir `project11` ≠ repo `ros2_agent_workspace`) — `.claude/skills/run-issue/SKILL.md:272` (fixed in-branch)
- [x] (suggestion) triage-reviews inline header template paraphrases the canonical format (`<phase>` vs `phase X of Y (<phase name>)`) — drift seed, cross-confirmed by all 3 specialists — `.claude/skills/triage-reviews/SKILL.md:388` (fixed in-branch: matches canonical verbatim)
- [ ] (suggestion) No mechanical enforcement layer for the header (ADR-0004/0005 Watch): a PreToolUse hook could assert the `<repo>#<N>` prefix — backlog note, not this PR — `.claude/skills/run-issue/SKILL.md:271`
