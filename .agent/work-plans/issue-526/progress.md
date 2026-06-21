---
issue: 526
---

# Issue #526 — dispatch: --repo-slug + fail-loud on cross-repo issue-# worktree collision

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-21 02:29 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-526 at `0932448`
**Mode**: pre-push
**Depth**: Standard (reason: 132 lines changed; scripts take input + shell out)
**Must-fix**: 0 | **Suggestions**: 3
**Round**: 1 | **Ship**: recommended — no must-fix; tested change, fail-loud over silent-guess; one consistency suggestion that fails cleanly today.

### Findings
- [ ] (suggestion) Sanitize `--repo-slug` (`sed 's/[^A-Za-z0-9_]/_/g'`) to match sibling worktree scripts; hyphenated slug currently misses (fails cleanly) — `dispatch_subagent.sh:245`, `docker_run_agent.sh:243`
- [ ] (suggestion, optional) Pre-existing: validate `$ISSUE` numeric to close traversal class with #1 — `dispatch_subagent.sh:203`, `docker_run_agent.sh:159`
- [ ] (suggestion, optional) Add `--repo-slug` to the `dispatch_subagent.sh` row in AGENTS.md Script Reference
