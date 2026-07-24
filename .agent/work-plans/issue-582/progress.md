---
issue: 582
---

# Issue #582 — make sync: throttle should be off by default, enabled only when the remote drops connections

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-23 22:59 -0400
**By**: Claude Code Agent (Claude Fable 5)
**Verdict**: approved

**Branch**: feature/issue-582 at `b0ebe6f`
**Mode**: pre-push
**Depth**: Standard (reason: 156 changed lines, 50–199 band)
**Must-fix**: 0 | **Suggestions**: 4 (2 applied in-branch)
**Round**: 1 | **Ship**: recommended — no must-fix findings; both actionable suggestions applied before push

### Findings
- [x] (suggestion) negative --throttle silently disabled adaptive pacing — `sync_repos.py` argparse (fixed: parser.error)
- [x] (suggestion) module-level flag never reset at main() entry — `sync_repos.py`/`lib/remote_utils.py` (fixed: reset_transient_error_seen() at entry)
- [ ] (suggestion) sync failures still exit 0 — pre-existing; raised odds in firewall scenario noted in PR body, not changed here
- [ ] (suggestion) push_remote/pull_remote set but never read the adaptive flag — scoped gap noted in PR body
