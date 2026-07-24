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

## Integrated Review
**Status**: complete
**When**: 2026-07-23 23:06 -0400
**By**: Claude Code Agent (Claude Fable 5)

**PR**: #583 at `0e48ff0`
**Sources**: 3 (Copilot @ `0e48ff0`, Local Review (Pre-Push) @ `b0ebe6f`, CI rollup)
**Cross-source confirmations**: 1
**CI**: all-pass

### Findings
- [ ] (cross-confirmed: Local Review negative-check + Copilot non-finite) `--throttle nan` silently disables all pacing and `inf` hangs time.sleep — extend guard to reject non-finite values — `.agent/scripts/sync_repos.py:233`
- [ ] (suggestion, Copilot) update rejection test to cover nan/inf and the new error text — `.agent/scripts/tests/test_net_retry.py:242`
- [ ] (suggestion, Local Review, deferred-by-design) sync failures still exit 0 — pre-existing, documented in PR body
- [ ] (suggestion, Local Review, deferred-by-design) push_remote/pull_remote set but don't read the adaptive flag — documented in PR body

### False positives
- none — both Copilot comments valid
