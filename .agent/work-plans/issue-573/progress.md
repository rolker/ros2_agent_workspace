---
issue: 573
---

# Issue #573 — ci_local.sh: containerized local CI runner with git-note attestation (Phase 1 of #572)

## Integrated Review
**Status**: complete
**When**: 2026-07-22 13:15 -04:00
**By**: Claude Code Agent (Claude Fable 5)

**PR**: #574 at `d44e7b3`
**Sources**: 3 (Copilot R1 @ `d44e7b3`, CI rollup @ `d44e7b3`, session pre-push workflow review — 10 findings all fixed pre-push, not re-listed)
**Cross-source confirmations**: 0 strict (Copilot's two attestation-format comments thematically continue the pre-push review's attestation-integrity findings, distinct locations)
**CI**: failures-noted — Script tests failed ×2 (real, finding 1); all other checks pass

### Findings
- [ ] (must-fix, CI) `git notes add/append` fails without ambient git identity (GitHub runner, fresh container/host) — attestation dies after a successful run; use explicit tool identity `-c user.name=ci_local -c user.email=ci-local@localhost` — `.agent/scripts/ci_local.sh`
- [ ] (low, Copilot) `--image` value written verbatim into note record; whitespace/newlines could inject a spurious `ci-local: pass` line — reject whitespace in `--image` — `.agent/scripts/ci_local.sh:76`
- [ ] (low, Copilot) `REPO_NAME`/repo path embedded in docker `-v` spec and note text; whitespace/control/`:`/`,` produce ambiguous mounts or malformed records — validate early — `.agent/scripts/ci_local.sh:92`
- [ ] (trivial, Copilot) unused `note_sha() { :; }` stub inside command substitution — delete — `.agent/scripts/tests/test_ci_local.sh:103`

### False positives
- (none)
