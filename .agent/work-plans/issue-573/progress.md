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
- [x] (must-fix, CI) `git notes add/append` fails without ambient git identity (GitHub runner, fresh container/host) — attestation dies after a successful run; use explicit tool identity `-c user.name=ci_local -c user.email=ci-local@localhost` — `.agent/scripts/ci_local.sh`
- [x] (low, Copilot) `--image` value written verbatim into note record; whitespace/newlines could inject a spurious `ci-local: pass` line — reject whitespace in `--image` — `.agent/scripts/ci_local.sh:76`
- [x] (low, Copilot) `REPO_NAME`/repo path embedded in docker `-v` spec and note text; whitespace/control/`:`/`,` produce ambiguous mounts or malformed records — validate early — `.agent/scripts/ci_local.sh:92`
- [x] (trivial, Copilot) unused `note_sha() { :; }` stub inside command substitution — delete — `.agent/scripts/tests/test_ci_local.sh:103`

### False positives
- (none)

## Implementation
**Status**: complete
**When**: 2026-07-22 13:30 -04:00
**By**: Claude Code Agent (Claude Fable 5)

**Commit**: `c5d7c0e` on `feature/issue-573`

All four Integrated Review findings addressed in one commit: explicit tool
identity (`ci_local` / `ci-local@localhost`) on the notes add/append path;
whitespace rejected in `--image`; whitespace/`:`/`,` rejected in repo paths
(docker `-v` spec + note-format hazard); leftover `note_sha` stub removed.
Test harness now exports `GIT_CONFIG_GLOBAL/SYSTEM=/dev/null` so every local
run reproduces the identity-less CI environment (the regression that CI
caught), with per-invocation `-c` identity on fixture commits and note
cleanup; two new validation tests. 47 checks green; full script-test suite
green.
