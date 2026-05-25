---
issue: 474
---

# Issue #474 — git_bug_setup: identity setup didn't cover overlay project repos

## External Review
**Status**: complete
**When**: 2026-05-25 (time n/a)
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #475 — 1 review, 1 valid, 0 false positives
**CI**: all-pass

### Actions
- [x] Fix (commit `de72b54`): replaced the `*"github.com"*` substring test + `sed` slug extraction in `git_bug_setup.sh` `setup_repo_gitbug()` with `extract_gh_slug` from `.agent/scripts/_worktree_helpers.sh`. Sourced the helper after `SCRIPT_DIR` is defined; a non-empty slug gates bridge configuration and supplies `owner`/`repo`. Smoke-tested: real GitHub forms (https/scp/ssh-over-443) resolve; gitcloud field origin, `mygithub.com`, and `github.com`-in-path all return empty (bridge skipped). Consistent with the field-mode allowlist (`github.com`, `ssh.github.com`).

## External Review
**Status**: complete
**When**: 2026-05-25 16:20 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #475 at `69baebe`
**Reviews**: 2 review(s) (1 stale @ `63a6c6e`, 1 fresh @ `69baebe`), 1 valid, 0 false positives
**CI**: all-pass

### Actions
- [ ] Fix (valid, Copilot @ `69baebe`): `git_bug_setup.sh:158` smoke-test count `_COUNT=$(… | grep -c . || echo "0")` double-prints when there are zero open issues — `grep -c` prints `0` and exits 1, so `|| echo "0"` also runs and `_COUNT` becomes `"0\n0"`, garbling the success message. Common case (fresh setup). Fix: drop `|| echo "0"` (grep -c already prints the count); use `|| true` to neutralize the exit code.
- [x] (addressed) Stale review @ `63a6c6e`: `github.com` substring lookalike-host concern — already fixed by the `extract_gh_slug` change in `69baebe` (this round's head).
