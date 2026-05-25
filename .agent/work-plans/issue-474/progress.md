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
- [x] Fix (commit `9acc75a`): `git_bug_setup.sh:158` smoke-test count `_COUNT=$(… | grep -c . || echo "0")` double-printed when there are zero open issues — `grep -c` prints `0` and exits 1, so `|| echo "0"` also ran and `_COUNT` became `"0\n0"`, garbling the success message. Replaced `|| echo "0"` with `|| true` (grep -c already prints the count).
- [x] (addressed) Stale review @ `63a6c6e`: `github.com` substring lookalike-host concern — already fixed by the `extract_gh_slug` change in `69baebe` (this round's head).

## External Review
**Status**: complete
**When**: 2026-05-25 17:05 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #475 at `a0ad09a`
**Reviews**: 3 (all Copilot bot); 2 inline comments, both **already addressed** at head; 0 false positives
**CI**: all pass (Validate Documentation, Validate commit identity Mechanism C, Lint pre-commit)

### Actions
- [x] (addressed) `github.com` substring host-detection could misclassify a field/gitcloud or lookalike URL → refactored to the anchored `extract_gh_slug` validator (rejects lookalike hosts + `github.com`-in-path) — `git_bug_setup.sh`
- [x] (addressed) `grep -c . || echo "0"` double-printed `0\n0` → changed to `|| true` (clean `0`) — `git_bug_setup.sh`

No outstanding actions; PR clean, CI green, mergeable. (Triaged by another agent on the author's behalf — author was occupied.)
