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
- [ ] Fix: replace the `*"github.com"*` substring test + `sed` slug extraction in `git_bug_setup.sh` `setup_repo_gitbug()` (around line 101) with `extract_gh_slug` from `.agent/scripts/_worktree_helpers.sh`. Source the helper after `SCRIPT_DIR` is defined; use a non-empty slug to gate bridge configuration and derive `owner`/`repo`. This prevents lookalike-host / path-injection URLs (e.g. gitcloud URLs containing `github.com`, `mygithub.com`) from configuring a GitHub bridge against the wrong repo, consistent with the field-mode allowlist (`github.com`, `ssh.github.com`).
