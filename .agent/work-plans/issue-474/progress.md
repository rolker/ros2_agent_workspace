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
