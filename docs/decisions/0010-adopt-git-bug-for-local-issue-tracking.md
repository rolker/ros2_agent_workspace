# ADR-0010: Adopt git-bug for local issue tracking

## Status

Proposed

## Context

AI agents working in this workspace depend on `gh` API calls for issue lookups
(titles, states, issue validation). This introduces latency on every worktree
creation and entry, requires network connectivity, and counts against GitHub API
rate limits. In offline or rate-limited scenarios, agents lose the ability to
verify issue context entirely.

The workspace already has patterns for graceful degradation (e.g., dashboard
skips GitHub sections in quick mode), but issue metadata is fundamental to the
worktree workflow — every `worktree_create.sh` and `worktree_enter.sh` invocation
queries GitHub.

git-bug (v0.10.1) is a distributed bug tracker embedded in git. It stores issue
data as git objects, enabling offline access with sub-millisecond lookups. Its
GitHub bridge provides bidirectional sync, keeping local data current with the
remote issue tracker.

Identity strategy per #353: agents use a single git identity for both git
commits and git-bug operations, avoiding multi-identity complexity.

## Decision

**Adopt git-bug v0.10.1 as the local-first issue cache with GitHub bridge sync.**

Key choices:

- **Binary install in bootstrap**: Pin version, download from GitHub releases,
  install to `/usr/local/bin/`. Follow existing `DRY_RUN`/`PENDING_COMMANDS`
  pattern.
- **Setup script** (`git_bug_setup.sh`): Idempotent bridge configuration —
  identity creation, GitHub bridge setup using `gh auth token`, initial sync.
  Failures warn but don't block (graceful degradation).
- **Sync integration**: `git bug pull` / `git bug push` added to `sync_repos.py`,
  triggered alongside repository sync.
- **Lookup priority**: Scripts try git-bug first (fast, offline-capable), fall
  back to `gh` API if git-bug is unavailable or returns no data.
- **Makefile stamp**: `$(STAMP)/git-bug.done` depends on bootstrap, with
  `skip-git-bug` escape hatch for environments that don't need it.

## Consequences

**Positive:**
- Issue lookups become near-instant and work offline
- Reduces GitHub API usage (rate limit pressure, especially in CI)
- Graceful degradation — all git-bug integrations fall back to `gh` when
  git-bug is not installed or not configured
- `skip-git-bug` target allows opting out without affecting other setup

**Negative:**
- New binary dependency (~15 MB) downloaded during bootstrap
- Additional stamp in the Makefile setup chain
- Scripts gain a second lookup path (git-bug → gh fallback), increasing
  code complexity slightly
- Bridge sync adds a few seconds to `make sync` and `dashboard`
