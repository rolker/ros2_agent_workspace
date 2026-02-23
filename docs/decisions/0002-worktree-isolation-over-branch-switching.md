# ADR-0002: Worktree Isolation Over Branch Switching

## Status

Accepted

## Context

AI coding agents working in a shared repository cause problems when they switch branches:
uncommitted changes are discarded, builds interfere with each other, test results
overwrite each other, and lock contention slows everyone down. The same problems affect
human developers working on multiple tasks, but agents make it worse because they work
faster and don't notice the side effects.

The workspace tried documentation-only solutions first ("never switch branches"), but
agents don't reliably follow written rules. A mechanical enforcement was needed.

## Decision

All feature work uses git worktrees — separate working directory checkouts that each have
their own branch, build artifacts, and uncommitted changes, while sharing the same git
history.

Enforcement:
- A shell wrapper blocks `git checkout` for branch switching (while still allowing
  `git checkout -- <file>` for restoring files)
- Worktree lifecycle is managed through dedicated scripts for creation, entry, listing,
  and removal
- Two worktree types: `workspace` (for workspace infrastructure) and `layer` (for ROS 2
  package work, using a hybrid structure where modified packages are git worktrees and
  unmodified packages are symlinks)

## Consequences

**Positive:**
- True parallel development — multiple tasks in progress without interference
- No risk of discarding uncommitted work when switching context
- Enables multi-agent workflows where different agents work on different issues
  simultaneously
- The guardrail is mechanical, not just documented — agents can't bypass it without
  deliberately circumventing the shell wrapper

**Negative:**
- More disk usage (each worktree is a separate checkout, though symlinks for unmodified
  packages mitigate this)
- Slightly more complex workflow than simple branch switching
- Worktrees share ports, databases, and ROS 2 DDS discovery — true isolation requires
  containers (tracked separately as a future enhancement)
