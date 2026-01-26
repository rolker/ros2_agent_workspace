---
name: Issue First Policy
description: Mandates that all code changes must be tracked by a GitHub Issue.
---

# Issue First Policy

**Rule**: No Code Without a Ticket.

## Core Principle
Agents should not begin coding until a GitHub Issue number is assigned and known.

## Exceptions
- Trivial "drive-by" fixes (typos, minor documentation updates).
- Urgent hotfixes (though an issue should be created retrospectively).

## Implementation
1.  **Check First**: Before starting any task, search for an existing issue.
2.  **Ask**: If no issue exists, ask the user: *"Should I open an issue to track this?"*
3.  **Link**: The `implementation_plan.md` (if used) must link to the Issue.
4.  **Branch**: Feature branches must follow `feature/ISSUE-<number>-<description>`.

## Why?
- **Tracking**: Ensures all work is visible on the roadmap.
- **Context**: Provides a single source of truth for "why" a change was made.
- **Memory**: Prevents context loss between agent sessions.
