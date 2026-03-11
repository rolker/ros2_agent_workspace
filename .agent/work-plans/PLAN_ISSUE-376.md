# Plan: Properly handle sub-issues — parent linking, branch base, and workflow

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/376

## Context

When agents create sub-task issues during work on a parent issue, two things go wrong:
(1) the sub-issue has no reference back to the parent, and (2) the sub-issue's worktree
branches from the default branch instead of the parent's feature branch. Both problems
were observed in the rolker/unh_marine_simulation#34 → #37 workflow.

This requires changes across documentation, issue-creation tooling, and worktree-creation
tooling. The plan breaks this into three sub-issues under #376.

## Approach — Sub-issues

### Sub-issue A: Add parent-issue linking rule and sub-issue workflow to `AGENTS.md`

**Documentation changes only.** Two additions to `AGENTS.md`:

1. **Parent-linking paragraph** in Issue-First Policy (after "Trivial fixes", before
   "Verify before committing"):
   > When creating a new issue during work on or discussion of an existing issue,
   > reference the parent issue in the body (e.g., "Part of #NNN"). This applies
   > whether the new issue is in the same repo or a different one (use full
   > `owner/repo#NNN` syntax for cross-repo references). The reference must be in
   > the issue body, not just a comment — GitHub only auto-links body mentions in
   > the sidebar.

2. **Sub-issue workflow paragraph** in Issue-First Policy or Worktree Workflow explaining
   the `--parent-issue` flag and when to use it.

| File | Change |
|------|--------|
| `AGENTS.md` | Add parent-linking rule + sub-issue workflow guidance |

### Sub-issue B: Auto-inject parent reference in `gh_create_issue.sh`

**Enforcement layer.** When `$WORKTREE_ISSUE` is set (agent is in a worktree for an
existing issue), `gh_create_issue.sh` should:

1. Check if the issue body already contains a parent reference (`Part of`, `Closes`,
   `Relates to`, or `#NNN` patterns).
2. If no reference is found, append `\n\nPart of #$WORKTREE_ISSUE` to the body (or
   body-file content).
3. Print a message: `ℹ️  Auto-added parent reference: Part of #$WORKTREE_ISSUE`

This makes the documentation rule from sub-issue A mechanically enforced.

| File | Change |
|------|--------|
| `.agent/scripts/gh_create_issue.sh` | Auto-inject parent reference when `$WORKTREE_ISSUE` is set |

### Sub-issue C: Add `--parent-issue` flag to `worktree_create.sh`

**Branch-base fix.** Add `--parent-issue <N>` to `worktree_create.sh` so sub-issue
worktrees branch from the parent's feature branch instead of HEAD.

Behavior:
1. Accept `--parent-issue <N>` argument.
2. Derive the parent's branch name: `feature/issue-<N>` (matching existing convention).
3. For **workspace worktrees**: `git worktree add -b feature/issue-<CHILD> <path> feature/issue-<PARENT>`
   (fetch from origin first if needed).
4. For **layer worktrees**: same logic in the per-package worktree creation — branch from
   the parent's branch instead of `origin/$REPOS_BRANCH` or HEAD.
5. If the parent branch doesn't exist locally or on origin, warn and fall back to current
   behavior (branch from default).

| File | Change |
|------|--------|
| `.agent/scripts/worktree_create.sh` | Add `--parent-issue` flag, modify branching logic |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Enforcement over documentation | Sub-issue A is documentation; sub-issue B adds mechanical enforcement; sub-issue C prevents the wrong-branch-base problem at the tool level. Together they satisfy the enforcement hierarchy. |
| Only what's needed | Each sub-issue solves one concrete facet of the observed problem. |
| Improve incrementally | Three small, independently reviewable PRs. |
| A change includes its consequences | `AGENTS.md` change → framework adapters verified (no changes needed). Script changes → update `AGENTS.md` usage docs in sub-issue A. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0004 — Enforcement hierarchy | Yes | Instructions (sub-issue A) + script enforcement (sub-issues B, C) = multi-layer coverage. |
| 0006 — Shared AGENTS.md | Yes | Documentation changes target `AGENTS.md`, not framework adapters. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `AGENTS.md` | Framework adapters if affected | Yes — verified no changes needed |
| `.agent/scripts/worktree_create.sh` | `AGENTS.md` worktree section, `.agent/WORKTREE_GUIDE.md` | Yes — sub-issue A covers docs |
| `.agent/scripts/gh_create_issue.sh` | Script reference in `AGENTS.md` (already listed) | Yes — no change needed (script name unchanged) |

## Open Questions

1. **PR target for sub-issue worktrees**: Should the sub-issue PR target the parent's
   branch (stacked PRs) or the default branch? Stacked PRs are cleaner but GitHub's
   support is limited. Recommend: target the default branch, and note the parent PR
   in the sub-issue PR description. The branching is about starting from the right
   code, not about merge topology.

## Estimated Scope

Three sub-issues, each a single-PR change. Sub-issue A (docs) should land last since
it documents the flags added by B and C.
