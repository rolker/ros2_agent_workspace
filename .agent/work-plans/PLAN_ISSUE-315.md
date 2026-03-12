# Plan: Prevent plan-task draft PRs from being confused with issues

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/315

## Context

When `plan-task` creates a draft PR for plan review, the PR number enters GitHub's
shared issue/PR namespace. Agents in later sessions can mistake the PR number for an
issue number, creating duplicate worktrees and orphaning the original issue (as happened
with #293 → PR #312 → spurious #313).

The `worktree_create.sh` script currently validates that an issue *exists* (line 564-584)
but does not check whether the number is a PR rather than an issue. The `plan-task` skill
creates draft PRs with no distinguishing title prefix. Both gaps contribute to the
confusion.

## Approach

Four targeted changes, matching the issue's acceptance criteria:

1. **Add PR-vs-issue guard to `worktree_create.sh`** — After resolving `GH_REPO_SLUG`
   (line ~560), before the title fetch, check if the number is a PR. Use `gh pr view`
   with the appropriate `--repo` flag (same as the existing `gh issue view` call).
   Exit with a clear error if the number is a PR, suggesting the source issue number.

   ```bash
   # Check if this number is actually a PR (not an issue)
   _PR_CHECK=""
   if [ -n "$GH_REPO_SLUG" ]; then
       _PR_CHECK=$(gh pr view "$ISSUE_NUM" --repo "$GH_REPO_SLUG" --json number --jq '.number' 2>/dev/null || echo "")
   else
       _PR_CHECK=$(gh pr view "$ISSUE_NUM" --json number --jq '.number' 2>/dev/null || echo "")
   fi
   if [ -n "$_PR_CHECK" ]; then
       echo "Error: #$ISSUE_NUM is a pull request, not an issue."
       echo "Use the original issue number instead."
       exit 1
   fi
   ```

2. **Add `[PLAN]` prefix to draft PR titles in `worktree_create.sh`** — In the
   `create_draft_pr()` function (line ~1050), when the PR is created via `--plan-file`,
   prefix the title with `[PLAN]`. This makes plan PRs visually distinct in the PR list.
   Add a variable `$IS_PLAN_PR` set when `--plan-file` is provided, and use it in
   `create_draft_pr()`.

3. **Add prominent issue link to plan PR body** — In the `create_draft_pr()` body
   template (lines 1023-1047), when `--plan-file` is active, start the body with
   `Plan for #<N>` instead of just `Closes #<N>`. Keep `Closes` as well so merge
   still auto-closes.

4. **Make `plan-task` skill detect and reuse existing draft PR** — Update
   `.claude/skills/plan-task/SKILL.md` step 7 to check for an existing PR on the
   branch before creating a new one. If found, update the PR body with the plan
   content using `gh pr edit`. Also add the `[PLAN]` prefix guidance to the title.

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/worktree_create.sh` | Add PR-vs-issue guard (~line 560); add `[PLAN]` title prefix and `Plan for #N` body when `--plan-file` is set |
| `.claude/skills/plan-task/SKILL.md` | Update step 7: detect existing draft PR, update instead of create; use `[PLAN]` prefix |
| `.agent/WORKTREE_GUIDE.md` | No change needed — the guide doesn't document `--plan-file` internals |
| `AGENTS.md` | No change needed — the `[PLAN]` prefix is an internal convention, not a workflow rule |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Enforcement over documentation | The PR-vs-issue guard is mechanical enforcement in the script — not just a doc warning. Core of this plan. |
| A change includes its consequences | All affected files identified. No test files exist for `worktree_create.sh` (shell script); manual verification is appropriate. |
| Only what's needed | Four targeted changes, no new files or abstractions. |
| Improve incrementally | Single PR, each change is independently valuable. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0002 — Worktree isolation | Yes | Strengthens worktree creation by rejecting invalid (PR) targets. |
| 0004 — Enforcement hierarchy | Yes | Adds script-level enforcement (local feedback layer). |
| 0005 — Layered enforcement | Yes | Script guard catches the error before any branches or directories are created. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `.agent/scripts/worktree_create.sh` | `AGENTS.md` script reference table | Not needed — no new scripts or changed interfaces |
| `.agent/scripts/worktree_create.sh` | `.agent/WORKTREE_GUIDE.md` | Not needed — `--plan-file` internals aren't documented there |
| `.claude/skills/plan-task/SKILL.md` | Framework adapter / `make generate-skills` | Not needed — skill invocation interface unchanged |

## Open Questions

- **Should the PR guard also extract and display the linked issue?** The PR body
  typically contains `Closes #N` — we could parse it and suggest the correct issue
  number. This is a nice-to-have; the basic guard is sufficient for v1.

## Estimated Scope

Single PR. All changes are in two files with straightforward edits.
