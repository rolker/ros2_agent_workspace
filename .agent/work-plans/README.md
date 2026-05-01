# Work Plans Directory

This directory contains **git-tracked work plans** for GitHub issues being actively worked on by AI agents.

## Purpose

Work plans make agent work-in-progress visible on GitHub through draft PRs. Each plan documents:
- The problem being solved
- The proposed approach
- Implementation tasks and progress
- Design decisions and rationale
- Open questions and considerations

## Workflow

### Starting Work on an Issue

1. Create worktree: `.agent/scripts/worktree_create.sh --issue <N> --type workspace`
2. Enter worktree: `source .agent/scripts/worktree_enter.sh --issue <N>`
3. Create plan directory and plan: `mkdir .agent/work-plans/issue-<N> && write .agent/work-plans/issue-<N>/plan.md`
4. Commit: `git add .agent/work-plans/ && git commit -m "Add work plan for #<N>"`
5. Push: `git push -u origin HEAD`
6. Create draft PR: `gh pr create --draft --title "..." --body-file .agent/work-plans/issue-<N>/plan.md`

### Updating Plans

As work progresses, update the plan to:
- Check off completed tasks
- Document new decisions
- Note blockers or changes in approach
- Update status

Commit plan updates regularly:
```bash
git add .agent/work-plans/issue-<number>/plan.md
git commit -m "Update work plan for #<number>"
git push
```

### Benefits

1. **Visibility**: Draft PRs show active work on GitHub
2. **Collision detection**: Branch naming prevents duplicate work
3. **Handover**: Other agents can take over using the plan
4. **Context**: Reviewers see approach before reviewing code
5. **History**: Plans are versioned alongside code changes

## Directory Layout

Each issue gets its own subdirectory: `.agent/work-plans/issue-<N>/`. The
plan itself is `issue-<N>/plan.md`. Other per-issue artifacts (progress
logs, review output, adversarial findings) live alongside it under the
same directory — one folder per issue, regardless of how many artifacts
accumulate.

- Format: `issue-<number>/plan.md`
- Example: `issue-69/plan.md` for issue #69

**Authoritative path for agents**: Create, update, and reference plans
at `.agent/work-plans/issue-<N>/plan.md`. Any older instructions that
mention only the flat `PLAN_ISSUE-<N>.md` convention should be treated
as legacy / deprecated guidance, not the current workflow.

**Legacy flat plans**: Plans created before the directory convention
landed remain at `.agent/work-plans/PLAN_ISSUE-<N>.md` for
compatibility. Each one has a matching symlink at `issue-<N>/plan.md`
so consumer skills can always look up plans at the new path. The flat
files are the canonical on-disk storage; the symlinks are the access
path.

One plan per issue. Keep plans concise but complete.

## Lifecycle

1. **Created**: When agent starts work (in draft PR)
2. **Updated**: As work progresses and tasks complete
3. **Final**: When PR is marked ready for review
4. **Merged**: Plan merges with code (permanent record)
5. **Archived**: No cleanup needed - plans serve as documentation

## Project Repos

This directory pattern is not unique to the workspace repo. Project repos
(under `layers/main/<layer>_ws/src/<project_repo>/`) maintain their own
`.agent/work-plans/` directories following the same conventions. Plans live
in whichever repo owns the issue being worked on:

- **Workspace issues** (changes to `.agent/`, `docs/`, configs, skills) →
  plan goes in the workspace repo's `.agent/work-plans/`
- **Project repo issues** (changes to ROS packages) → plan goes in that
  project repo's `.agent/work-plans/`

The workflow is the same — the only difference is which repo you're committing to.

## Notes

- Plans are **git-tracked** (visible in PRs and commits)
- Plans **should be updated** as approach evolves
- Plans **persist** after merge (useful for future reference)
- File accumulation is intentional - keeps project history

## See Also

- `.agent/templates/ISSUE_PLAN.md` - Plan template
- `.agent/WORKFORCE_PROTOCOL.md` - Multi-agent coordination
- `.agent/work-plans/issue-69/plan.md` - Example plan
