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

**Manual approach:**
1. Create feature branch: `git checkout -b feature/ISSUE-<number>-<description>`
2. Copy plan template: `cp .agent/templates/ISSUE_PLAN.md .agent/work-plans/PLAN_ISSUE-<number>.md`
3. Fill in the plan with your approach
4. Commit: `git add .agent/work-plans/ && git commit -m "docs: Add work plan for Issue #<number>"`
5. Push: `git push -u origin HEAD`
6. Create draft PR: `gh pr create --draft --title "..." --body "Closes #<number>"`

**Helper script approach:**
```bash
# Creates branch, generates plan from template
.agent/scripts/start_issue_work.sh <issue_number> "Agent Name"

# Then edit the plan file and commit it
```

### Updating Plans

As work progresses, update the plan to:
- Check off completed tasks
- Document new decisions
- Note blockers or changes in approach
- Update status

Commit plan updates regularly:
```bash
# Helper script
.agent/scripts/update_issue_plan.sh <issue_number> "Optional commit message"

# Or manually
git add .agent/work-plans/PLAN_ISSUE-<number>.md
git commit -m "docs: Update work plan - completed Phase 1"
git push
```

### Benefits

1. **Visibility**: Draft PRs show active work on GitHub
2. **Collision detection**: Branch naming prevents duplicate work
3. **Handover**: Other agents can take over using the plan
4. **Context**: Reviewers see approach before reviewing code
5. **History**: Plans are versioned alongside code changes

## File Naming Convention

- Format: `PLAN_ISSUE-{number}.md`
- Example: `PLAN_ISSUE-69.md` for issue #69

One plan per issue. Keep plans concise but complete.

## Lifecycle

1. **Created**: When agent starts work (in draft PR)
2. **Updated**: As work progresses and tasks complete
3. **Final**: When PR is marked ready for review
4. **Merged**: Plan merges with code (permanent record)
5. **Archived**: No cleanup needed - plans serve as documentation

## Notes

- Plans are **git-tracked** (visible in PRs and commits)
- Plans **should be updated** as approach evolves
- Plans **persist** after merge (useful for future reference)
- File accumulation is intentional - keeps project history

## See Also

- `.agent/templates/ISSUE_PLAN.md` - Plan template
- `.agent/WORKFORCE_PROTOCOL.md` - Multi-agent coordination
- `.agent/work-plans/PLAN_ISSUE-69.md` - Example plan
