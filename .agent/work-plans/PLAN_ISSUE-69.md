# Work Plan: Issue #69 - Agent Work Visibility

**Issue**: #69  
**Title**: Make sure it's easy to discover that an agent started work on an issue  
**Assignee**: Copilot CLI Agent  
**Started**: 2026-01-27  
**Status**: 🚀 Ready for Review

---

## Problem Analysis

Currently, when an AI agent starts working on a GitHub issue, there's no visible indicator on GitHub that work is in progress. This creates several problems:

1. **No visibility**: Other agents/users can't tell if an issue is being worked on
2. **No context**: Can't see the agent's approach or plan without local access
3. **No handover**: Difficult for another agent to take over if needed
4. **Race conditions**: Multiple agents might pick up the same issue

The issue suggests committing plans to `.agent/scratchpad/` and creating draft PRs, but also asks to consider alternatives.

## Proposed Approach

Implement a **draft PR workflow** where agents:

1. **On issue start**:
   - Create feature branch `feature/ISSUE-<number>-<description>`
   - Generate structured plan in `.agent/work-plans/PLAN_ISSUE-{number}.md` (git-tracked)
   - Commit and push the plan
   - Create **draft PR** linking to the issue

2. **During work**:
   - Update plan as tasks complete
   - Commit updates to show progress
   - Plan serves as handover documentation

3. **On completion**:
   - Mark PR as ready for review
   - Ensure plan reflects final state

**Why git-tracked plans?**
- Visible in PR diff/commits
- Not cluttered (one file per issue in dedicated directory)
- Enables review of approach before code
- Remains visible in the PR history and in git as long as the branch/PR/commit stays referenced (e.g., if merged or tagged)

**Alternative considered**: `.agent/scratchpad/` (git-ignored) - rejected because plans wouldn't be visible in PR without extra tooling.

## Implementation Tasks

### Phase 1: Core Infrastructure ✅
- [x] Create `.agent/work-plans/` directory
- [x] Create plan template: `.agent/templates/ISSUE_PLAN.md`
- [x] Create script: `.agent/scripts/start_issue_work.sh`
  - Accepts issue number
  - Fetches issue metadata from GitHub
  - Creates feature branch
  - Generates plan file from template in `.agent/work-plans/`
  - Prints manual next steps for committing, pushing, and creating a draft PR
- [x] Create script: `.agent/scripts/update_issue_plan.sh`
  - Updates plan file
  - Commits with standardized message
  - Pushes update

### Phase 2: Workflow Integration ✅
- [x] Update `.agent/workflows/dev/start-feature.md`
  - Add section on starting work from an issue
  - Document when to use `start_issue_work.sh`
- [x] Update `.agent/workflows/dev/submit-pr.md`
  - Add step to convert draft → ready for review
  - Include plan update verification
- [x] Update `.agent/WORKFORCE_PROTOCOL.md`
  - Add "Work Visibility Protocol" section
  - Document draft PR convention
  - Explain plan handover mechanism

### Phase 3: Documentation & Guidance ✅
- [x] Add `.agent/work-plans/README.md` explaining the directory
- [x] Update `.agent/AI_RULES.md` - Added to WORKFORCE_PROTOCOL instead
- [ ] Update `.agent/AI_CLI_QUICKSTART.md` (if applicable)
  - Add example of starting work on an issue
- [ ] Consider creating `.agent/skills/work-tracking/SKILL.md`
  - Document best practices
  - Examples of good plans
  - Handover procedures

### Phase 4: Testing & Validation ✅
- [x] Use workflow on Issue #69 itself (dogfooding)
- [x] Verify draft PR visibility on GitHub UI
- [x] Confirm plan updates are visible in PR commits
- [x] Address PR review feedback
- [ ] Test with a second issue to verify repeatability
- [ ] Test handover scenario (if possible)

### Phase 5: Cleanup & Polish 🚧
- [x] Add `.agent/work-plans/README.md` explaining the directory
- [x] Update `.gitignore` if needed (work-plans is tracked - no change needed)
- [x] Add AI signature to PR body when creating draft
- [ ] Document edge cases (multi-repo issues, plan conflicts)

## Design Decisions

### ✅ Git-tracked plans in `.agent/work-plans/`
**Rationale**: Plans are visible in PR, reviewable, and don't clutter root directory.

### ✅ Draft PR immediately after plan commit
**Rationale**: Makes work visible on GitHub without waiting for code changes.

### ✅ One plan file per issue
**Rationale**: Simple, predictable, easy to find. No need for complex indexing.

### ✅ Scripts are helpers, not requirements
**Rationale**: Start as recommended tooling; can enforce later if adopted well.

## Open Questions

1. **Multi-repo issues**: If work spans multiple repos, where does the plan live?
   - **Initial answer**: Primary repo being modified; reference others in plan
   
2. **Plan updates**: Auto-commit on every change or manual commits?
   - **Initial answer**: Manual - agent decides when plan updates are significant
   
3. **Plan format**: Strict template or flexible markdown?
   - **Initial answer**: Template with required sections, flexible content

## Success Criteria

- [x] This plan is visible in a draft PR on GitHub
- [ ] Helper scripts reduce friction to create plans
- [ ] Other agents can discover active work by browsing draft PRs
- [ ] Plans provide enough context for handover
- [ ] Workflow documentation is clear and actionable

## Notes

- This issue is self-referential - implementing the workflow by using it
- Low risk: purely additive changes, no modification to existing code
- High value: enables better multi-agent coordination
- Aligns with WORKFORCE_PROTOCOL.md principles

---

**🤖 Authored-By**: `Copilot CLI Agent`
