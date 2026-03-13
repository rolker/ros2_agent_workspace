# Plan: Phase 1 — Review system foundation

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/396

Part of [#395 — Design: Multi-specialist local code review system](https://github.com/rolker/ros2_agent_workspace/issues/395)

## Context

The existing `review-pr` skill evaluates PRs against workspace governance
(principles, ADRs, consequences) but does not cover code quality, bugs, tests,
or ROS practices. Issue #395 designs a multi-specialist review system with a
lead reviewer orchestrating parallel specialists. This Phase 1 establishes the
foundation: lead reviewer orchestration, static analysis specialist, governance
specialist (refactored from existing `review-pr`), project context staleness
check, and a new `review-plan` skill.

## Approach

### Commit 1: Rename `review-pr` → `review-code`

Rename the skill directory and update all 10 active references (skip historical
work plans — they're records of past decisions, not living docs).

1. `mv .claude/skills/review-pr/ .claude/skills/review-code/`
2. Update `SKILL.md` frontmatter: `name: review-code`
3. Update lifecycle position lines in:
   - `.claude/skills/review-code/SKILL.md`
   - `.claude/skills/review-issue/SKILL.md`
   - `.claude/skills/plan-task/SKILL.md`
4. Update skill lists in adapters:
   - `.github/copilot-instructions.md`
   - `.agent/instructions/gemini-cli.instructions.md`
   - `.agent/AGENT_ONBOARDING.md`
5. Update `.agent/knowledge/skill_workflows.md` (diagram + table)
6. Update `.agent/scripts/README.md` and `.agent/scripts/pr_status.sh`

### Commit 2: Restructure `review-code` as lead reviewer + governance specialist

Rewrite `SKILL.md` to implement the lead reviewer pattern:

1. **Lead reviewer orchestration** (steps 1-5 in the new skill):
   - Gather context (PR diff, full files, linked issue, work plan)
   - Load `review-context.yaml` if available; warn if stale
   - Classify changed files by language and location
   - Dispatch specialists (initially: static analysis + governance)
   - Collect results, deduplicate, apply silence filter
   - Produce unified report with severity levels (must-fix / suggestion)

2. **Governance specialist** section (refactored from current steps 3-6):
   - Same principle evaluation, ADR compliance, consequence check
   - Same existing review comment check
   - Output as a specialist sub-report that the lead merges

3. **Plan drift check** (new):
   - If work plan exists, compare implementation against plan
   - Flag deviations as findings (severity: suggestion)

4. **Silence filter** description:
   - Drop findings duplicated across specialists
   - Drop nits that linters already enforce
   - If nothing meaningful remains, say "No issues found"
   - Target: >=85% of reported findings should be actionable

### Commit 3: Create static analysis specialist

Create `.agent/knowledge/review_static_analysis.md` — a reference document the
lead reviewer reads when dispatching static analysis:

1. **Tool inventory** with ament-aligned configs:
   - Python (flake8): max-line-length=99 for ROS packages, 100 for workspace;
     ament ignore list for ROS packages
   - Python (mypy): basic type checking on changed `.py` files
   - C++ (cppcheck): ament_cppcheck defaults
   - C++ (cpplint): ament_cpplint filters
   - Shell (shellcheck): severity=warning
   - YAML (yamllint): max-line-length=120
   - XML (xmllint): well-formedness check

2. **File classification rules**:
   - Files under `layers/*/src/` → ament-aligned configs
   - Files under `.agent/scripts/`, workspace root → pre-commit configs
   - Determine language from file extension

3. **Output format**: List of findings with file, line, tool, severity, message

### Commit 4: Add project context staleness check

Add a section to the lead reviewer that checks freshness:

1. If PR targets a project repo, check for `.agents/review-context.yaml`
   (or fall back to `.agents/README.md`)
2. Compare last-modified timestamps against source files in the project
3. If stale (source files newer), include warning in report header
4. Non-blocking — review proceeds with available context

### Commit 5: Create `review-plan` skill

Create `.claude/skills/review-plan/SKILL.md`:

1. **Usage**: `/review-plan <pr-number>`
2. **Lifecycle position**: review-issue → plan-task → **review-plan** → implement → review-code
3. **Steps**:
   - Read the draft PR and its plan file (`.agent/work-plans/PLAN_ISSUE-*.md`)
   - Read the linked issue and any `review-issue` comments
   - Load governance context (principles, ADRs, consequences map)
   - If project repo, load `review-context.yaml` or `.agents/README.md`
   - Evaluate plan against criteria:
     - Addresses `review-issue` findings?
     - Right files targeted? (cross-ref with project context)
     - Consequences mapped?
     - Scope appropriate? (too big → split; too vague → needs specifics)
     - ROS conventions for the change type?
     - Principle & ADR alignment?
   - Produce structured report (similar format to review-code but plan-focused)
4. **Guidelines**: Independent evaluation — the planner shouldn't grade its own work

### Commit 6: Update skill index, adapters, and cross-references

1. Update `.agent/knowledge/skill_workflows.md`:
   - New lifecycle diagram including `review-plan` and `review-code`
   - Add `review-plan` to lifecycle skills table
   - Update `review-code` description
2. Update adapter skill lists (3 files): add `review-plan`
3. Update `review-issue/SKILL.md` and `plan-task/SKILL.md` lifecycle lines
   to include `review-plan`
4. Update `triage-reviews/SKILL.md` if it references the lifecycle

### Commit 7: Side fix — update `github_metadata.json` label cache

Commit the label cache update from the #395 discussion (added `dependencies`,
`github_actions`, `proposal`, `rfc` labels).

## Files to Change

| File | Change |
|------|--------|
| `.claude/skills/review-pr/` → `.claude/skills/review-code/` | Rename directory |
| `.claude/skills/review-code/SKILL.md` | Rewrite as lead reviewer + specialists |
| `.claude/skills/review-plan/SKILL.md` | New skill |
| `.agent/knowledge/review_static_analysis.md` | New: static analysis tool configs |
| `.agent/knowledge/skill_workflows.md` | Update lifecycle diagram + tables |
| `.claude/skills/review-issue/SKILL.md` | Update lifecycle position line |
| `.claude/skills/plan-task/SKILL.md` | Update lifecycle position + guidelines |
| `.github/copilot-instructions.md` | Update skill list |
| `.agent/instructions/gemini-cli.instructions.md` | Update skill list |
| `.agent/AGENT_ONBOARDING.md` | Update skill list |
| `.agent/scripts/README.md` | Update review-pr reference |
| `.agent/scripts/pr_status.sh` | Update review-pr reference |
| `.agent/github_metadata.json` | Update label cache |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Lead reviewer produces reports for human decision — no auto-action |
| Enforcement over documentation | Static analysis specialist runs actual tools, not just docs |
| A change includes its consequences | All cross-references updated in same PR (adapters, skill index) |
| Only what's needed | Phase 1 only — LLM specialists deferred to Phase 2 |
| Improve incrementally | Builds on existing review-pr; each commit is atomic and reviewable |
| Workspace vs. project separation | Static analysis uses ament configs for project code, pre-commit for workspace |
| Primary framework first | Skill is Claude Code native; adapters point other frameworks to SKILL.md |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0003 — Project-agnostic workspace | Yes | Review system is generic ROS 2 infra, not project-specific |
| 0005 — Layered enforcement | Yes | Static analysis specialist aligns with ament linters (CI layer) |
| 0006 — Shared AGENTS.md | Yes | Skill logic in SKILL.md (shared); adapter files just list skill names |
| 0008 — Follow ROS 2 conventions | Yes | Linter configs match ament defaults; ROS practices checklist planned for Phase 3 |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| A framework skill (review-pr → review-code) | Framework adapters | Yes (commit 6) |
| Workflow skill list (add review-plan) | All adapter skill lists | Yes (commit 6) |
| Skill workflows knowledge doc | Skills that reference lifecycle | Yes (commits 1, 6) |

## Open Questions

- None — design decisions resolved in #395 discussion

## Estimated Scope

Single PR, 7 atomic commits, ~13 files changed/created.
