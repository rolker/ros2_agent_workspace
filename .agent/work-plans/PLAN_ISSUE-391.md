# Plan: Resurrect 4 Deleted Skills

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/391

## Context

During a February 2026 cleanup (commit `0a95cfa`), 9 skills were deleted from
`.agent/skills/`. Four fill gaps in the current skill set and should be brought
back, updated for current conventions: skills now live in `.claude/skills/`,
use current frontmatter format, and reference existing knowledge/template files
instead of duplicating content.

The originals are preserved in git history at commits `ba5cb5f` (skill-importer,
issue-triage) and `0a95cfa~1` (test-engineering, ros-documentation,
ros-code-documentation).

## Approach

1. **Create `skill-importer/SKILL.md`** — Adapt from `ba5cb5f`. Drop references
   to deleted scripts (`init_skill.py`, `quick_validate.py`, `package_skill.py`).
   Inline validation criteria. Update target path to `.claude/skills/`. Keep the
   quality evaluation framework (reject/merge/modify/accept).

2. **Create `document-package/SKILL.md`** — Merge `ros-documentation` and
   `ros-code-documentation` into one skill. Reference (not duplicate)
   `.agent/knowledge/documentation_verification.md` and
   `.agent/templates/package_documentation.md`. Cover both README-level node
   docs and API-level code docs in a single workflow.

3. **Create `issue-triage/SKILL.md`** — Adapt from `ba5cb5f`. Replace GitHub
   MCP dependency with `gh` CLI. Use `list_overlay_repos.py` for repo
   enumeration. Remove roadmap references (`.agent/ROADMAP.md` no longer
   exists). Add stale issue detection and categorization. Omit git-bug
   (deferred to follow-up issue).

4. **Create `test-engineering/SKILL.md`** — Adapt from `0a95cfa~1`. Reference
   existing templates at `.agent/templates/testing/` instead of embedding them.
   Reference `ros2launch_session` for integration tests. Reference worktree
   `build.sh`/`test.sh` for build/test commands.

5. **Update `.agent/knowledge/skill_workflows.md`** — Add all 4 skills to
   the utility skills table.

6. **Update adapter skill lists** — Add the 4 new skill names to the
   "Available workflow skills" line in `.github/copilot-instructions.md`,
   `.agent/instructions/gemini-cli.instructions.md`, and
   `.agent/AGENT_ONBOARDING.md` (per consequences map).

## Files to Change

| File | Change |
|------|--------|
| `.claude/skills/skill-importer/SKILL.md` | Create — adapted from git history |
| `.claude/skills/document-package/SKILL.md` | Create — merged from two deleted skills |
| `.claude/skills/issue-triage/SKILL.md` | Create — adapted from git history |
| `.claude/skills/test-engineering/SKILL.md` | Create — adapted from git history |
| `.agent/knowledge/skill_workflows.md` | Update utility skills table |
| `.github/copilot-instructions.md` | Add 4 skills to skill list |
| `.agent/instructions/gemini-cli.instructions.md` | Add 4 skills to skill list |
| `.agent/AGENT_ONBOARDING.md` | Add 4 skills to skill list |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Only what's needed | Each skill fills a distinct gap not covered by existing skills. Git-bug deferred. |
| A change includes its consequences | Adapter files and skill index updated in the same PR. |
| Workspace vs. project separation | All skills are generic ROS 2 tooling, not project-specific. |
| Improve incrementally | Resurrecting proven skills with targeted updates, not building from scratch. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0002 — Worktree isolation | Yes | Working in worktree `issue-workspace-391` |
| 0003 — Project-agnostic workspace | Yes | Skills reference generic patterns, not project-specific content |
| 0006 — Shared AGENTS.md | Watch | Skills are Claude-specific; shared skill index updated; adapter lists updated |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Framework skills (`.claude/skills/`) | Adapter files | Yes — step 6 |
| Workflow skill list | Non-Claude adapters | Yes — step 6 |

## Open Questions

None — scope is clear and all referenced files exist.

## Estimated Scope

Single PR with 8 files (4 new, 4 updated).
