# Plan: Import field changes from gitcloud

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/432

## Context

Field machines push changes to gitcloud during testing. These need to flow
back to GitHub for review. Manual imports are error-prone (wrong topics,
missing tests, violated worktree rules). Today's session (2026-04-20) proved
the PR review step catches real issues — but the ceremony of per-repo
worktree creation was heavy enough to tempt shortcuts.

## Approach

Phase 1 only (happy path). Phases 2-3 deferred to follow-up issues.

1. **Add `--fetch-only` flag to `pull_remote.py`** — currently default mode
   fetches and reports. Formalize this as an explicit flag that returns
   structured output (JSON) listing repos with ahead commits, commit count,
   and whether diverged.

2. **Create `import-field-changes` skill** — batch skill that:
   a. Calls `pull_remote.py --fetch-only --remote <name>` (default: gitcloud)
   b. For each repo with changes: examines diff, generates issue body with
      commit summary and pre-review findings
   c. Creates issue in the project repo via `gh_create_issue.sh`
   d. Creates feature branch from gitcloud HEAD (or notes divergence)
   e. Pushes branch, opens draft PR referencing issue
   f. Reports summary to user

3. **Pre-review integration** — skill examines each diff against Quality
   Standard criteria (tests present? topic names match remaps? idempotent?)
   and includes findings in the issue body.

4. **Divergence handling (Phase 1 scope)** — detect and report. Create
   worktree for diverged repos (merge needed). Do not auto-merge in Phase 1;
   report to user for manual resolution or follow-up.

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/pull_remote.py` | Add `--fetch-only` with JSON output mode |
| `.agent/project_config.yaml` | New gitignored config file (document in setup) |
| `.gitignore` | Add `.agent/project_config.yaml` |
| `.claude/skills/import-field-changes/` | New skill directory |
| `.claude/skills/import-field-changes/content.md` | Skill definition |
| `AGENTS.md` | Add skill to reference if needed |
| `.github/copilot-instructions.md` | Add skill to adapter list |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Skill reports findings; human decides whether to merge PRs or request fixes |
| Enforcement over documentation | Automates the "treat field imports as drafts" Quality Standard directive |
| Only what's needed | Phase 1 only — no auto-fix, no force-push resync |
| Improve incrementally | Single skill + minor script extension; builds on existing infrastructure |
| Workspace vs. project separation | Skill is generic; issues created in project repos |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0002 — Worktree isolation | Yes | Diverged repos get worktrees; non-diverged repos only create branches (no main-tree editing) |
| 0003 — Project-agnostic | Yes | Skill works for any remote name, any repo |
| 0006 — Shared AGENTS.md | Yes | Skill added to reference tables in adapters |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `pull_remote.py` flags | Script reference table in AGENTS.md | Yes |
| Add new skill | Framework adapter skill lists | Yes |

## Decisions

- **Remote name**: read from `.agent/project_config.yaml` (gitignored, project-specific).
  No hardcoded default. Error with helpful message if file missing or key absent.
  ```yaml
  # .agent/project_config.yaml
  field_remote: gitcloud
  ```
- **Issue title convention**: `Field import: <repo> (YYYY-MM-DD)` — one issue per
  repo, predictable and searchable.
- **Resync**: deferred to Phase 2. After PRs merge, normal `push_remote.py` handles
  non-diverged repos. Force-push resync is a separate concern.

## Open Questions

None — all design questions resolved.

## Estimated Scope

Single PR for Phase 1. Follow-up issues for: Phase 2 (auto-fix in worktree +
resync), ADR for field branch conventions.
